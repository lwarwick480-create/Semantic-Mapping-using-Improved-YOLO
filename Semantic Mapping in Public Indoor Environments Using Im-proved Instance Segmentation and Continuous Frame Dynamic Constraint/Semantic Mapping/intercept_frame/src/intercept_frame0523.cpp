#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>


class PointCloudCropper : public rclcpp::Node
{
public:
    PointCloudCropper() : Node("pointcloud_cropper_from_yolo")
    {
        detection_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
            "/yolo/detections", 10,
            std::bind(&PointCloudCropper::detection_callback, this, std::placeholders::_1));

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points", 10,
            std::bind(&PointCloudCropper::pointcloud_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/intercept_points", 10);
    }

private:
    std::mutex mutex_;
    std::vector<yolo_msgs::msg::Detection> detections_;

    void detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        detections_ = msg->detections;  // 记录全部检测结果（可以为空）
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (detections_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No detections, skip cropping.");
            return;  // 没有检测，不发布
        }

        // 创建输出点云容器，稍后合并多个裁剪区域
        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;
        output.fields = msg->fields;
        output.is_bigendian = msg->is_bigendian;
        output.point_step = msg->point_step;
        output.is_dense = msg->is_dense;

        std::vector<uint8_t> merged_data;

        for (const auto& detection : detections_)
{
    const auto& mask = detection.mask;
    std::vector<cv::Point> contour;

    for (const auto& pt : mask.data) {
        contour.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
    }

    // 创建一个全零的 mask 图像
    cv::Mat binary_mask(mask.height, mask.width, CV_8UC1, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> contours = { contour };
    cv::fillPoly(binary_mask, contours, cv::Scalar(255));

    // 遍历所有点云像素（或裁剪区域内）
    for (int y = 0; y < msg->height; ++y)
    {
        for (int x = 0; x < msg->width; ++x)
        {
            if (x >= binary_mask.cols || y >= binary_mask.rows)
                continue;

            if (binary_mask.at<uchar>(y, x) == 0)
                continue;

            int point_index = y * msg->row_step + x * msg->point_step;
            merged_data.insert(
                merged_data.end(),
                msg->data.begin() + point_index,
                msg->data.begin() + point_index + msg->point_step
            );
        }
    }
}



        // 设置合并后的输出点云属性
        output.width = merged_data.size() / msg->point_step;
        output.height = 1;
        output.row_step = output.width * output.point_step;
        output.data = std::move(merged_data);
        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudCropper>());
    rclcpp::shutdown();
    return 0;
}

