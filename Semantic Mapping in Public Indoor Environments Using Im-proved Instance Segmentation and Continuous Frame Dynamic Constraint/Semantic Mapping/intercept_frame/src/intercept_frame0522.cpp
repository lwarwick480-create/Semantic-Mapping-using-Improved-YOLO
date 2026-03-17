#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <mutex>

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
            int center_x = static_cast<int>(detection.bbox.center.position.x);
            int center_y = static_cast<int>(detection.bbox.center.position.y);
            int width = static_cast<int>(detection.bbox.size.x);
            int height = static_cast<int>(detection.bbox.size.y);

            int start_x = std::max(0, center_x - width / 2);
            int start_y = std::max(0, center_y - height / 2);
            int crop_width = std::min(width, static_cast<int>(msg->width) - start_x);
            int crop_height = std::min(height, static_cast<int>(msg->height) - start_y);

            for (int y = 0; y < crop_height; ++y)
            {
                int input_row = (start_y + y) * msg->row_step;

                for (int x = 0; x < crop_width; ++x)
                {
                    int input_index = input_row + (start_x + x) * msg->point_step;
                    merged_data.insert(
                        merged_data.end(),
                        msg->data.begin() + input_index,
                        msg->data.begin() + input_index + msg->point_step
                    );
                }
            }
        }

        // 设置合并后的输出点云属性
        output.width = merged_data.size() / msg->point_step;
        output.height = 1;  // 设为无组织点云
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

