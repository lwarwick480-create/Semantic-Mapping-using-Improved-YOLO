#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <unordered_map>

class PointCloudCropper : public rclcpp::Node
{
public:
    PointCloudCropper() : Node("pointcloud_cropper_with_semantic")
    {
        detection_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
            "/yolo/detections", 10,
            std::bind(&PointCloudCropper::detection_callback, this, std::placeholders::_1));

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudCropper::pointcloud_callback, this, std::placeholders::_1));


        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/intercept_points", 10);

        // 自定义语义标签对应整数 ID
        class_to_id_ = {
            {"evacuation_box", 0},
            {"fire_extinguisher", 1},
            {"fire_hydrant", 2},
            {"rubbish_bin", 3},
            {"door", 4}
        };
    }

private:
    std::mutex mutex_;
    std::vector<yolo_msgs::msg::Detection> detections_;
    std::unordered_map<std::string, uint32_t> class_to_id_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    void detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        detections_ = msg->detections;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (detections_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No detections, skip cropping.");
            return;
        }

        std::vector<uint8_t> merged_data;

        for (const auto& detection : detections_)
        {
            const auto& mask = detection.mask;
            if (class_to_id_.find(detection.class_name) == class_to_id_.end()) {
                RCLCPP_WARN(this->get_logger(), "Unknown class: %s", detection.class_name.c_str());
                continue;
            }
            uint32_t semantic_id = class_to_id_[detection.class_name];

            std::vector<cv::Point> contour;
            for (const auto& pt : mask.data) {
                contour.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
            }

            cv::Mat binary_mask(mask.height, mask.width, CV_8UC1, cv::Scalar(0));
            std::vector<std::vector<cv::Point>> contours = { contour };
            cv::fillPoly(binary_mask, contours, cv::Scalar(255));

            for (int y = 0; y < msg->height; ++y)
            {
                for (int x = 0; x < msg->width; ++x)
                {
                    if (x >= binary_mask.cols || y >= binary_mask.rows)
                        continue;

                    if (binary_mask.at<uchar>(y, x) == 0)
                        continue;

                    int point_index = y * msg->row_step + x * msg->point_step;

                    float px, py, pz;
                    uint32_t rgb;
                    memcpy(&px,  &msg->data[point_index + 0], sizeof(float));
                    memcpy(&py,  &msg->data[point_index + 4], sizeof(float));
                    memcpy(&pz,  &msg->data[point_index + 8], sizeof(float));
                    memcpy(&rgb, &msg->data[point_index + 16], sizeof(uint32_t)); // offset 16

                    float camera_to_base = 0.14;
                    float height_in_base = py + camera_to_base;
                    if (py > 0.07 ) continue;
                    if (py < -0.63) continue;

                    // 构造自定义点结构：x y z rgb semantic_id（20 字节）
                    uint8_t buffer[20];
                    memcpy(buffer + 0,  &px, sizeof(float));
                    memcpy(buffer + 4,  &py, sizeof(float));
                    memcpy(buffer + 8,  &pz, sizeof(float));
                    memcpy(buffer + 12, &rgb, sizeof(uint32_t));
                    memcpy(buffer + 16, &semantic_id, sizeof(uint32_t));

                    merged_data.insert(merged_data.end(), buffer, buffer + 20);
                }
            }
        }

        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;
        output.height = 1;
        output.width = merged_data.size() / 20;
        output.is_bigendian = false;
        output.point_step = 20;
        output.row_step = output.width * output.point_step;
        output.is_dense = false;
        output.data = std::move(merged_data);

        // 定义字段结构：x y z rgb semantic
        output.fields.resize(5);
        output.fields[0].name = "x";        output.fields[0].offset = 0;  output.fields[0].datatype = 7; output.fields[0].count = 1;
        output.fields[1].name = "y";        output.fields[1].offset = 4;  output.fields[1].datatype = 7; output.fields[1].count = 1;
        output.fields[2].name = "z";        output.fields[2].offset = 8;  output.fields[2].datatype = 7; output.fields[2].count = 1;
        output.fields[3].name = "rgb";      output.fields[3].offset = 12; output.fields[3].datatype = 7; output.fields[3].count = 1;
        output.fields[4].name = "semantic"; output.fields[4].offset = 16; output.fields[4].datatype = 6; output.fields[4].count = 1;  // UINT32

        pub_->publish(output);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudCropper>());
    rclcpp::shutdown();
    return 0;
}

