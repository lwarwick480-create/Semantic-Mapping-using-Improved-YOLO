#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>
#include <cmath>
  
using std::placeholders::_1;
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

class SemanticMapNode : public rclcpp::Node
{
public:
    SemanticMapNode() : Node("semantic_map_node")
    {
        // 创建TF监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建订阅者和发布者
        subscription_ = this->create_subscription<PointCloud2>(
            "/plane_points", 10,
            std::bind(&SemanticMapNode::pointCloudCallback, this, _1));

        auto marker_qos = rclcpp::QoS(10).transient_local();
        marker_pub_ = this->create_publisher<MarkerArray>(
            "/semantic_markers", marker_qos);

        // 设置标记的基本属性
        marker_id_ = 0;
        last_stable_time_ = this->now();
        is_first_cloud_ = true;
        
        // 初始化存储所有标记的容器
        all_markers_ = std::make_shared<MarkerArray>();
    }

private:
    bool isCloudStable(const PointCloud2& current_cloud)
    {
        if (is_first_cloud_) {
            last_cloud_ = current_cloud;
            is_first_cloud_ = false;
            last_stable_time_ = this->now();
            return false;
        }

        // 检查点云大小是否相似
        if (std::abs(static_cast<int>(current_cloud.width * current_cloud.height) - 
                    static_cast<int>(last_cloud_.width * last_cloud_.height)) > 50) {
            last_cloud_ = current_cloud;
            last_stable_time_ = this->now();
            return false;
        }

        // 使用常量迭代器来读取点云数据
        sensor_msgs::PointCloud2ConstIterator<float> current_x(current_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> current_y(current_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> current_z(current_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> last_x(last_cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> last_y(last_cloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> last_z(last_cloud_, "z");

        float max_diff = 0.0;
        int points_checked = 0;
        const int check_interval = 10;

        for (; current_x != current_x.end() && last_x != last_x.end(); 
             ++current_x, ++current_y, ++current_z, ++last_x, ++last_y, ++last_z) {
            
            if (points_checked++ % check_interval != 0) continue;

            float diff = std::sqrt(
                std::pow(*current_x - *last_x, 2) +
                std::pow(*current_y - *last_y, 2) +
                std::pow(*current_z - *last_z, 2)
            );
            max_diff = std::max(max_diff, diff);

            if (max_diff > 0.05) {
                last_cloud_ = current_cloud;
                last_stable_time_ = this->now();
                return false;
            }
        }

        rclcpp::Duration stable_duration = this->now() - last_stable_time_;
        if (stable_duration.seconds() >= 3.0) {
            return true;
        }

        return false;
    }

    void pointCloudCallback(const PointCloud2::SharedPtr cloud_msg)
    {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform("map", cloud_msg->header.frame_id,
                                          tf2::TimePointZero);

            PointCloud2 cloud_transformed;
            tf2::doTransform(*cloud_msg, cloud_transformed, transform_stamped);

            if (!isCloudStable(cloud_transformed)) {
                return;
            }

            MarkerArray new_markers;

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transformed, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_transformed, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_transformed, "z");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_semantic(cloud_transformed, "semantic");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_semantic)
            {
                Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.ns = "semantic_objects";
                marker.id = marker_id_++;
                marker.type = Marker::SPHERE;
                marker.action = Marker::ADD;

                marker.pose.position.x = *iter_x;
                marker.pose.position.y = *iter_y;
                marker.pose.position.z = *iter_z;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                uint32_t semantic_id = *iter_semantic;
                RCLCPP_INFO(this->get_logger(), "Semantic ID: %d at [%.2f, %.2f, %.2f]", 
                           semantic_id, *iter_x, *iter_y, *iter_z);
                
                switch(semantic_id) {
                    case 1:  // box
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        break;
                    case 2:  // table
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        break;
                    default:
                        marker.color.r = 0.5;
                        marker.color.g = 0.5;
                        marker.color.b = 0.5;
                }
                marker.color.a = 1.0;
                marker.lifetime = rclcpp::Duration::from_seconds(0);

                new_markers.markers.push_back(marker);
                all_markers_->markers.push_back(marker);  // 添加到累积的标记中
            }

            if (!new_markers.markers.empty()) {
                marker_pub_->publish(*all_markers_);  // 发布所有累积的标记
            }

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
    int marker_id_;
    
    PointCloud2 last_cloud_;
    rclcpp::Time last_stable_time_;
    bool is_first_cloud_;
    std::shared_ptr<MarkerArray> all_markers_;  // 新增：存储所有标记的容器
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticMapNode>());
    rclcpp::shutdown();
    return 0;
}
