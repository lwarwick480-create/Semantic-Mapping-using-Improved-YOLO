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
        
        if (std::abs(static_cast<int>(current_cloud.width * current_cloud.height) - 
                    static_cast<int>(last_cloud_.width * last_cloud_.height)) > 50) {
            last_cloud_ = current_cloud;
            last_stable_time_ = this->now();
            return false;
        }

        sensor_msgs::PointCloud2ConstIterator<float> current_x(current_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> current_y(current_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> current_z(current_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> last_x(last_cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> last_y(last_cloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> last_z(last_cloud_, "z");

        float sum_diff = 0.0;
        int points_checked = 0;
        const int check_interval = 10;
        int valid_points = 0;

        for (; current_x != current_x.end() && last_x != last_x.end(); 
             ++current_x, ++current_y, ++current_z, ++last_x, ++last_y, ++last_z) {
            if (points_checked++ % check_interval != 0) continue;
            float diff = std::sqrt(
                std::pow(*current_x - *last_x, 2) +
                std::pow(*current_y - *last_y, 2) +
                std::pow(*current_z - *last_z, 2)
            );
            sum_diff += diff;
            ++valid_points;
        }

        float avg_diff = valid_points > 0 ? sum_diff / valid_points : 0.0;
        if(std::isnan(avg_diff)){
           avg_diff = 0.2;
        }
        
        if (avg_diff > 0.4) {
            last_cloud_ = current_cloud;
            last_stable_time_ = this->now();
            return false;
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
            // 转换点云到map坐标系
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform("map", cloud_msg->header.frame_id,
                                          tf2::TimePointZero);

            PointCloud2 cloud_transformed;
            tf2::doTransform(*cloud_msg, cloud_transformed, transform_stamped);

            // 检查点云是否稳定
            if (!isCloudStable(cloud_transformed)) {
                return;
            }
            
            is_first_cloud_ = true;

            // 如果点云稳定，发布markers
            MarkerArray marker_array;

            // 创建新的markers
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transformed, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_transformed, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_transformed, "z");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_semantic(cloud_transformed, "semantic");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_semantic)
            {
                if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                    continue; // 跳过非法值
                }
                
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

                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;

                uint32_t semantic_id = *iter_semantic;
                switch(semantic_id) {
                    case 0:  // evacuation_box - 红色
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        break;
                    case 1:  // fire_extinguisher - 绿色
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        break;
                    case 2:  // fire_hydrant - 蓝色
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                        break;
                    case 3:  // rubbish_bin - 黄色
                        marker.color.r = 1.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        break;
                    case 4:  // door - 紫色
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                        break;
                    default:
                        marker.color.r = 0.5;
                        marker.color.g = 0.5;
                        marker.color.b = 0.5;
                }
                marker.color.a = 1.0;
                marker.lifetime = rclcpp::Duration::from_seconds(0);

                marker_array.markers.push_back(marker);
            }

            if (!marker_array.markers.empty()) {
                marker_pub_->publish(marker_array);
            }

        } catch (tf2::TransformException &ex) {
            // Transform error
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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticMapNode>());
    rclcpp::shutdown();
    return 0;
}
