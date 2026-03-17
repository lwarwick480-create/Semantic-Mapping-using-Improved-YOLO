#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <cmath>
#include <unordered_map>
  
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

        trajectory_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/trajectory_node_list", 10,
            std::bind(&SemanticMapNode::trajectoryCallback, this, _1));

        auto marker_qos = rclcpp::QoS(10).transient_local();
        marker_pub_ = this->create_publisher<MarkerArray>(
            "/semantic_markers", marker_qos);

        // 设置标记的基本属性
        marker_id_ = 0;
        last_stable_time_ = this->now();
        is_first_cloud_ = true;
        
        // 初始化存储容器
        all_markers_ = std::make_shared<MarkerArray>();
        marker_positions_ = std::make_shared<std::unordered_map<int, geometry_msgs::msg::Point>>();
        last_update_pose_ = nullptr;
    }

private:
    void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr trajectory_msg)
    {
        if (trajectory_msg->poses.empty() || all_markers_->markers.empty()) {
            return;
        }

        const auto& latest_pose = trajectory_msg->poses.back();
        
        if (shouldUpdateMarkers(latest_pose)) {
            updateMarkerPositions(latest_pose);
            marker_pub_->publish(*all_markers_);
        }
    }

    bool shouldUpdateMarkers(const geometry_msgs::msg::PoseStamped& current_pose)
    {
        if (!last_update_pose_) {
            last_update_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(current_pose);
            return false;
        }

        double dx = current_pose.pose.position.x - last_update_pose_->pose.position.x;
        double dy = current_pose.pose.position.y - last_update_pose_->pose.position.y;
        double distance_change = std::sqrt(dx * dx + dy * dy);

        if (distance_change > 0.05) {  // 5cm threshold
            last_update_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(current_pose);
            return true;
        }
        return false;
    }

    void updateMarkerPositions(const geometry_msgs::msg::PoseStamped& current_pose)
    {
        for (auto& marker : all_markers_->markers) {
            if (marker_positions_->find(marker.id) == marker_positions_->end()) {
                (*marker_positions_)[marker.id] = marker.pose.position;
                continue;
            }

            const auto& initial_pos = (*marker_positions_)[marker.id];

            marker.pose.position.x = initial_pos.x + 
                (current_pose.pose.position.x - last_update_pose_->pose.position.x);
            marker.pose.position.y = initial_pos.y + 
                (current_pose.pose.position.y - last_update_pose_->pose.position.y);
            
            (*marker_positions_)[marker.id] = marker.pose.position;
        }
    }

    bool isCloudStable(const PointCloud2& current_cloud)
    {
        RCLCPP_INFO(this->get_logger(), "in isCloudStable");
        if (is_first_cloud_) {
            last_cloud_ = current_cloud;
            is_first_cloud_ = false;
            last_stable_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "aaaaaaaaa");
            return false;
            

        }
        
        if (std::abs(static_cast<int>(current_cloud.width * current_cloud.height) - 
                    static_cast<int>(last_cloud_.width * last_cloud_.height)) > 50) {
            last_cloud_ = current_cloud;
            last_stable_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "bbbbbbb");
            return false;
            

        }

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
            RCLCPP_INFO(this->get_logger(), "Point cloud movement too large: max_diff = %.3f", max_diff);

            if (max_diff > 0.05) {
            	  RCLCPP_INFO(this->get_logger(), "Point cloud movement too large: max_diff = %.3f", max_diff);
                last_cloud_ = current_cloud;
                last_stable_time_ = this->now();
                return false;
                RCLCPP_INFO(this->get_logger(), "ccccc");

            }
        }

        rclcpp::Duration stable_duration = this->now() - last_stable_time_;
        RCLCPP_INFO(this->get_logger(), stable_duration);
        if (stable_duration.seconds() >= 3.0) {
            RCLCPP_INFO(this->get_logger(), "dddddd");
            return true;
            

        }
        RCLCPP_INFO(this->get_logger(), "eeeee");

        return false;
        

    }

    void pointCloudCallback(const PointCloud2::SharedPtr cloud_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "I have get topic!!");
        
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform("map", cloud_msg->header.frame_id,
                                          tf2::TimePointZero);

            PointCloud2 cloud_transformed;
            tf2::doTransform(*cloud_msg, cloud_transformed, transform_stamped);
            
            
            RCLCPP_INFO(this->get_logger(), "first!!");

            if (!isCloudStable(cloud_transformed)) {
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "seccond!!");

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
                
                
                if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                        RCLCPP_WARN(this->get_logger(), 
                           "Skipping invalid point: x=%f, y=%f, z=%f", 
                           *iter_x, *iter_y, *iter_z);
                    continue;  // 跳过非法值
                }
                
                RCLCPP_INFO(this->get_logger(), "third!!");

                marker.pose.position.x = *iter_x;
                marker.pose.position.y = *iter_y;
                marker.pose.position.z = *iter_z;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                uint32_t semantic_id = *iter_semantic;
                RCLCPP_INFO(this->get_logger(), "semantic_id: %u", semantic_id);
                
                switch(semantic_id) {
                    case 0:  
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        break;
                    case 1:  
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        break;
                    case 2:  
                        marker.color.r = 0.0;
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

                new_markers.markers.push_back(marker);
                all_markers_->markers.push_back(marker);
            }

            if (!new_markers.markers.empty()) {
                marker_pub_->publish(*all_markers_);
            }

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
    int marker_id_;
    
    PointCloud2 last_cloud_;
    rclcpp::Time last_stable_time_;
    bool is_first_cloud_;
    std::shared_ptr<MarkerArray> all_markers_;
    std::shared_ptr<std::unordered_map<int, geometry_msgs::msg::Point>> marker_positions_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> last_update_pose_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticMapNode>());
    rclcpp::shutdown();
    return 0;
}
