#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <string>

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

class SemanticClusteringNode : public rclcpp::Node
{
public:
    SemanticClusteringNode() : Node("semantic_clustering_node")
    {
        marker_sub_ = this->create_subscription<MarkerArray>(
            "/semantic_markers", rclcpp::QoS(10).transient_local(),
            std::bind(&SemanticClusteringNode::markerCallback, this, std::placeholders::_1));

        clustered_marker_pub_ = this->create_publisher<MarkerArray>(
            "/clustered_semantic_markers", rclcpp::QoS(10).transient_local());

        clustering_distance_ = this->declare_parameter("clustering_distance", 0.3);
    }

private:
    void markerCallback(const MarkerArray::SharedPtr msg)
    {
        if (msg->markers.empty()) return;

        // 1. 按类别分组
        std::unordered_map<int, std::vector<size_t>> semantic_groups;
        for (size_t i = 0; i < msg->markers.size(); ++i) {
            int color_hash = getColorHash(msg->markers[i].color);
            semantic_groups[color_hash].push_back(i);
        }

        // 2. 对每个类别聚类
        std::vector<int> point_object_ids(msg->markers.size(), -1);
        std::vector<std::string> object_labels;
        int object_count = 0;
        std::unordered_map<int, int> class_counts;

        for (const auto& group : semantic_groups) {
            std::vector<size_t> indices = group.second;
            std::vector<bool> processed(indices.size(), false);

            for (size_t i = 0; i < indices.size(); ++i) {
                if (processed[i]) continue;
                std::vector<size_t> cluster;
                cluster.push_back(indices[i]);
                processed[i] = true;

                for (size_t j = 0; j < indices.size(); ++j) {
                    if (processed[j]) continue;
                    if (calculateDistance(msg->markers[indices[i]].pose.position, msg->markers[indices[j]].pose.position) <= clustering_distance_) {
                        cluster.push_back(indices[j]);
                        processed[j] = true;
                    }
                }

                // 给该聚类分配object_id和label
                int object_id = ++object_count;
                std::string object_type = getObjectTypeFromColor(msg->markers[indices[i]].color);
                std::string label = object_type + "_" + std::to_string(++class_counts[group.first]);
                object_labels.push_back(label);

                for (auto idx : cluster) {
                    point_object_ids[idx] = object_id;
                }

                // 打印聚类信息
                RCLCPP_INFO(this->get_logger(), "聚类出物体 %s (object_id=%d), 点数: %zu", label.c_str(), object_id, cluster.size());
            }
        }

        // 3. 构造输出MarkerArray
        MarkerArray clustered_markers;
        for (size_t i = 0; i < msg->markers.size(); ++i) {
            Marker marker = msg->markers[i];
            int object_id = point_object_ids[i];
            marker.ns = object_id > 0 && object_id <= (int)object_labels.size() ? object_labels[object_id-1] : "unknown";
            auto color = getRandomColor(object_id);
            marker.color.r = color.r;
            marker.color.g = color.g;
            marker.color.b = color.b;
            marker.color.a = 1.0;
            marker.action = Marker::ADD; // 保证为ADD
            marker.lifetime = rclcpp::Duration::from_seconds(0); // 永久显示
            clustered_markers.markers.push_back(marker);

            if (i % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "marker[%zu] 位置[%.2f, %.2f, %.2f] 属于物体 %s (object_id=%d)",
                    i, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
                    marker.ns.c_str(), object_id);
            }
        }

        clustered_marker_pub_->publish(clustered_markers);
        RCLCPP_INFO(this->get_logger(), "输出MarkerArray %zu 个marker，每个marker已分配物体标签和颜色", clustered_markers.markers.size());
    }

    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        return std::sqrt(
            std::pow(p1.x - p2.x, 2) +
            std::pow(p1.y - p2.y, 2) +
            std::pow(p1.z - p2.z, 2)
        );
    }

    int getColorHash(const std_msgs::msg::ColorRGBA& color)
    {
        return static_cast<int>(color.r * 255) << 16 |
               static_cast<int>(color.g * 255) << 8 |
               static_cast<int>(color.b * 255);
    }

    std::string getObjectTypeFromColor(const std_msgs::msg::ColorRGBA& color)
    {
        if (color.r > 0.9 && color.g < 0.1 && color.b < 0.1) return "evacuation_box";
        if (color.g > 0.9 && color.r < 0.1 && color.b < 0.1) return "fire_extinguisher";
        if (color.b > 0.9 && color.r < 0.1 && color.g < 0.1) return "fire_hydrant";
        return "unknown";
    }

    std_msgs::msg::ColorRGBA getRandomColor(int seed)
    {
        static std::vector<std::tuple<float, float, float>> palette = {
            {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
            {1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0},
            {0.5, 0.5, 0.0}, {0.5, 0.0, 0.5}, {0.0, 0.5, 0.5}
        };
        std_msgs::msg::ColorRGBA color;
        if (seed > 0)
        {
            auto [r, g, b] = palette[(seed-1) % palette.size()];
            color.r = r; color.g = g; color.b = b; color.a = 1.0;
        }
        else
        {
            color.r = color.g = color.b = 0.5; color.a = 1.0;
        }
        return color;
    }

    rclcpp::Subscription<MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr clustered_marker_pub_;
    double clustering_distance_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticClusteringNode>());
    rclcpp::shutdown();
    return 0;
}
