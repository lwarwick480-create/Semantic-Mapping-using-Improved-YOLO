#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <string>
#include <random>

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

class SemanticClusteringNode : public rclcpp::Node
{
public:
    SemanticClusteringNode() : Node("semantic_clustering_node")
    {
        marker_sub_ = this->create_subscription<MarkerArray>(
            "/loaded_semantic_markers", rclcpp::QoS(10).transient_local(),
            std::bind(&SemanticClusteringNode::markerCallback, this, std::placeholders::_1));

        clustered_marker_pub_ = this->create_publisher<MarkerArray>(
            "/clustered_semantic_markers", rclcpp::QoS(10).transient_local());

        clustering_distance_ = this->declare_parameter("clustering_distance", 2.0);
    }

private:
    void markerCallback(const MarkerArray::SharedPtr msg)
    {
        if (msg->markers.empty()) return;

        // 1. 按类别分组 (根据颜色Hash)
        std::unordered_map<int, std::vector<size_t>> semantic_groups;
        for (size_t i = 0; i < msg->markers.size(); ++i) {
            int color_hash = getColorHash(msg->markers[i].color);
            semantic_groups[color_hash].push_back(i);
        }

        // 存储每个点属于哪个物体ID
        std::vector<int> point_object_ids(msg->markers.size(), -1);
        
        // 存储每个物体ID对应的原始颜色（用于保持原色）
        std::unordered_map<int, std_msgs::msg::ColorRGBA> object_original_colors;
        
        // 存储每个物体ID对应的最终标签名
        std::unordered_map<int, std::string> object_final_labels;
        
        // 存储每个物体ID是否需要变色（如果有多个同类物体，除了第一个外都需要变色）
        std::unordered_map<int, bool> object_needs_color_change;

        int global_object_count = 0;

        // 2. 对每个颜色组进行空间聚类
        for (const auto& group : semantic_groups) {
            std::vector<size_t> indices = group.second;
            std::vector<bool> processed(indices.size(), false);
            int clusters_in_this_group = 0;
            
            // 获取该组的原始颜色
            std_msgs::msg::ColorRGBA original_color = msg->markers[indices[0]].color;
            std::string base_type_name = getObjectTypeFromColor(original_color);

            // 遍历组内所有点
            for (size_t i = 0; i < indices.size(); ++i) {
                if (processed[i]) continue;

                // 发现新聚类
                clusters_in_this_group++;
                global_object_count++;
                int current_object_id = global_object_count;
                
                // 记录该物体的原始颜色
                object_original_colors[current_object_id] = original_color;
                
                // 命名与变色逻辑：
                // 第1个物体 -> 名字: "door", 变色: false
                // 第2个物体 -> 名字: "door_1", 变色: true
                // 第3个物体 -> 名字: "door_2", 变色: true
                if (clusters_in_this_group == 1) {
                    object_final_labels[current_object_id] = base_type_name;
                    object_needs_color_change[current_object_id] = false;
                } else {
                    object_final_labels[current_object_id] = base_type_name + "_" + std::to_string(clusters_in_this_group - 1);
                    object_needs_color_change[current_object_id] = true;
                }

                // 开始聚类搜索 (BFS/DFS)
                std::vector<size_t> cluster;
                cluster.push_back(indices[i]);
                processed[i] = true;
                
                // 简单的聚类实现：寻找所有距离在阈值内的点
                // 注意：这里用的是简单的O(N^2)逻辑，对于大量点可能较慢，但对于Marker通常够用
                // 更好的做法是使用KD-Tree，但为了保持代码简单且不依赖PCL，这里维持原逻辑并优化结构
                
                // 使用队列进行扩展搜索，以支持链式连接的物体
                std::vector<size_t> queue;
                queue.push_back(indices[i]);
                
                size_t head = 0;
                while(head < queue.size()){
                    size_t curr_idx = queue[head++];
                    
                    for (size_t j = 0; j < indices.size(); ++j) {
                        if (processed[j]) continue;
                        
                        double dist = calculateDistance(
                            msg->markers[curr_idx].pose.position, 
                            msg->markers[indices[j]].pose.position
                        );
                        
                        if (dist <= clustering_distance_) {
                            processed[j] = true;
                            cluster.push_back(indices[j]);
                            queue.push_back(indices[j]);
                        }
                    }
                }

                // 将聚类结果写入 point_object_ids
                for (auto idx : cluster) {
                    point_object_ids[idx] = current_object_id;
                }
                
                // 识别物体类型并打印日志
                std::string object_type = getObjectTypeFromColor(original_color);
                RCLCPP_INFO(this->get_logger(), "Found object: %s (ID: %d), Points: %zu, Group Cluster Index: %d", 
                    object_type.c_str(), current_object_id, cluster.size(), clusters_in_this_group);
            }
        }

        // 3. 构造输出 MarkerArray
        MarkerArray clustered_markers;
        for (size_t i = 0; i < msg->markers.size(); ++i) {
            Marker marker = msg->markers[i];
            int object_id = point_object_ids[i];
            
            if (object_id != -1) {
                // 设置命名空间
                marker.ns = object_final_labels[object_id];
                
                // 颜色处理逻辑
                if (object_needs_color_change[object_id]) {
                    auto random_color = getRandomColor(object_id * 12345); // 使用ID作为种子
                    marker.color = random_color;
                } else {
                    marker.color = object_original_colors[object_id];
                }
                
                // 确保不透明
                if (marker.color.a < 0.1) marker.color.a = 1.0;
            }
            
            clustered_markers.markers.push_back(marker);
        }

        clustered_marker_pub_->publish(clustered_markers);
        RCLCPP_INFO(this->get_logger(), "Published %zu clustered markers.", clustered_markers.markers.size());
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
        // 简单的量化哈希，允许微小的颜色误差
        return static_cast<int>(color.r * 100) << 16 |
               static_cast<int>(color.g * 100) << 8 |
               static_cast<int>(color.b * 100);
    }

    std::string getObjectTypeFromColor(const std_msgs::msg::ColorRGBA& color)
    {
        // 红色: evacuation_box
        if (color.r > 0.8 && color.g < 0.2 && color.b < 0.2) return "evacuation_box";
        
        // 绿色: fire_extinguisher
        if (color.g > 0.8 && color.r < 0.2 && color.b < 0.2) return "fire_extinguisher";
        
        // 深蓝色: fire_hydrant (r=0, g=0, b=0.6~0.8)
        if (color.b > 0.5 && color.r < 0.2 && color.g < 0.2) return "fire_hydrant";
        
        // 黄色: rubbish_bin (r=1, g=1, b=0)
        if (color.r > 0.8 && color.g > 0.8 && color.b < 0.2) return "rubbish_bin";
        
        // 紫粉色: door (r=0.8, g=0, b=0.8) 或 (r=1, g=0, b=1)
        if (color.r > 0.6 && color.b > 0.6 && color.g < 0.3) return "door";

        // 青色/浅蓝色 (r=0, g=1, b=1) - 以前可能用到，这里也保留一下识别能力
        if (color.g > 0.8 && color.b > 0.8 && color.r < 0.2) return "unknown_cyan";

        return "unknown";
    }

    std_msgs::msg::ColorRGBA getRandomColor(int seed)
    {
        std::mt19937 gen(seed);
        std::uniform_real_distribution<float> dis(0.0, 1.0);
        
        std_msgs::msg::ColorRGBA color;
        // 生成明亮且鲜艳的颜色，避免太暗或太接近原色
        color.r = dis(gen);
        color.g = dis(gen);
        color.b = dis(gen);
        
        // 简单的HSV转换逻辑或直接随机生成，这里直接用随机
        // 为了区分度，可以避开某些颜色，但完全随机通常足够
        
        // 确保不透明
        color.a = 1.0;
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
