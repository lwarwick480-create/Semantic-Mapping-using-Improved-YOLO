#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cstdlib>  // 为了使用 getenv
#include <map>

using std::placeholders::_1;
using std::placeholders::_2;

class SemanticMapSaver : public rclcpp::Node
{
public:
    SemanticMapSaver() : Node("semantic_map_saver")
    {
        // 获取用户家目录
        const char* home = std::getenv("HOME");
        if (home != nullptr) {
            home_dir_ = std::string(home);
            RCLCPP_INFO(this->get_logger(), "Home directory: %s", home_dir_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get home directory");
            home_dir_ = ".";
        }

        // 订阅栅格地图
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&SemanticMapSaver::mapCallback, this, _1));

        // 订阅聚类后的语义标记
        // 增加 Depth 到 1000，以便在启动晚时能接收历史累积的消息
        auto marker_qos = rclcpp::QoS(1000).transient_local();
        marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/semantic_markers", marker_qos, std::bind(&SemanticMapSaver::markerCallback, this, _1));

        // 创建保存服务
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_semantic_map",
            std::bind(&SemanticMapSaver::saveMapCallback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Semantic map saver node initialized");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        have_map_ = true;
    }

    void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        // 累积保存 marker
        for (const auto& marker : msg->markers) {
            if (marker.action == visualization_msgs::msg::Marker::DELETE) {
                accumulated_markers_.erase(marker.id);
            } else {
                accumulated_markers_[marker.id] = marker;
            }
        }
        
        if (!accumulated_markers_.empty()) {
            have_markers_ = true;
            // 降低日志频率，或者只在数量变化大时打印
            // RCLCPP_INFO(this->get_logger(), "Updated markers. Total accumulated markers: %zu", accumulated_markers_.size());
        }
    }

    void saveMapCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!have_map_ || !have_markers_) {
            response->success = false;
            response->message = "No map or markers data available";
            return;
        }

        try {
            // 创建保存目录
            std::string save_time = getCurrentTimeString();
            std::string save_dir = home_dir_ + "/Desktop/dev_ws/src/semantic_map_saver/semantic_maps/map_" + save_time;
            
            // 创建目录并输出日志
            RCLCPP_INFO(this->get_logger(), "Creating directory: %s", save_dir.c_str());
            std::system(("mkdir -p " + save_dir).c_str());

            // 保存栅格地图
            saveOccupancyGrid(save_dir + "/map.pgm", save_dir + "/map.yaml");
            RCLCPP_INFO(this->get_logger(), "Saved occupancy grid map");

            // 保存语义标记
            saveSemanticMarkers(save_dir + "/semantic_markers.yaml");
            RCLCPP_INFO(this->get_logger(), "Saved semantic markers");

            response->success = true;
            response->message = "Map saved to " + save_dir;
        }
        catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to save map: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Error saving map: %s", e.what());
        }
    }

    void saveOccupancyGrid(const std::string& pgm_file, const std::string& yaml_file)
    {
        // 保存PGM文件
        std::ofstream pgm(pgm_file, std::ios::binary);
        if (!pgm.is_open()) {
            throw std::runtime_error("Failed to open PGM file: " + pgm_file);
        }

        // PGM头
        pgm << "P5\n";
        pgm << current_map_.info.width << " " << current_map_.info.height << "\n";
        pgm << "255\n";

        // 写入地图数据
        for (unsigned int y = 0; y < current_map_.info.height; y++) {
            for (unsigned int x = 0; x < current_map_.info.width; x++) {
                unsigned int i = x + (current_map_.info.height - y - 1) * current_map_.info.width;
                if (current_map_.data[i] == -1) { // 未知
                    pgm.put(205);
                } else if (current_map_.data[i] == 0) { // 空闲
                    pgm.put(254);
                } else { // 占据
                    pgm.put(0);
                }
            }
        }
        pgm.close();

        // 保存YAML文件
        YAML::Node yaml;
        yaml["image"] = pgm_file;
        yaml["resolution"] = current_map_.info.resolution;
        yaml["origin"] = std::vector<double>{
            current_map_.info.origin.position.x,
            current_map_.info.origin.position.y,
            0.0
        };
        yaml["negate"] = 0;
        yaml["occupied_thresh"] = 0.65;
        yaml["free_thresh"] = 0.196;

        std::ofstream yaml_out(yaml_file);
        if (!yaml_out.is_open()) {
            throw std::runtime_error("Failed to open YAML file: " + yaml_file);
        }
        yaml_out << yaml;
        yaml_out.close();
    }

    void saveSemanticMarkers(const std::string& yaml_file)
    {
        if (accumulated_markers_.empty()) {
            throw std::runtime_error("No markers to save");
        }

        YAML::Node yaml;
        // 使用第一个marker的frame_id
        yaml["frame_id"] = accumulated_markers_.begin()->second.header.frame_id;
        yaml["markers"] = YAML::Node(YAML::NodeType::Sequence);

        for (const auto& pair : accumulated_markers_) {
            const auto& marker = pair.second;
            YAML::Node marker_node;
            marker_node["id"] = marker.id;
            marker_node["ns"] = marker.ns;
            marker_node["type"] = marker.type;
            
            marker_node["pose"]["position"]["x"] = marker.pose.position.x;
            marker_node["pose"]["position"]["y"] = marker.pose.position.y;
            marker_node["pose"]["position"]["z"] = marker.pose.position.z;
            
            marker_node["pose"]["orientation"]["x"] = marker.pose.orientation.x;
            marker_node["pose"]["orientation"]["y"] = marker.pose.orientation.y;
            marker_node["pose"]["orientation"]["z"] = marker.pose.orientation.z;
            marker_node["pose"]["orientation"]["w"] = marker.pose.orientation.w;
            
            marker_node["scale"]["x"] = marker.scale.x;
            marker_node["scale"]["y"] = marker.scale.y;
            marker_node["scale"]["z"] = marker.scale.z;
            
            marker_node["color"]["r"] = marker.color.r;
            marker_node["color"]["g"] = marker.color.g;
            marker_node["color"]["b"] = marker.color.b;
            marker_node["color"]["a"] = marker.color.a;

            yaml["markers"].push_back(marker_node);
        }

        std::ofstream yaml_out(yaml_file);
        if (!yaml_out.is_open()) {
            throw std::runtime_error("Failed to open semantic markers YAML file: " + yaml_file);
        }
        yaml_out << yaml;
        yaml_out.close();
    }

    std::string getCurrentTimeString()
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;

    nav_msgs::msg::OccupancyGrid current_map_;
    // 使用 map 来存储累积的 marker，key 是 marker.id
    std::map<int, visualization_msgs::msg::Marker> accumulated_markers_;
    
    bool have_map_ = false;
    bool have_markers_ = false;
    std::string home_dir_;  // 存储用户家目录
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticMapSaver>());
    rclcpp::shutdown();
    return 0;
}
