#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

class SemanticMapLoader : public rclcpp::Node
{
public:
    SemanticMapLoader() : Node("semantic_map_loader")
    {
        // 创建发布者，使用可靠性和持久性QoS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/loaded_map", qos);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/loaded_semantic_markers", qos);

        // 声明并获取参数
        this->declare_parameter("map_dir", "");
        std::string map_dir = this->get_parameter("map_dir").as_string();

        if (map_dir.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please specify the map directory using map_dir parameter");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_dir.c_str());

        try {
            // 加载并发布地图
            loadAndPublishMap(map_dir + "/map.yaml");
            
            // 加载并发布语义标记
            loadAndPublishMarkers(map_dir + "/semantic_markers.yaml");

            RCLCPP_INFO(this->get_logger(), "Map and markers loaded successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading map: %s", e.what());
        }

        // 创建定时器，定期重新发布数据
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SemanticMapLoader::timerCallback, this));
    }

private:
    void loadAndPublishMap(const std::string& yaml_file)
    {
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        // 读取地图元数据
        std::string image_file = config["image"].as<std::string>();
        // 处理相对路径：如果image_file不是绝对路径，则将其拼接到yaml_file所在目录
        std::string pgm_filename;
        if (image_file.front() == '/') {
            pgm_filename = image_file;
        } else {
            // 获取yaml文件所在的目录路径
            size_t last_slash_idx = yaml_file.rfind('/');
            if (std::string::npos != last_slash_idx) {
                pgm_filename = yaml_file.substr(0, last_slash_idx + 1) + image_file;
            } else {
                pgm_filename = image_file;
            }
        }
        
        double resolution = config["resolution"].as<double>();
        std::vector<double> origin = config["origin"].as<std::vector<double>>();

        // 读取PGM文件
        std::ifstream pgm(pgm_filename, std::ios::binary);
        if (!pgm.is_open()) {
            throw std::runtime_error("Failed to open PGM file: " + pgm_filename);
        }

        // 读取PGM头
        std::string line;
        std::getline(pgm, line); // P5 or P2
        // 处理可能存在的回车符 (Windows格式)
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (line != "P5" && line != "P2") {
             throw std::runtime_error("Invalid PGM file format (must be P5 or P2), found: " + line);
        }
        
        // 跳过注释行
        while (pgm.peek() == '#' || pgm.peek() == '\n' || pgm.peek() == '\r') {
            if (pgm.peek() == '#' || pgm.peek() == '\n' || pgm.peek() == '\r') {
                std::getline(pgm, line);
            }
        }

        int width, height, maxval;
        pgm >> width >> height >> maxval;

        std::vector<unsigned char> pgm_data(width * height);
        
        if (line == "P5") {
            pgm.get(); // 读取换行符或空格
            pgm.read(reinterpret_cast<char*>(pgm_data.data()), width * height);
        } else if (line == "P2") {
            for (int i = 0; i < width * height; ++i) {
                int pixel_val;
                pgm >> pixel_val;
                pgm_data[i] = static_cast<unsigned char>(pixel_val);
            }
        }
        
        // 创建地图消息
        map_msg_.info.resolution = resolution;
        map_msg_.info.width = width;
        map_msg_.info.height = height;
        map_msg_.info.origin.position.x = origin[0];
        map_msg_.info.origin.position.y = origin[1];
        map_msg_.info.origin.position.z = origin[2];
        map_msg_.info.origin.orientation.w = 1.0;
        
        map_msg_.data.resize(width * height);
        
        // PGM通常存储为：第一行是图像的最顶行
        // ROS OccupancyGrid data[0] 是地图的(0,0)，即左下角
        // 图像数据是从上到下存储的，需要翻转Y轴
        for (int y = height - 1; y >= 0; y--) {
            for (int x = 0; x < width; x++) {
                int pgm_index = (height - 1 - y) * width + x;
                int map_index = y * width + x;
                
                unsigned char pixel = pgm_data[pgm_index];
                
                // Standard ROS map interpretation:
                // 0 = Occupied (Black)
                // 255 = Free (White)
                // 205 = Unknown (Gray)
                // However, map_server uses thresholds.
                // occ = (255 - pixel) / 255.0
                // if occ > occupied_thresh (0.65) -> Occupied (100)
                //    (255 - pixel) > 165.75 => pixel < 89.25
                // if occ < free_thresh (0.196) -> Free (0)
                //    (255 - pixel) < 49.98 => pixel > 205.02
                
                // Let's implement robust thresholding
                if (pixel >= 250) {
                     map_msg_.data[map_index] = 0; // Free (White)
                } else if (pixel <= 10) {
                     map_msg_.data[map_index] = 100; // Occupied (Black)
                } else if (pixel == 205) {
                     map_msg_.data[map_index] = -1; // Unknown
                } else {
                     // Fallback for intermediate values
                     double occ = (255.0 - pixel) / 255.0;
                     if (occ > 0.65) map_msg_.data[map_index] = 100;
                     else if (occ < 0.196) map_msg_.data[map_index] = 0;
                     else map_msg_.data[map_index] = -1;
                }
            }
        }

        map_msg_.header.frame_id = "map";
        publishMap();
    }

    void loadAndPublishMarkers(const std::string& yaml_file)
    {
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        marker_array_.markers.clear();
        
        std::string frame_id = config["frame_id"].as<std::string>();
        const YAML::Node& markers = config["markers"];

        for (const auto& marker_node : markers) {
            try {
                visualization_msgs::msg::Marker marker;
                
                marker.header.frame_id = frame_id;
                marker.id = marker_node["id"].as<int>();
                marker.ns = marker_node["ns"].as<std::string>();
                marker.type = marker_node["type"].as<int>();
                
                auto get_double = [](const YAML::Node& node) -> double {
                    if (node && node.IsScalar()) {
                        try { return node.as<double>(); } catch (...) {}
                        try { return static_cast<double>(node.as<int>()); } catch (...) {}
                    }
                    return 0.0;
                };

                if (marker_node["pose"] && marker_node["pose"]["position"]) {
                    marker.pose.position.x = get_double(marker_node["pose"]["position"]["x"]);
                    marker.pose.position.y = get_double(marker_node["pose"]["position"]["y"]);
                    marker.pose.position.z = get_double(marker_node["pose"]["position"]["z"]);
                }
                
                if (marker_node["pose"] && marker_node["pose"]["orientation"]) {
                    marker.pose.orientation.x = get_double(marker_node["pose"]["orientation"]["x"]);
                    marker.pose.orientation.y = get_double(marker_node["pose"]["orientation"]["y"]);
                    marker.pose.orientation.z = get_double(marker_node["pose"]["orientation"]["z"]);
                    marker.pose.orientation.w = get_double(marker_node["pose"]["orientation"]["w"]);
                } else {
                    marker.pose.orientation.w = 1.0;
                }
                
                if (marker_node["scale"]) {
                    marker.scale.x = get_double(marker_node["scale"]["x"]);
                    marker.scale.y = get_double(marker_node["scale"]["y"]);
                    marker.scale.z = get_double(marker_node["scale"]["z"]);
                } else {
                    marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;
                }
                
                if (marker_node["color"] && marker_node["color"].IsMap()) {
                    marker.color.r = get_double(marker_node["color"]["r"]);
                    marker.color.g = get_double(marker_node["color"]["g"]);
                    marker.color.b = get_double(marker_node["color"]["b"]);
                    marker.color.a = get_double(marker_node["color"]["a"]);
                } else {
                    marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;
                }

                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.lifetime = rclcpp::Duration::from_seconds(0);
                
                marker_array_.markers.push_back(marker);
            } catch (const YAML::Exception& e) {
                RCLCPP_WARN(this->get_logger(), "Skipping a malformed marker node: %s", e.what());
                continue;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Skipping a marker due to error: %s", e.what());
                continue;
            }
        }

        publishMarkers();
    }

    void publishMap()
    {
        map_msg_.header.stamp = this->now();
        map_pub_->publish(map_msg_);
    }

    void publishMarkers()
    {
        for (auto& marker : marker_array_.markers) {
            marker.header.stamp = this->now();
        }
        marker_pub_->publish(marker_array_);
    }

    void timerCallback()
    {
        static int count = 0;
        if (count++ % 5 == 0) {  // 降低地图发布频率，每5秒发一次，避免带宽占用过大
            publishMap();
        }
        publishMarkers();
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::OccupancyGrid map_msg_;
    visualization_msgs::msg::MarkerArray marker_array_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticMapLoader>());
    rclcpp::shutdown();
    return 0;
}
