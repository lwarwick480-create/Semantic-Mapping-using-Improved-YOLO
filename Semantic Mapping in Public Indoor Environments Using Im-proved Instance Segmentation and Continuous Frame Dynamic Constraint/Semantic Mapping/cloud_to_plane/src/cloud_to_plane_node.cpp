#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <unordered_map>
#include <cmath>

// 用于网格索引的结构体
struct GridIndex {
    int x;
    int z;
    
    bool operator==(const GridIndex& other) const {
        return x == other.x && z == other.z;
    }
};

// 网格索引的哈希函数
struct GridIndexHash {
    std::size_t operator()(const GridIndex& index) const {
        return std::hash<int>()(index.x) ^ (std::hash<int>()(index.z) << 1);
    }
};

class CloudToPlaneNode : public rclcpp::Node
{
public:
    CloudToPlaneNode() : Node("cloud_to_plane_node")
    {
        // 声明并获取参数
        this->declare_parameter("target_y", 0.0);
        this->declare_parameter("grid_size", 0.05);  // 网格大小，单位：米 (从0.1减小到0.05)
        
        target_y_ = this->get_parameter("target_y").as_double();
        grid_size_ = this->get_parameter("grid_size").as_double();

        RCLCPP_INFO(this->get_logger(), "Target Y coordinate: %.2f", target_y_);
        RCLCPP_INFO(this->get_logger(), "Grid size: %.2f", grid_size_);

        // 创建订阅者和发布者
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/intercept_points", 10,
            std::bind(&CloudToPlaneNode::pointCloudCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/plane_points", 10);
    }

private:
    GridIndex getGridIndex(float x, float z) {
        return {
            static_cast<int>(std::floor(x / grid_size_)),
            static_cast<int>(std::floor(z / grid_size_))
        };
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 使用哈希表存储每个网格中的点
        std::unordered_map<GridIndex, std::vector<uint8_t>, GridIndexHash> grid_points;

        // 遍历输入点云
        for (size_t i = 0; i < msg->width * msg->height; ++i)
        {
            size_t point_offset = i * msg->point_step;
            
            // 读取x和z坐标
            float x, z;
            memcpy(&x, &msg->data[point_offset], sizeof(float));
            memcpy(&z, &msg->data[point_offset + 2 * sizeof(float)], sizeof(float));
            
            // 获取网格索引
            GridIndex index = getGridIndex(x, z);
            
            // 如果这个网格还没有点，添加这个点
            if (grid_points.find(index) == grid_points.end()) {
                std::vector<uint8_t> point_data(msg->point_step);
                
                // 复制x坐标
                memcpy(&point_data[0], &x, sizeof(float));
                
                // 设置y坐标为目标值
                memcpy(&point_data[sizeof(float)], &target_y_, sizeof(float));
                
                // 复制z坐标和其他数据
                memcpy(&point_data[2 * sizeof(float)], 
                       &msg->data[point_offset + 2 * sizeof(float)],
                       msg->point_step - 2 * sizeof(float));
                
                grid_points[index] = point_data;
            }
        }

        // 创建输出点云消息
        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;
        output.fields = msg->fields;
        output.point_step = msg->point_step;
        output.height = 1;
        output.width = grid_points.size();  // 降采样后的点数
        output.is_bigendian = msg->is_bigendian;
        output.is_dense = msg->is_dense;
        output.row_step = output.width * output.point_step;
        
        // 分配空间
        output.data.resize(output.width * output.point_step);
        
        // 填充数据
        size_t current_point = 0;
        for (const auto& grid_point : grid_points) {
            memcpy(&output.data[current_point * output.point_step],
                   grid_point.second.data(),
                   output.point_step);
            current_point++;
        }

        // 发布降采样后的点云
        publisher_->publish(output);
        
        // 输出降采样信息
        RCLCPP_INFO(this->get_logger(), 
                    "Downsampled points from %zu to %zu", 
                    msg->width * msg->height,
                    output.width);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    double target_y_;
    double grid_size_;  // 网格大小
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudToPlaneNode>());
    rclcpp::shutdown();
    return 0;
}
