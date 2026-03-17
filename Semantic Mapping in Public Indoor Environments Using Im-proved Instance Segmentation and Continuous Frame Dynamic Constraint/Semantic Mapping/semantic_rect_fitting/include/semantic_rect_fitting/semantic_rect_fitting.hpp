#ifndef SEMANTIC_RECT_FITTING_HPP_
#define SEMANTIC_RECT_FITTING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <vector>
#include <map>
#include <string>

namespace semantic_rect_fitting
{

struct ColorKey
{
    float r, g, b;
    
    bool operator<(const ColorKey& other) const
    {
        if (r != other.r) return r < other.r;
        if (g != other.g) return g < other.g;
        return b < other.b;
    }
    
    bool operator==(const ColorKey& other) const
    {
        return (std::abs(r - other.r) < 0.01f) && 
               (std::abs(g - other.g) < 0.01f) && 
               (std::abs(b - other.b) < 0.01f);
    }
};

struct Rectangle
{
    geometry_msgs::msg::Point center;
    double width;
    double height;
    double angle;  // rotation angle in radians
    std_msgs::msg::ColorRGBA color;
    std::vector<geometry_msgs::msg::Point> corner_points;
    int point_count;
    std::string object_type;
};

class SemanticRectFitter : public rclcpp::Node
{
public:
    SemanticRectFitter();
    ~SemanticRectFitter() = default;

private:
    void markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    Rectangle fitRectangleToPCA(const std::vector<geometry_msgs::msg::Point>& points, 
                               const std_msgs::msg::ColorRGBA& color);
    
    Rectangle fitRectangleToMinBounding(const std::vector<geometry_msgs::msg::Point>& points, 
                                       const std_msgs::msg::ColorRGBA& color);
    
    std::vector<geometry_msgs::msg::Point> calculateRectangleCorners(const Rectangle& rect);
    
    visualization_msgs::msg::MarkerArray createRectangleMarkers(const std::vector<Rectangle>& rectangles);
    
    std::string getObjectTypeFromColor(const std_msgs::msg::ColorRGBA& color);
    
    bool shouldFilterSmallObjects(const std::vector<geometry_msgs::msg::Point>& points);
    
    void publishDebugInfo(const std::vector<Rectangle>& rectangles);

    // ROS2 components
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rectangle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
    
    // Parameters
    double min_points_threshold_;
    double min_area_threshold_;
    double max_area_threshold_;
    bool use_pca_fitting_;
    bool publish_debug_;
    double rectangle_height_;  // Z height for 3D visualization
    
    // Object type mapping
    std::map<ColorKey, std::string> color_to_object_map_;
};

} // namespace semantic_rect_fitting

#endif // SEMANTIC_RECT_FITTING_HPP_
