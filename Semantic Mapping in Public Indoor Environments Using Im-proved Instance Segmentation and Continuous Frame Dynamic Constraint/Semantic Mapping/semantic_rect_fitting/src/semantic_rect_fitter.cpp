#include "semantic_rect_fitting/semantic_rect_fitting.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace semantic_rect_fitting
{

SemanticRectFitter::SemanticRectFitter() : Node("semantic_rect_fitter")
{
    // Declare parameters
    this->declare_parameter("min_points_threshold", 3);
    this->declare_parameter("min_area_threshold", 0.01);  // m²
    this->declare_parameter("max_area_threshold", 10.0);  // m²
    this->declare_parameter("use_pca_fitting", true);
    this->declare_parameter("publish_debug", true);
    this->declare_parameter("rectangle_height", 0.5);  // m

    // Get parameters
    min_points_threshold_ = this->get_parameter("min_points_threshold").as_double();
    min_area_threshold_ = this->get_parameter("min_area_threshold").as_double();
    max_area_threshold_ = this->get_parameter("max_area_threshold").as_double();
    use_pca_fitting_ = this->get_parameter("use_pca_fitting").as_bool();
    publish_debug_ = this->get_parameter("publish_debug").as_bool();
    rectangle_height_ = this->get_parameter("rectangle_height").as_double();

    // Initialize color to object type mapping
    color_to_object_map_[{1.0f, 0.0f, 0.0f}] = "fire_extinguisher";  // Red
    color_to_object_map_[{0.0f, 1.0f, 0.0f}] = "evacuation_box";     // Green
    color_to_object_map_[{0.0f, 0.0f, 1.0f}] = "fire_hydrant";       // Blue
    color_to_object_map_[{1.0f, 1.0f, 0.0f}] = "rubbish_bin";        // Yellow
    color_to_object_map_[{1.0f, 0.0f, 1.0f}] = "door";               // Purple

    // Create subscribers and publishers
    marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/loaded_semantic_markers", 
        rclcpp::QoS(10).transient_local(),
        std::bind(&SemanticRectFitter::markerArrayCallback, this, std::placeholders::_1)
    );

    rectangle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/semantic_rectangles", 
        rclcpp::QoS(10).transient_local()
    );

    if (publish_debug_)
    {
        debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/semantic_rectangles_debug", 
            rclcpp::QoS(10).transient_local()
        );
    }

    RCLCPP_INFO(this->get_logger(), "Semantic Rectangle Fitter initialized");
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  - min_points_threshold: %.1f", min_points_threshold_);
    RCLCPP_INFO(this->get_logger(), "  - min_area_threshold: %.3f m²", min_area_threshold_);
    RCLCPP_INFO(this->get_logger(), "  - max_area_threshold: %.1f m²", max_area_threshold_);
    RCLCPP_INFO(this->get_logger(), "  - use_pca_fitting: %s", use_pca_fitting_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  - rectangle_height: %.2f m", rectangle_height_);
}

void SemanticRectFitter::markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received %zu markers for rectangle fitting", msg->markers.size());

    // Group markers by color
    std::map<ColorKey, std::vector<geometry_msgs::msg::Point>> color_groups;
    std::map<ColorKey, std_msgs::msg::ColorRGBA> color_map;

    for (const auto& marker : msg->markers)
    {
        ColorKey key{marker.color.r, marker.color.g, marker.color.b};
        color_groups[key].push_back(marker.pose.position);
        color_map[key] = marker.color;
    }

    // Fit rectangles for each color group
    std::vector<Rectangle> rectangles;
    
    for (const auto& [color_key, points] : color_groups)
    {
        if (shouldFilterSmallObjects(points))
        {
            RCLCPP_DEBUG(this->get_logger(), "Skipping small object with %zu points", points.size());
            continue;
        }

        std_msgs::msg::ColorRGBA color = color_map[color_key];
        
        Rectangle rect;
        if (use_pca_fitting_)
        {
            rect = fitRectangleToPCA(points, color);
        }
        else
        {
            rect = fitRectangleToMinBounding(points, color);
        }
        
        // Validate rectangle area
        double area = rect.width * rect.height;
        if (area >= min_area_threshold_ && area <= max_area_threshold_)
        {
            rect.object_type = getObjectTypeFromColor(color);
            rect.point_count = points.size();
            rectangles.push_back(rect);
            
            RCLCPP_DEBUG(this->get_logger(), "Created rectangle for %s: %.2fm x %.2fm (area: %.3fm²)", 
                        rect.object_type.c_str(), rect.width, rect.height, area);
        }
    }

    // Publish rectangles
    if (!rectangles.empty())
    {
        auto rectangle_markers = createRectangleMarkers(rectangles);
        rectangle_pub_->publish(rectangle_markers);
        
        if (publish_debug_)
        {
            publishDebugInfo(rectangles);
        }
        
        RCLCPP_INFO(this->get_logger(), "Published %zu semantic rectangles", rectangles.size());
    }
}

Rectangle SemanticRectFitter::fitRectangleToPCA(const std::vector<geometry_msgs::msg::Point>& points, 
                                               const std_msgs::msg::ColorRGBA& color)
{
    Rectangle rect;
    rect.color = color;

    if (points.size() < 3)
    {
        // Fallback for very few points
        return fitRectangleToMinBounding(points, color);
    }

    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : points)
    {
        cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Perform PCA
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    
    // Get eigenvectors and eigenvalues
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    Eigen::Vector4f centroid = pca.getMean();

    // Project points onto principal components
    std::vector<float> proj_x, proj_y;
    for (const auto& point : points)
    {
        Eigen::Vector3f p(point.x - centroid[0], point.y - centroid[1], point.z - centroid[2]);
        proj_x.push_back(p.dot(eigenvectors.col(0)));
        proj_y.push_back(p.dot(eigenvectors.col(1)));
    }

    // Calculate bounding box in PCA space
    float min_x = *std::min_element(proj_x.begin(), proj_x.end());
    float max_x = *std::max_element(proj_x.begin(), proj_x.end());
    float min_y = *std::min_element(proj_y.begin(), proj_y.end());
    float max_y = *std::max_element(proj_y.begin(), proj_y.end());

    rect.width = max_x - min_x;
    rect.height = max_y - min_y;
    
    // Ensure minimum size
    rect.width = std::max(rect.width, 0.1f);
    rect.height = std::max(rect.height, 0.1f);

    // Calculate center
    rect.center.x = centroid[0];
    rect.center.y = centroid[1];
    rect.center.z = centroid[2];

    // Calculate rotation angle (angle of first principal component)
    rect.angle = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));

    // Calculate corner points
    rect.corner_points = calculateRectangleCorners(rect);

    return rect;
}

Rectangle SemanticRectFitter::fitRectangleToMinBounding(const std::vector<geometry_msgs::msg::Point>& points, 
                                                       const std_msgs::msg::ColorRGBA& color)
{
    Rectangle rect;
    rect.color = color;

    if (points.empty())
    {
        return rect;
    }

    // Find min/max coordinates
    auto min_x_it = std::min_element(points.begin(), points.end(),
        [](const auto& a, const auto& b) { return a.x < b.x; });
    auto max_x_it = std::max_element(points.begin(), points.end(),
        [](const auto& a, const auto& b) { return a.x < b.x; });
    auto min_y_it = std::min_element(points.begin(), points.end(),
        [](const auto& a, const auto& b) { return a.y < b.y; });
    auto max_y_it = std::max_element(points.begin(), points.end(),
        [](const auto& a, const auto& b) { return a.y < b.y; });

    rect.width = max_x_it->x - min_x_it->x;
    rect.height = max_y_it->y - min_y_it->y;
    
    // Ensure minimum size
    rect.width = std::max(rect.width, 0.1f);
    rect.height = std::max(rect.height, 0.1f);

    // Calculate center
    rect.center.x = (min_x_it->x + max_x_it->x) / 2.0;
    rect.center.y = (min_y_it->y + max_y_it->y) / 2.0;
    
    // Average Z coordinate
    double sum_z = 0.0;
    for (const auto& point : points)
    {
        sum_z += point.z;
    }
    rect.center.z = sum_z / points.size();

    rect.angle = 0.0;  // Axis-aligned rectangle

    // Calculate corner points
    rect.corner_points = calculateRectangleCorners(rect);

    return rect;
}

std::vector<geometry_msgs::msg::Point> SemanticRectFitter::calculateRectangleCorners(const Rectangle& rect)
{
    std::vector<geometry_msgs::msg::Point> corners(4);

    float cos_angle = std::cos(rect.angle);
    float sin_angle = std::sin(rect.angle);
    float half_width = rect.width / 2.0;
    float half_height = rect.height / 2.0;

    // Calculate the four corners
    std::vector<std::pair<float, float>> local_corners = {
        {-half_width, -half_height},  // Bottom-left
        {half_width, -half_height},   // Bottom-right
        {half_width, half_height},    // Top-right
        {-half_width, half_height}    // Top-left
    };

    for (size_t i = 0; i < 4; ++i)
    {
        float local_x = local_corners[i].first;
        float local_y = local_corners[i].second;

        // Rotate and translate
        corners[i].x = rect.center.x + (local_x * cos_angle - local_y * sin_angle);
        corners[i].y = rect.center.y + (local_x * sin_angle + local_y * cos_angle);
        corners[i].z = rect.center.z;
    }

    return corners;
}

visualization_msgs::msg::MarkerArray SemanticRectFitter::createRectangleMarkers(const std::vector<Rectangle>& rectangles)
{
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < rectangles.size(); ++i)
    {
        const auto& rect = rectangles[i];

        // Create rectangle outline marker
        visualization_msgs::msg::Marker outline_marker;
        outline_marker.header.frame_id = "map";
        outline_marker.header.stamp = this->get_clock()->now();
        outline_marker.ns = "semantic_rectangles";
        outline_marker.id = i * 2;  // Even IDs for outlines
        outline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        outline_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set scale (line width)
        outline_marker.scale.x = 0.05;

        // Set color (slightly darker than original)
        outline_marker.color = rect.color;
        outline_marker.color.a = 1.0;

        // Add corner points to create closed rectangle
        for (const auto& corner : rect.corner_points)
        {
            outline_marker.points.push_back(corner);
        }
        // Close the rectangle
        outline_marker.points.push_back(rect.corner_points[0]);

        marker_array.markers.push_back(outline_marker);

        // Create filled rectangle marker
        visualization_msgs::msg::Marker filled_marker;
        filled_marker.header.frame_id = "map";
        filled_marker.header.stamp = this->get_clock()->now();
        filled_marker.ns = "semantic_rectangles_filled";
        filled_marker.id = i * 2 + 1;  // Odd IDs for filled rectangles
        filled_marker.type = visualization_msgs::msg::Marker::CUBE;
        filled_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set position
        filled_marker.pose.position = rect.center;
        filled_marker.pose.position.z += rectangle_height_ / 2.0;  // Lift above ground

        // Set orientation
        filled_marker.pose.orientation.x = 0.0;
        filled_marker.pose.orientation.y = 0.0;
        filled_marker.pose.orientation.z = std::sin(rect.angle / 2.0);
        filled_marker.pose.orientation.w = std::cos(rect.angle / 2.0);

        // Set scale
        filled_marker.scale.x = rect.width;
        filled_marker.scale.y = rect.height;
        filled_marker.scale.z = rectangle_height_;

        // Set color (opaque)
        filled_marker.color = rect.color;
        filled_marker.color.a = 1.0;

        marker_array.markers.push_back(filled_marker);

        // Create text label marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.ns = "semantic_rectangles_text";
        text_marker.id = i;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        // Position text above the rectangle
        text_marker.pose.position = rect.center;
        text_marker.pose.position.z += rectangle_height_ + 0.2;

        // Set scale (text size)
        text_marker.scale.z = 0.3;

        // Set color (white with full opacity)
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        // Set text content
        std::stringstream ss;
        ss << rect.object_type << "\n(" << rect.point_count << " pts)\n"
           << std::fixed << std::setprecision(2) 
           << rect.width << "m × " << rect.height << "m";
        text_marker.text = ss.str();

        marker_array.markers.push_back(text_marker);
    }

    return marker_array;
}

std::string SemanticRectFitter::getObjectTypeFromColor(const std_msgs::msg::ColorRGBA& color)
{
    ColorKey key{color.r, color.g, color.b};
    
    auto it = color_to_object_map_.find(key);
    if (it != color_to_object_map_.end())
    {
        return it->second;
    }
    
    // Default fallback
    return "unknown_object";
}

bool SemanticRectFitter::shouldFilterSmallObjects(const std::vector<geometry_msgs::msg::Point>& points)
{
    return points.size() < min_points_threshold_;
}

void SemanticRectFitter::publishDebugInfo(const std::vector<Rectangle>& rectangles)
{
    visualization_msgs::msg::MarkerArray debug_markers;

    for (size_t i = 0; i < rectangles.size(); ++i)
    {
        const auto& rect = rectangles[i];

        // Create center point marker
        visualization_msgs::msg::Marker center_marker;
        center_marker.header.frame_id = "map";
        center_marker.header.stamp = this->get_clock()->now();
        center_marker.ns = "rectangle_centers";
        center_marker.id = i;
        center_marker.type = visualization_msgs::msg::Marker::SPHERE;
        center_marker.action = visualization_msgs::msg::Marker::ADD;

        center_marker.pose.position = rect.center;
        center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.1;
        center_marker.color.r = 1.0;
        center_marker.color.g = 1.0;
        center_marker.color.b = 0.0;
        center_marker.color.a = 1.0;

        debug_markers.markers.push_back(center_marker);

        // Create corner markers
        for (size_t j = 0; j < rect.corner_points.size(); ++j)
        {
            visualization_msgs::msg::Marker corner_marker;
            corner_marker.header.frame_id = "map";
            corner_marker.header.stamp = this->get_clock()->now();
            corner_marker.ns = "rectangle_corners";
            corner_marker.id = i * 10 + j;
            corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
            corner_marker.action = visualization_msgs::msg::Marker::ADD;

            corner_marker.pose.position = rect.corner_points[j];
            corner_marker.scale.x = corner_marker.scale.y = corner_marker.scale.z = 0.05;
            corner_marker.color.r = 0.0;
            corner_marker.color.g = 1.0;
            corner_marker.color.b = 0.0;
            corner_marker.color.a = 1.0;

            debug_markers.markers.push_back(corner_marker);
        }
    }

    if (debug_pub_)
    {
        debug_pub_->publish(debug_markers);
    }
}

} // namespace semantic_rect_fitting

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<semantic_rect_fitting::SemanticRectFitter>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Semantic Rectangle Fitter node...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
