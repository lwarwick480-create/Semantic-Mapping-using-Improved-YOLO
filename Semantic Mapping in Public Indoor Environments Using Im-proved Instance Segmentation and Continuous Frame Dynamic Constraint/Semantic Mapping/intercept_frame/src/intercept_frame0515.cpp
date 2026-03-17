#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class PointCloudCropper : public rclcpp::Node
{
public:
    PointCloudCropper() : Node("intercept_frame_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points", 10,
            std::bind(&PointCloudCropper::pointcloud_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/intercept_points", 10);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const int crop_width = 200;
        const int crop_height = 200;
        const int center_x = msg->width / 2;
        const int center_y = msg->height / 2;
        const int start_x = center_x - crop_width / 2;
        const int start_y = center_y - crop_height / 2;

        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;
        output.height = crop_height;
        output.width = crop_width;
        output.fields = msg->fields;
        output.is_bigendian = msg->is_bigendian;
        output.point_step = msg->point_step;
        output.row_step = crop_width * msg->point_step;
        output.is_dense = msg->is_dense;
        output.data.resize(crop_height * output.row_step);

        for (int y = 0; y < crop_height; ++y)
        {
            int input_row = (start_y + y) * msg->row_step;
            int output_row = y * output.row_step;

            for (int x = 0; x < crop_width; ++x)
            {
                int input_index = input_row + (start_x + x) * msg->point_step;
                int output_index = output_row + x * msg->point_step;

                std::copy_n(
                    msg->data.begin() + input_index,
                    msg->point_step,
                    output.data.begin() + output_index);
            }
        }

        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudCropper>());
    rclcpp::shutdown();
    return 0;
}

