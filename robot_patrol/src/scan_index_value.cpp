#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>  // For std::numeric_limits

class ScanDataPrinter : public rclcpp::Node
{
public:
    ScanDataPrinter() : Node("scan_data_printer")
    {
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanDataPrinter::laser_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        size_t start_idx = 0;
        size_t quarter_idx = msg->ranges.size() / 4;
        size_t mid_idx = msg->ranges.size() / 2;
        size_t three_quarters_idx = 3 * (msg->ranges.size() / 4);

        float min_distance = std::numeric_limits<float>::max();
        size_t min_distance_idx = 0;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] < min_distance && std::isfinite(msg->ranges[i]))
            {
                min_distance = msg->ranges[i];
                min_distance_idx = i;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Scan data at specific indices:");
        RCLCPP_INFO(this->get_logger(), "Start index (0%%): %zu, Range: %f", start_idx, msg->ranges[start_idx]);
        RCLCPP_INFO(this->get_logger(), "25%% index: %zu, Range: %f", quarter_idx, msg->ranges[quarter_idx]);
        RCLCPP_INFO(this->get_logger(), "Mid index (50%%): %zu, Range: %f", mid_idx, msg->ranges[mid_idx]);
        RCLCPP_INFO(this->get_logger(), "75%% index: %zu, Range: %f", three_quarters_idx, msg->ranges[three_quarters_idx]);

        RCLCPP_INFO(this->get_logger(), "Lowest distance index: %zu, Range: %f", min_distance_idx, min_distance);

        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanDataPrinter>();

    rclcpp::spin(node);
    
    return 0;
}
