#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

class TestServiceClient : public rclcpp::Node
{
public:
    TestServiceClient() : Node("test_service_client")
    {
        client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TestServiceClient::laser_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg;

        auto future = client_->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->direction.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestServiceClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
