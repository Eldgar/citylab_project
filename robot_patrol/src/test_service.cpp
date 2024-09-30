#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

class TestServiceClient : public rclcpp::Node
{
public:
    TestServiceClient() : Node("test_service_client")
    {
        RCLCPP_INFO(this->get_logger(), "Service Client Ready");
        client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
        }

        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TestServiceClient::laser_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!client_->service_is_ready())
        {
            RCLCPP_ERROR(this->get_logger(), "Service is not available");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Service Request sent");

        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg;

        // Send the request asynchronously and provide a callback
        auto future_result = client_->async_send_request(request,
            std::bind(&TestServiceClient::response_callback, this, std::placeholders::_1));
    }

    void response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->direction.c_str());
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
