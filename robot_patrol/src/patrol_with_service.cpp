#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"

class PatrolWithService : public rclcpp::Node
{
public:
    PatrolWithService() : Node("patrol_with_service")
    {
        client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PatrolWithService::laser_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
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
            auto twist_msg = geometry_msgs::msg::Twist();


            if (response->direction == "forward")
            {
                twist_msg.linear.x = 0.1;
                twist_msg.angular.z = 0.0;
            }
            else if (response->direction == "left")
            {
                twist_msg.linear.x = 0.1;
                twist_msg.angular.z = 0.5;
            }
            else if (response->direction == "right")
            {
                twist_msg.linear.x = 0.1;
                twist_msg.angular.z = -0.5;
            }

            velocity_publisher_->publish(twist_msg);
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
    auto node = std::make_shared<PatrolWithService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
