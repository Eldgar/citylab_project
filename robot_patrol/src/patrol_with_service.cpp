#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"

class PatrolWithService : public rclcpp::Node
{
public:
    PatrolWithService() : Node("patrol_with_service")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing PatrolWithService node");

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
        int front_min_idx = (msg->ranges.size() / 2) - (msg->ranges.size()/12);
        int front_max_idx = (msg->ranges.size() / 2) + (msg->ranges.size()/12);

        bool obstacle_in_front = false;
        for (int i = front_min_idx; i <= front_max_idx; ++i)
        {
            if (msg->ranges[i] < 0.35)
            {
                obstacle_in_front = true;
                break;
            }
        }

        if (obstacle_in_front)
        {
            if (!client_->service_is_ready())
            {
                RCLCPP_ERROR(this->get_logger(), "Service not available.");
                return;
            }

            auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
            request->laser_data = *msg;

            client_->async_send_request(request,
                std::bind(&PatrolWithService::response_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Obstacle detected. Service request sent.");
        }
        else
        {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.1;
            twist_msg.angular.z = 0.0;
            velocity_publisher_->publish(twist_msg);

            RCLCPP_INFO(this->get_logger(), "Moving forward.");
        }
    }

    void response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future)
    {
        auto response = future.get();

        auto twist_msg = geometry_msgs::msg::Twist();

        if (response->direction == "left")
        {
            twist_msg.linear.x = 0.1;
            twist_msg.angular.z = 0.55;
        }
        else if (response->direction == "right")
        {
            twist_msg.linear.x = 0.1;
            twist_msg.angular.z = -0.55;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown direction received: %s", response->direction.c_str());
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
        }

        velocity_publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Published Twist message: linear.x = %f, angular.z = %f",
                    twist_msg.linear.x, twist_msg.angular.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<PatrolWithService>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
