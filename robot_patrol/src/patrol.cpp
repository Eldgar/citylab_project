#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#define M_PI 3.14159265358979323846
class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol")
    {
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1));

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        direction_ = 0.0;

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::control_loop, this));
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double direction_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int start_idx = msg->ranges.size() / 4;
        int end_idx = 3 * (msg->ranges.size() / 4);
        int front_idx = msg->ranges.size() / 2;
        int safest_idx = msg->ranges.size() / 2;
        float front_distance = msg->ranges[front_idx];
        float max_distance = 0.0;


        if (front_distance < 0.35)
        {
            for (int i = start_idx; i < end_idx; i++)
            {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_distance)
                {
                    max_distance = msg->ranges[i];
                    safest_idx = i;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Index:  %d", safest_idx);
            direction_ = (static_cast<double>(safest_idx) - (static_cast<double>(msg->ranges.size()) / 2.0)) * M_PI / (static_cast<double>(msg->ranges.size()) / 2.0);

            RCLCPP_INFO(this->get_logger(), "Index Difference:  %d", (safest_idx - 330));
            RCLCPP_INFO(this->get_logger(), "Direction:  %f", direction_);
        }
        else
        {
            direction_ = 0.0;
        }
    }

    void control_loop()
    {
        auto twist_msg = geometry_msgs::msg::Twist();

        twist_msg.linear.x = 0.1;

        // Inside your control loop or appropriate function
        if (direction_ > 2.0 || direction_ < -2.0)
        {
            // Log the error if direction is out of range
            RCLCPP_ERROR(this->get_logger(), "Direction value %f is out of bounds! It should be between -2 and 2.", direction_);

            // Optionally, you can stop the robot or take corrective action
            twist_msg.angular.z = 0.0;  // Set angular velocity to 0 to stop turning
            twist_msg.linear.x = 0.0;   // Optionally stop linear motion as well

            // Publish the twist message to stop the robot
            velocity_publisher_->publish(twist_msg);

            return;  // Return or handle the situation as needed
        }
        else
        {
            // If direction is within bounds, proceed with normal motion
            if (direction_ != 0.0)
            {
                twist_msg.angular.z = direction_ / 2.0;
            }
            else
            {
                twist_msg.angular.z = 0.0;
            }

            velocity_publisher_->publish(twist_msg);
        }

        velocity_publisher_->publish(twist_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
