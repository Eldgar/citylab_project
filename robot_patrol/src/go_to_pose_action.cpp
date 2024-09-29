#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

class GoToPose : public rclcpp::Node
{
public:
    using GoToPoseAction = robot_patrol::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

    GoToPose() : Node("go_to_pose_action")
    {
        // Create the action server
        action_server_ = rclcpp_action::create_server<GoToPoseAction>(
            this,
            "/go_to_pose",
            std::bind(&GoToPose::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GoToPose::handle_cancel, this, std::placeholders::_1),
            std::bind(&GoToPose::handle_accepted, this, std::placeholders::_1));

        // Create publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create subscriber to the odometry topic
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GoToPose::odom_callback, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    geometry_msgs::msg::Pose2D current_pos_;
    geometry_msgs::msg::Pose2D desired_pos_;

    // Handle goal requests
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const GoToPoseAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request for position: (%f, %f, %f)", 
                    goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
        desired_pos_ = goal->goal_pos;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancel requests
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Start goal execution
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        std::thread{std::bind(&GoToPose::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Odometry callback
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_pos_.theta = yaw;
    }

    // Execution of the goal
    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        rclcpp::Rate rate(10);  // Control loop runs at 10 Hz
        bool reached_goal = false;

        while (rclcpp::ok() && !reached_goal)
        {
            // Compute distance and angle to the goal
            double dx = desired_pos_.x - current_pos_.x;
            double dy = desired_pos_.y - current_pos_.y;
            double distance = sqrt(dx * dx + dy * dy);
            double goal_angle = atan2(dy, dx);
            double angle_diff = goal_angle - current_pos_.theta;

            // Feedback
            auto feedback = std::make_shared<GoToPoseAction::Feedback>();
            feedback->current_pos = current_pos_;
            goal_handle->publish_feedback(feedback);

            if (distance < 0.05 && fabs(angle_diff) < 0.05)  // Goal tolerance
            {
                reached_goal = true;
                RCLCPP_INFO(this->get_logger(), "Goal reached");
                break;
            }

            // Create velocity command
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = angle_diff;
            velocity_publisher_->publish(cmd_vel);

            rate.sleep();
        }

        // Goal result
        auto result = std::make_shared<GoToPoseAction::Result>();
        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal execution completed");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

