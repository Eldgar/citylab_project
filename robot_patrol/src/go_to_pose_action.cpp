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
        // Convert theta from degrees to radians
        double theta_radians = goal->goal_pos.theta * (M_PI / 180.0);
        desired_pos_ = goal->goal_pos;
        desired_pos_.theta = theta_radians; // Store converted theta
        RCLCPP_INFO(this->get_logger(), "Received goal request: Position (%f, %f), Orientation (theta = %f radians)", 
                    desired_pos_.x, desired_pos_.y, desired_pos_.theta);
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
        rclcpp::Rate rate(20);
        bool reached_goal = false;

        while (rclcpp::ok() && !reached_goal)
        {
            // Compute distance and angle to the goal
            double dx = desired_pos_.x - current_pos_.x;
            double dy = desired_pos_.y - current_pos_.y;
            double distance = sqrt(dx * dx + dy * dy);
            double goal_angle = atan2(dy, dx);
            double angle_diff = goal_angle - current_pos_.theta;

            // Publish feedback
            auto feedback = std::make_shared<GoToPoseAction::Feedback>();
            feedback->current_pos = current_pos_;
            goal_handle->publish_feedback(feedback);

            if (distance < 0.04)
            {
                reached_goal = true;
                RCLCPP_INFO(this->get_logger(), "Position goal reached");

                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.angular.z = 0.0;
                velocity_publisher_->publish(stop_cmd);

                rotate_to_goal_theta();

                auto result = std::make_shared<GoToPoseAction::Result>();
                result->status = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal execution completed");
                return;
            }

            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = angle_diff * 1.5;
            velocity_publisher_->publish(cmd_vel);

            rate.sleep();
        }
    }

    void rotate_to_goal_theta()
    {
        rclcpp::Rate rate(20);
        bool rotation_done = false;
        
        while (rclcpp::ok() && !rotation_done)
        {
            double angle_diff = desired_pos_.theta - current_pos_.theta;
            if (fabs(angle_diff) < 0.02) 
            {
                rotation_done = true;
                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.angular.z = 0.0;
                velocity_publisher_->publish(stop_cmd);
                RCLCPP_INFO(this->get_logger(), "Orientation goal reached, stopping");
            }
            else
            {
                geometry_msgs::msg::Twist rotate_cmd;
                rotate_cmd.angular.z = angle_diff * 1.5;
                velocity_publisher_->publish(rotate_cmd);
            }

            rate.sleep();
        }
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



