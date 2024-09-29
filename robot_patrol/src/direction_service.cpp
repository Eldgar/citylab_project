#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <algorithm>

class DirectionService : public rclcpp::Node
{
public:
    DirectionService() : Node("direction_service")
    {
        service_ = this->create_service<robot_patrol::srv::GetDirection>(
            "/direction_service", std::bind(&DirectionService::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Service Server Ready");
    }

private:
    rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;

    void handle_service(const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
                    std::shared_ptr<robot_patrol::srv::GetDirection::Response> response)
    {
        // These are index-based calculations
        int start_idx = request->laser_data.ranges.size() / 4;
        int end_idx = 3 * (request->laser_data.ranges.size() / 4);
        size_t section_size = (end_idx - start_idx) / 3;

        // Right section
        float total_dist_right = std::accumulate(
            request->laser_data.ranges.begin() + start_idx, 
            request->laser_data.ranges.begin() + start_idx + section_size, 
            0.0f);

        // Front section
        float total_dist_front = std::accumulate(
            request->laser_data.ranges.begin() + start_idx + section_size, 
            request->laser_data.ranges.begin() + start_idx + 2 * section_size, 
            0.0f);

        // Left section
        float total_dist_left = std::accumulate(
            request->laser_data.ranges.begin() + start_idx + 2 * section_size, 
            request->laser_data.ranges.begin() + end_idx, 
            0.0f);

        // Compare totals to determine direction
        if (total_dist_front > total_dist_right && total_dist_front > total_dist_left)
        {
            response->direction = "forward";
        }
        else if (total_dist_left > total_dist_right)
        {
            response->direction = "left";
        }
        else
        {
            response->direction = "right";
        }
        RCLCPP_INFO(this->get_logger(), "Service Requested: Moving %s", response->direction.c_str());
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
