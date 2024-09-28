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
        // Divide laser scan data into three sections (right, front, left)
        size_t range_size = request->laser_data.ranges.size();
        size_t section_size = range_size / 3;

        float total_dist_right = std::accumulate(request->laser_data.ranges.begin(), request->laser_data.ranges.begin() + section_size, 0.0f);
        float total_dist_front = std::accumulate(request->laser_data.ranges.begin() + section_size, request->laser_data.ranges.begin() + 2 * section_size, 0.0f);
        float total_dist_left = std::accumulate(request->laser_data.ranges.begin() + 2 * section_size, request->laser_data.ranges.end(), 0.0f);

        // Determine the safest direction based on the total distances in each section
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
