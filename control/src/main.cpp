#include "rclcpp/rclcpp.hpp"
#include "pid_controller_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_controller_node::PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}