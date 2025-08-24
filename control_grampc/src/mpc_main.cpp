#include "control_grampc/mpc_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<control_grampc::MPCNode>();
        
        RCLCPP_INFO(node->get_logger(), "Starting MPC control node...");
        
        // Initialize MPC after the node is fully constructed
        node->initializeMPC();
        
        // Use multi-threaded executor for better performance
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mpc_main"), "Exception in main: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
