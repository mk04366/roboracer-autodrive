#ifndef CONTROL_GRAMPC_MPC_H
#define CONTROL_GRAMPC_MPC_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "state.h"
#include "input.h"
#include "model.h"
#include "cost.h"
#include "constraints.h"

extern "C" {
#include "grampc.h"
#include "grampc_setopt.h"
}

namespace control_grampc {

class MPC {
public:
    MPC();
    MPC(rclcpp::Node::SharedPtr node);
    virtual ~MPC();

    // Main MPC update function
    void update(const State& current_state, const Input& previous_input, 
               const std::vector<State>& reference_trajectory);
    
    // Visualization
    void visualize();
    
    // Update scan for constraints
    void updateScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg);
    
    // Getters
    double dt() const { return dt_; }
    int horizon() const { return horizon_; }
    const std::vector<Input>& getSolvedTrajectory() const { return solved_trajectory_; }
    
    // Check if solution is valid
    bool hasSolution() const { return has_solution_; }
    
    // Set node pointer after construction
    void setNode(rclcpp::Node::SharedPtr node);
    
    // Initialize GRAMPC solver
    void initializeGrampc();
    
    // Getter for direct GRAMPC solution (for debugging)
    Input getDirectSolution() const {
        if (grampc_ && grampc_->sol && grampc_->sol->unext && has_solution_) {
            return Input(grampc_->sol->unext[0], grampc_->sol->unext[1]);
        }
        return Input(0.0, 0.0);
    }
    
    // Setter for reference lookahead
    void setRefLookahead(double sec) { ref_lookahead_sec_ = sec; }
    
    // Getter for reference lookahead
    double refLookahead() const { return ref_lookahead_sec_; }

private:
    // GRAMPC solver
    typeGRAMPC* grampc_;
    void* ctx_; // user context allocated with malloc/calloc
    
    // MPC parameters
    int horizon_;
    int state_size_;
    int input_size_;
    double dt_;
    bool solver_initialized_;
    bool has_solution_;
    double ref_lookahead_sec_ = 0.5; // time ahead for reference selection (single point)
    
    // Solver stability tracking
    int failure_count_;
    double last_cost_;
    
    // Cost and constraints
    Cost cost_;
    Constraints constraints_;
    Model model_;
    
    // Current state and references
    State current_state_;
    std::vector<State> reference_trajectory_;
    Input previous_input_;
    
    // Solution
    std::vector<Input> solved_trajectory_;
    
    // Sensor data
    sensor_msgs::msg::LaserScan scan_msg_;
    
    // ROS2 interfaces
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpc_viz_pub_;
    
    // GRAMPC setup functions
    void updateGrampcParameters();
    void extractSolution();
    
    // Visualization helpers
    void drawCar(const State& state, const Input& input, 
                std::vector<geometry_msgs::msg::Point>& points,
                std::vector<std_msgs::msg::ColorRGBA>& colors);
};

} // namespace control_grampc

#endif // CONTROL_GRAMPC_MPC_H
