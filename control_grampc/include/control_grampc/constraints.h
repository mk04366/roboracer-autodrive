#ifndef CONTROL_GRAMPC_CONSTRAINTS_H
#define CONTROL_GRAMPC_CONSTRAINTS_H

#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "state.h"

namespace control_grampc {

class Constraints {
public:
    Constraints();
    virtual ~Constraints() = default;

    // Update constraints based on current state and scan
    void findHalfSpaces(const State& state, const sensor_msgs::msg::LaserScan& scan);
    
    // Set current state for constraint calculations
    void setState(const State& state) { current_state_ = state; }
    
    // Check if constraints are satisfied
    bool satisfiesConstraints(const State& state) const;
    
    // Get constraint parameters for GRAMPC
    void getConstraintBounds(std::vector<double>& lower_bounds, 
                           std::vector<double>& upper_bounds) const;

private:
    State current_state_;
    
    // Constraint parameters
    double max_steering_angle_;
    double min_velocity_;
    double max_velocity_;
    double safety_margin_;
    
    // Obstacle constraints from LiDAR
    std::vector<Eigen::Vector3d> halfspaces_;  // ax + by + c <= 0 format
    
    // Helper functions
    void processLaserScan(const sensor_msgs::msg::LaserScan& scan);
    double distanceToObstacle(const State& state) const;
};

} // namespace control_grampc

#endif // CONTROL_GRAMPC_CONSTRAINTS_H
