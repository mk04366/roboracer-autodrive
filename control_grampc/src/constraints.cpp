#include "control_grampc/constraints.h"
#include <cmath>
#include <algorithm>

namespace control_grampc {

Constraints::Constraints() 
    : max_steering_angle_(0.4)
    , min_velocity_(0.0)
    , max_velocity_(5.0)
    , safety_margin_(0.2) {}

void Constraints::findHalfSpaces(const State& state, const sensor_msgs::msg::LaserScan& scan) {
    current_state_ = state;
    processLaserScan(scan);
}

void Constraints::processLaserScan(const sensor_msgs::msg::LaserScan& scan) {
    halfspaces_.clear();
    
    // Process laser scan to extract obstacle constraints
    const double angle_min = scan.angle_min;
    const double angle_increment = scan.angle_increment;
    const double range_max = scan.range_max;
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const double range = scan.ranges[i];
        
        // Skip invalid readings
        if (std::isnan(range) || std::isinf(range) || range <= 0.0 || range >= range_max) {
            continue;
        }
        
        // Calculate obstacle position in vehicle frame
        const double angle = angle_min + i * angle_increment;
        const double obs_x = range * cos(angle);
        const double obs_y = range * sin(angle);
        
        // Only consider obstacles that are close enough to matter
        if (range < 3.0) {
            // Create halfspace constraint: normal pointing away from obstacle
            // For simplicity, use point-to-point constraints
            const double norm = sqrt(obs_x * obs_x + obs_y * obs_y);
            if (norm > 1e-6) {
                Eigen::Vector3d halfspace;
                halfspace(0) = obs_x / norm;  // normal x
                halfspace(1) = obs_y / norm;  // normal y
                halfspace(2) = -(obs_x * obs_x + obs_y * obs_y) / norm + safety_margin_;  // offset
                halfspaces_.push_back(halfspace);
            }
        }
    }
}

bool Constraints::satisfiesConstraints(const State& state) const {
    // Check input constraints (steering and velocity bounds)
    // These are handled by GRAMPC bounds, so we focus on obstacle constraints
    
    // Check distance to obstacles
    const double min_distance = distanceToObstacle(state);
    return min_distance > safety_margin_;
}

double Constraints::distanceToObstacle(const State& state) const {
    if (halfspaces_.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double min_distance = std::numeric_limits<double>::max();
    const double x = state.x();
    const double y = state.y();
    
    for (const auto& halfspace : halfspaces_) {
        const double distance = halfspace(0) * x + halfspace(1) * y + halfspace(2);
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

void Constraints::getConstraintBounds(std::vector<double>& lower_bounds, 
                                    std::vector<double>& upper_bounds) const {
    // Return bounds for input constraints
    lower_bounds = {min_velocity_, -max_steering_angle_};
    upper_bounds = {max_velocity_, max_steering_angle_};
}

} // namespace control_grampc
