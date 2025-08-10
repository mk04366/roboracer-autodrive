#include "control/stanley_controller_node.hpp"

StanleyController::StanleyController(double wheelbase)
    : wheelbase_(wheelbase)
{
}

void StanleyController::setWaypoints(const std::vector<Waypoint> &new_waypoints)
{
    waypoints_ = new_waypoints;
}

const std::vector<Waypoint> StanleyController::getWaypoints()
{
    return waypoints_;
}

double StanleyController::pi2pi(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

// Modified to find nearest point on path segment
std::pair<size_t, std::array<double, 2>> StanleyController::nearestPoint(
    const std::array<double, 2> &point,
    const std::vector<Waypoint> &waypoints)
{
    size_t target_index = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    std::array<double, 2> nearest_on_segment = {0.0, 0.0};

    if (waypoints.empty()) {
        return {0, {0.0, 0.0}}; // Handle empty waypoints case, though it should be caught earlier
    }

    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        const Waypoint &p1 = waypoints[i];
        const Waypoint &p2 = waypoints[i + 1];

        double dx_segment = p2.x - p1.x;
        double dy_segment = p2.y - p1.y;
        double len_sq = dx_segment * dx_segment + dy_segment * dy_segment;

        double t = 0.0;
        if (len_sq > std::numeric_limits<double>::epsilon()) // Avoid division by zero for coincident points
        {
            t = ((point[0] - p1.x) * dx_segment + (point[1] - p1.y) * dy_segment) / len_sq;
        }
        
        double t_clamped = std::clamp(t, 0.0, 1.0);

        double closest_x = p1.x + t_clamped * dx_segment;
        double closest_y = p1.y + t_clamped * dy_segment;

        double current_dist_sq = (point[0] - closest_x) * (point[0] - closest_x) +
                                 (point[1] - closest_y) * (point[1] - closest_y);

        if (current_dist_sq < min_dist_sq)
        {
            min_dist_sq = current_dist_sq;
            nearest_on_segment = {closest_x, closest_y};
            
            // Determine the target_index based on which end of the segment is closer
            // If the closest point is on the segment (not an endpoint), we typically use the starting waypoint's index.
            // If it's an endpoint, we use that endpoint's index.
            if (t_clamped == 0.0) {
                target_index = i; 
            } else if (t_clamped == 1.0) {
                target_index = i + 1;
            } else {
                target_index = i; // Point is within the segment, use the start of the segment
            }
        }
    }

    // Handle the last waypoint if it's the closest point and no segment was found closer
    if (waypoints.size() > 0) {
        double dx_last = point[0] - waypoints.back().x;
        double dy_last = point[1] - waypoints.back().y;
        double dist_sq_last = dx_last * dx_last + dy_last * dy_last;
        if (dist_sq_last < min_dist_sq) {
            min_dist_sq = dist_sq_last;
            nearest_on_segment = {waypoints.back().x, waypoints.back().y};
            target_index = waypoints.size() - 1;
        }
    }

    return {target_index, nearest_on_segment};
}

std::tuple<double, double, size_t, double> StanleyController::calcThetaAndEf(
    const VehicleState &state,
    const std::vector<Waypoint> &waypoints)
{
    // 1. Compute front axle position
    double fx = state.x + wheelbase_ * std::cos(state.heading);
    double fy = state.y + wheelbase_ * std::sin(state.heading);
    std::array<double, 2> front_axle = {fx, fy};

    // 2. Find nearest point on path segment
    auto [target_index, nearest] = nearestPoint(front_axle, waypoints);

    // 3. Vector from nearest point on path to front axle
    double dx = front_axle[0] - nearest[0];
    double dy = front_axle[1] - nearest[1];

    // 4. Cross-track error (ef):
    // Calculate the signed cross-track error using the path's orientation at the target point.
    // A positive ef means the vehicle is to the left of the path.
    // To ensure this, we use the vector perpendicular to the path heading, pointing left.
    
    // Ensure target_index is valid for accessing waypoint heading
    size_t effective_target_index = target_index;
    if (waypoints.empty()) {
        // If waypoints are empty, this case should ideally be prevented earlier or handled as an error.
        // For now, return zero errors.
        return std::make_tuple(0.0, 0.0, 0, 0.0);
    }

    // If target_index is the last waypoint, and the path has at least 2 points,
    // we can use the heading of the segment leading to it, or just its own heading if available.
    // For simplicity, we'll use the target_index's heading. If the target_index is the last point, it's fine.
    
    double path_heading_at_nearest = waypoints[effective_target_index].heading;

    // Perpendicular vector to path heading, pointing to the left (counter-clockwise from path heading)
    double path_perp_x = -std::sin(path_heading_at_nearest); // cos(heading + pi/2)
    double path_perp_y = std::cos(path_heading_at_nearest);  // sin(heading + pi/2)

    double ef = dx * path_perp_x + dy * path_perp_y;
    
    // Increased clamping range for ef to allow for larger cross-track errors
    ef = std::clamp(ef, -5.0, 5.0); 

    // 5. Heading error (wrap to [-pi, pi])
    double theta_raceline = waypoints[effective_target_index].heading; // Use heading of the segment's starting point or nearest waypoint
    double theta_e = pi2pi(theta_raceline - state.heading);

    // 6. Target velocity
    double goal_velocity = waypoints[effective_target_index].velocity;

    return std::make_tuple(theta_e, ef, effective_target_index, goal_velocity);
}

std::pair<double, double> StanleyController::controller(
    const VehicleState &state,
    const std::vector<Waypoint> &waypoints,
    double k_path)
{
    const double max_steering_angle = M_PI / 8; // 22.5 degrees limit
    auto [theta_e, ef, target_index, goal_velocity] = calcThetaAndEf(state, waypoints);

    // 7. Protect against zero or near-zero velocity to avoid division issues
    double velocity_safe = std::max(0.1, std::abs(state.velocity));

    // 8. Stanley control law: steering = heading error + arctan(k * cross_track_error / velocity)
    double max_cte_correction = M_PI / 6; // e.g., 30 degrees
    double cte_term = std::clamp(std::atan2(k_path * ef, velocity_safe),
                                 -max_cte_correction, max_cte_correction);

    double steering_angle = theta_e + cte_term;

    // 9. Clamp steering angle to physical limits
    steering_angle = std::clamp(steering_angle, -max_steering_angle, max_steering_angle);

    return {steering_angle, goal_velocity};
}

std::pair<double, double> StanleyController::plan(
    double pose_x,
    double pose_y,
    double pose_theta,
    double velocity,
    double k_path,
    const std::vector<Waypoint> *override_waypoints)
{
    if (override_waypoints)
    {
        if (override_waypoints->empty())
        {
            throw std::invalid_argument("Override waypoints are empty");
        }
        return controller({pose_x, pose_y, pose_theta, velocity}, *override_waypoints, k_path);
    }

    if (waypoints_.empty())
    {
        throw std::runtime_error("No waypoints provided.");
    }

    return controller({pose_x, pose_y, pose_theta, velocity}, waypoints_, k_path);
}
