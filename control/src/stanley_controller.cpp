#include "control/stanley_controller_node.hpp"

StanleyController::StanleyController(double wheelbase)
    : wheelbase_(wheelbase) {}

void StanleyController::setWaypoints(const std::vector<Waypoint> &new_waypoints)
{
    waypoints_ = new_waypoints;
}

double StanleyController::pi2pi(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

std::pair<size_t, std::array<double, 2>> StanleyController::nearestPoint(
    const std::array<double, 2> &point,
    const std::vector<Waypoint> &waypoints)
{
    size_t index = 0;
    double min_dist = std::numeric_limits<double>::max();
    std::array<double, 2> nearest;

    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        double dx = point[0] - waypoints[i].x;
        double dy = point[1] - waypoints[i].y;
        double dist = dx * dx + dy * dy;
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest = {waypoints[i].x, waypoints[i].y};
            index = i;
        }
    }

    return {index, nearest};
}

std::tuple<double, double, size_t, double> StanleyController::calcThetaAndEf(
    const VehicleState &state,
    const std::vector<Waypoint> &waypoints)
{
    double fx = state.x + wheelbase_ * std::cos(state.heading);
    double fy = state.y + wheelbase_ * std::sin(state.heading);
    std::array<double, 2> front_axle = {fx, fy};

    auto [target_index, nearest] = nearestPoint(front_axle, waypoints);
    double dx = front_axle[0] - nearest[0];
    double dy = front_axle[1] - nearest[1];

    double perp_heading = state.heading - M_PI_2;
    double ef = dx * std::cos(perp_heading) + dy * std::sin(perp_heading);
    double theta_raceline = waypoints[target_index].heading;
    double theta_e = pi2pi(theta_raceline - state.heading);
    double goal_velocity = waypoints[target_index].velocity;

    return std::make_tuple(theta_e, ef, target_index, goal_velocity);
}

std::pair<double, double> StanleyController::controller(
    const VehicleState &state,
    const std::vector<Waypoint> &waypoints,
    double k_path)
{
    auto [theta_e, ef, _, goal_velocity] = calcThetaAndEf(state, waypoints);
    double cte_term = std::atan2(k_path * ef, state.velocity);
    double steering_angle = cte_term + theta_e;
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
