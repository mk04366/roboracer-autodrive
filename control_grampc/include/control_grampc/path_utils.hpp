#pragma once
#include <vector>
#include <Eigen/Dense>
#include <string>

namespace mpcc
{

class Path
{
public:
    Path(const std::vector<Eigen::Vector2d> &waypoints);

    // New simplified interface - find closest waypoint directly
    size_t findClosestWaypoint(const Eigen::Vector2d& position) const;
    Eigen::Vector2d getWaypoint(size_t index) const;
    
    // Advanced lookahead that ensures target is always ahead along the path
    std::pair<Eigen::Vector2d, size_t> getTargetWaypointAhead(const Eigen::Vector2d& position, double lookahead_distance = 2.0) const;
    
    double getWaypointHeading(size_t index) const;
    size_t getWaypointCount() const;
    
    // Arc-length based interface
    Eigen::Vector2d interpolate(double s) const;
    double curvature(double s) const;
    double heading(double s) const;
    double total_length() const;

private:
    std::vector<Eigen::Vector2d> waypoints_;
    double total_length_;
};

// Utility: load waypoints from CSV file
Path load_path_from_csv(const std::string &filename);

} // namespace mpcc
