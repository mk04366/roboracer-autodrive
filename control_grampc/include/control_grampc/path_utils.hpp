#pragma once
#include <vector>
#include <Eigen/Dense>
#include <string>

namespace mpcc
{

    class Path
    {
    public:
        Path(const std::vector<Eigen::Vector4d> &waypoints,
             const std::vector<double> &time_profile,
             const std::vector<double> &velocities = std::vector<double>());
        size_t findNextWaypointIdx(double current_time, double horizon_time) const;
        Eigen::Vector2d getWaypoint(size_t index) const;
        double getHeading(size_t index) const;
        double getCurvature(size_t index) const;
        double getTimeFromIndex(size_t index) const;
        size_t getTotalLength() const;
        double getVelocity(size_t index) const;

    private:
        std::vector<Eigen::Vector4d> waypoints_;
        std::vector<double> time_profile_;
        std::vector<double> velocities_;
    };

    // Utility: load waypoints from CSV file
    Path load_path_from_csv(const std::string &filename);

} // namespace mpcc
