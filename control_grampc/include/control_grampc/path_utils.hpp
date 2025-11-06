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
             const std::vector<double> &arc_lengths,
             const std::vector<double> &velocities = std::vector<double>());
        size_t findNextWaypointIdx(double current_s) const;
        Eigen::Vector2d getWaypoint(size_t index) const;
        double getHeading(size_t index) const;
        double getCurvature(size_t index) const;
        double getArcLength(size_t index) const;
        size_t getWaypointCount() const;
        double getTotalLength() const;
        double getVelocity(size_t index) const;

    private:
        std::vector<Eigen::Vector4d> waypoints_;
        std::vector<double> arc_lengths_;
        double total_length_;
        std::vector<double> velocities_;
    };

    // Utility: load waypoints from CSV file
    Path load_path_from_csv(const std::string &filename);

} // namespace mpcc
