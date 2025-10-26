#include "control_grampc/path_utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

namespace mpcc
{

    Path::Path(const std::vector<Eigen::Vector4d> &waypoints,
               const std::vector<double> &arc_lengths,
               const std::vector<double> &velocities = std::vector<double>())
        : waypoints_(waypoints), arc_lengths_(arc_lengths), velocities_(velocities), total_length_(0.0)
    {
        if (!arc_lengths_.empty())
            total_length_ = arc_lengths_.back();
        else
        {
            for (size_t i = 1; i < waypoints_.size(); ++i)
                total_length_ += (waypoints_[i].head<2>() - waypoints_[i - 1].head<2>()).norm();
        }

        // If velocities not provided, fill with zeros
        if (velocities_.empty())
            velocities_ = std::vector<double>(waypoints_.size(), 0.0);
    }

    // Find closest waypoint using arc length (s_m value)
    size_t Path::findNextWaypointIdx(double current_s) const
    {
        if (arc_lengths_.empty())
            return 0;

        // Find the first waypoint whose arc length is strictly greater than current_s
        for (size_t i = 0; i < arc_lengths_.size(); ++i)
        {
            if (arc_lengths_[i] > current_s)
            {
                return i;
            }
        }
        // If none is greater, return the start of the path to loop around
        return 0;
    }

    Eigen::Vector2d Path::getWaypoint(size_t index) const
    {
        if (index >= waypoints_.size())
            return waypoints_.back().head<2>();
        return waypoints_[index].head<2>();
    }

    double Path::getHeading(size_t index) const
    {
        if (index >= waypoints_.size())
            return waypoints_.back()(2);
        return waypoints_[index](2);
    }

    double Path::getVelocity(size_t index) const
    {
        if (index >= velocities_.size())
            return velocities_.back();
        return velocities_[index];
    }

    double Path::getCurvature(size_t index) const
    {
        if (index >= waypoints_.size())
            return waypoints_.back()(3);
        return waypoints_[index](3);
    }

    double Path::getArcLength(size_t index) const
    {
        if (index >= arc_lengths_.size())
            return arc_lengths_.empty() ? 0.0 : arc_lengths_.back();
        return arc_lengths_[index];
    }

    size_t Path::getWaypointCount() const
    {
        return waypoints_.size();
    }

    Path load_path_from_csv(const std::string &filename)
    {
        std::ifstream file(filename);
        std::string line;
        std::vector<Eigen::Vector4d> waypoints;
        std::vector<double> arc_lengths;
        std::vector<double> velocities;

        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#')
                continue;

            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ';'))
                tokens.push_back(token);

            if (tokens.size() >= 6) // we need at least 6 columns now
            {
                try
                {
                    double s = std::stod(tokens[0]);
                    double x = std::stod(tokens[1]);
                    double y = std::stod(tokens[2]);
                    double heading = std::stod(tokens[3]);
                    double curvature = std::stod(tokens[4]);
                    double vx = std::stod(tokens[5]);

                    waypoints.emplace_back(x, y, heading, curvature);
                    arc_lengths.push_back(s);
                    velocities.push_back(vx);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
                }
            }
        }

        return Path(waypoints, arc_lengths, velocities);
    }

} // namespace mpcc
