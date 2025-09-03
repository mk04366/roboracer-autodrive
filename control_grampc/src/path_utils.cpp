#include "control_grampc/path_utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

namespace mpcc
{

    Path::Path(const std::vector<Eigen::Vector2d> &waypoints)
        : waypoints_(waypoints), total_length_(0.0)
    {
        for (size_t i = 1; i < waypoints_.size(); ++i)
        {
            total_length_ += (waypoints_[i] - waypoints_[i - 1]).norm();
        }
    }

    Eigen::Vector2d Path::interpolate(double s) const
    {
        if (this->waypoints_.empty())
            return Eigen::Vector2d::Zero();

        if (s <= 0.0)
            return this->waypoints_.front();
        if (s >= this->total_length_)
            return this->waypoints_.back();

        double accumulated = 0.0;
        for (size_t i = 1; i < this->waypoints_.size(); ++i)
        {
            double seg_len = (this->waypoints_[i] - this->waypoints_[i - 1]).norm();
            if (accumulated + seg_len >= s)
            {
                double ratio = (s - accumulated) / seg_len;
                return this->waypoints_[i - 1] + ratio * (this->waypoints_[i] - this->waypoints_[i - 1]);
            }
            accumulated += seg_len;
        }
        return this->waypoints_.back();
    }

    double Path::curvature(double s) const
    {
        const double ds = 0.1;
        Eigen::Vector2d p_prev = interpolate(std::max(0.0, s - ds));
        Eigen::Vector2d p = interpolate(s);
        Eigen::Vector2d p_next = interpolate(std::min(total_length_, s + ds));

        Eigen::Vector2d d1 = (p_next - p_prev) / (2 * ds);
        Eigen::Vector2d d2 = (p_next - 2.0 * p + p_prev) / (ds * ds);

        double num = d1.x() * d2.y() - d1.y() * d2.x();
        double den = std::pow(d1.squaredNorm(), 1.5);

        if (den < 1e-6)
            return 0.0;

        return num / den;
    }

    double Path::heading(double s) const
    {
        const double ds = 0.1;
        Eigen::Vector2d p_prev = interpolate(std::max(0.0, s - ds));
        Eigen::Vector2d p_next = interpolate(std::min(this->total_length_, s + ds));

        Eigen::Vector2d d = (p_next - p_prev).normalized();
        return std::atan2(d.y(), d.x());
    }

    double Path::total_length() const
    {
        return this->total_length_;
    }

    // New simplified interface implementation
    size_t Path::findClosestWaypoint(const Eigen::Vector2d& position) const
    {
        if (waypoints_.empty()) return 0;
        
        size_t closest_idx = 0;
        double min_distance = (position - waypoints_[0]).norm();
        
        for (size_t i = 1; i < waypoints_.size(); ++i)
        {
            double distance = (position - waypoints_[i]).norm();
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }

    Eigen::Vector2d Path::getWaypoint(size_t index) const
    {
        if (index >= waypoints_.size()) return waypoints_.back();
        return waypoints_[index];
    }

    std::pair<Eigen::Vector2d, size_t> Path::getTargetWaypointAhead(const Eigen::Vector2d& position, double lookahead_distance) const
    {
        if (waypoints_.empty()) 
            return std::make_pair(Eigen::Vector2d::Zero(), 0);
        
        size_t closest_idx = findClosestWaypoint(position);
        Eigen::Vector2d closest_point = waypoints_[closest_idx];
        
        // Find target point ahead of current position along the path
        size_t target_idx = closest_idx;
        Eigen::Vector2d target_point = closest_point;
        double accumulated_distance = 0.0;
        
        // Move forward along the path until we reach the desired lookahead distance
        for (size_t i = closest_idx; i < waypoints_.size() - 1; ++i) {
            Eigen::Vector2d current_wp = waypoints_[i];
            Eigen::Vector2d next_wp = waypoints_[i + 1];
            double segment_length = (next_wp - current_wp).norm();
            
            if (accumulated_distance + segment_length >= lookahead_distance) {
                // Interpolate to get exact lookahead point
                double remaining_distance = lookahead_distance - accumulated_distance;
                double t = remaining_distance / segment_length;
                target_point = current_wp + t * (next_wp - current_wp);
                target_idx = i + 1;
                break;
            }
            
            accumulated_distance += segment_length;
            target_point = next_wp;
            target_idx = i + 1;
        }
        
        // If we've reached the end of the path, use the last waypoint
        if (target_idx >= waypoints_.size() - 1) {
            target_idx = waypoints_.size() - 1;
            target_point = waypoints_[target_idx];
        }
        
        return std::make_pair(target_point, target_idx);
    }

    double Path::getWaypointHeading(size_t index) const
    {
        if (waypoints_.size() < 2) return 0.0;
        
        // Use next waypoint for direction if available, otherwise previous
        Eigen::Vector2d direction;
        if (index < waypoints_.size() - 1)
        {
            direction = (waypoints_[index + 1] - waypoints_[index]).normalized();
        }
        else if (index > 0)
        {
            direction = (waypoints_[index] - waypoints_[index - 1]).normalized();
        }
        else
        {
            return 0.0;
        }
        
        return std::atan2(direction.y(), direction.x());
    }

    size_t Path::getWaypointCount() const
    {
        return waypoints_.size();
    }

    Path load_path_from_csv(const std::string &filename)
    {
        std::ifstream file(filename);
        std::string line;
        std::vector<Eigen::Vector2d> waypoints;

        while (std::getline(file, line))
        {
            // Skip comment lines
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            // Parse CSV with semicolon separators
            // Format: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;

            while (std::getline(ss, token, ';'))
            {
                tokens.push_back(token);
            }

            if (tokens.size() >= 3)
            {
                try
                {
                    double s = std::stod(tokens[0]); // arc length (not used directly)
                    double x = std::stod(tokens[1]); // x coordinate
                    double y = std::stod(tokens[2]); // y coordinate
                    waypoints.emplace_back(x, y);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
                }
            }
        }

        if (waypoints.size() < 2)
        {
            std::cerr << "Error: Not enough waypoints loaded from " << filename << std::endl;
        }

        return Path(waypoints);
    }

} // namespace mpcc
