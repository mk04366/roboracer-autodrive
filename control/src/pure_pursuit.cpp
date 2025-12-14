#include "control/pure_pursuit.hpp"

PurePursuitController::PurePursuitController()
    : Node("pure_pursuit_autodrive"),
      throttle_pid_(0.0, 0.0, 0.0, -1.0, 1.0)
{
    // Parameters (tunable)
    max_speed_ = this->declare_parameter("max_speed", 20.0);
    min_speed_ = this->declare_parameter("min_speed", 1.0);
    max_lookahead_ = this->declare_parameter("max_lookahead", 1.3);
    min_lookahead_ = this->declare_parameter("min_lookahead", 1.0);
    wheelbase_ = this->declare_parameter("wheelbase", 0.33);
    beta_ = this->declare_parameter("beta", 0.5);
    heading_scale_ = this->declare_parameter("heading_scale", 1.1);
    area_threshold_ = this->declare_parameter("area_threshold", 1.0);
    window_size_ = this->declare_parameter("window_size", 5);
    vel_window = this->declare_parameter("vel_window", 5);

    // PID parameters for throttle control
    pid_kp_ = this->declare_parameter("pid_kp", 0.15);
    pid_ki_ = this->declare_parameter("pid_ki", 0.02);
    pid_kd_ = this->declare_parameter("pid_kd", 0.01);
    throttle_pid_.setGains(pid_kp_, pid_ki_, pid_kd_);

    // Initialize position and IMU data
    current_quaternion_ = {0.0, 0.0, 0.0, 1.0};
    current_speed_ = 0.1;

    // **Initialize with given starting position**
    Eigen::Vector2d initial_position_(0.0, 0.0);
    current_position_ = initial_position_;
    previous_position_ = initial_position_;

    // Initialize timing
    last_control_time_ = this->get_clock()->now();

    vehicle_sub_ = this->create_subscription<autodrive_msgs::msg::Vehiclestate>(
        "/autodrive/f1tenth_1/vehicle_state", 10,
        std::bind(&PurePursuitController::vehicle_callback, this, std::placeholders::_1));

    // Publish actuators
    steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);

    // For RViz visualization
    goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal", 10);
    cp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cp", 10);
    race_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/raceline", 10);

    // **Add publisher for estimated position visualization**
    est_pos_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/estimated_position", 10);

    // Load the path
    load_raceline_csv("/home/ammar/ros2_ws/src/global-planning/outputs/map5/ay_safe_2.csv");
}

void PurePursuitController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    yaw_ = get_yaw_from_imu(msg);

    current_quaternion_ = {msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w};
}

void PurePursuitController::speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_speed_ = msg->data;
}

void PurePursuitController::vehicle_callback(const autodrive_msgs::msg::Vehiclestate::SharedPtr msg)
{
    imu_callback(std::make_shared<sensor_msgs::msg::Imu>(msg->imu));
    speed_callback(std::make_shared<std_msgs::msg::Float32>(std_msgs::msg::Float32().set__data(msg->speed)));
    current_position_ = Eigen::Vector2d(msg->position.x, msg->position.y);

    main_control_loop();
}

void PurePursuitController::main_control_loop()
{
    // Visualize estimated position
    publish_estimated_position();

    update_lookahead_distance(current_speed_);
    auto [closest_point, goal_point] = find_lookahead_point();

    if (goal_point.has_value())
    {
        double alpha = calculate_alpha(goal_point.value(), yaw_);
        heading_angle_ = calculate_heading_angle(alpha);
        calculate_deviation(current_position_.value(), closest_point);
        double curvature = calculate_curvature(alpha);
        double max_velocity_pp = calculate_max_velocity_pure_pursuit(curvature);
        double min_deviation_pp = calculate_min_deviation_pure_pursuit();
        control_velocity_ = convex_combination(max_velocity_pp, min_deviation_pp, current_speed_);
        publish_markers(closest_point, goal_point.value());
        publish_raceline_visualization();
        publish_control_commands();
    }
}

/**
 * Publish visualization marker for estimated position
 * Shows where the Kalman filter thinks the vehicle is
 */
void PurePursuitController::publish_estimated_position()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position
    marker.pose.position.x = current_position_->x();
    marker.pose.position.y = current_position_->y();
    marker.pose.position.z = 0.1;

    // Orientation (from IMU)
    marker.pose.orientation.x = current_quaternion_[0];
    marker.pose.orientation.y = current_quaternion_[1];
    marker.pose.orientation.z = current_quaternion_[2];
    marker.pose.orientation.w = current_quaternion_[3];

    // Scale
    marker.scale.x = 0.3;  // Arrow length
    marker.scale.y = 0.05; // Arrow width
    marker.scale.z = 0.05; // Arrow height

    // Color - Purple for estimated position
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    est_pos_pub_->publish(marker);
}

void PurePursuitController::load_raceline_csv(const std::string &filename)
{

    std::ifstream file(filename);
    std::string line;
    std::vector<Eigen::Vector2d> temp_path;

    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ','))
            tokens.push_back(token);

        if (tokens.size() >= 6) // we need at least 6 columns now
        {
            try
            {
                double x = std::stod(tokens[1]);
                double y = std::stod(tokens[2]);
                temp_path.emplace_back(x, y);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error parsing line: " << line << " - " << e.what() << std::endl;
            }
        }
    }

    path_ = temp_path;
}

double PurePursuitController::get_yaw_from_imu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    tf2::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw; // in radians
}

// we essentially use a sigmoid function to map speed to lookahead distance
void PurePursuitController::update_lookahead_distance(double speed)
{
    double normalized_speed = (speed - min_speed_) / (max_speed_ - min_speed_);
    double sigmoid_value = 1.0 / (1.0 + std::exp(-(normalized_speed * 10 - 5)));
    if (speed < min_speed_)
        lookahead_distance_ = min_lookahead_;
    else
        lookahead_distance_ = std::min(max_lookahead_, min_lookahead_ + sigmoid_value * (max_lookahead_ - min_lookahead_));
}

std::pair<Eigen::Vector2d, std::optional<Eigen::Vector2d>> PurePursuitController::find_lookahead_point()
{
    Eigen::Vector2d closest_point;
    std::optional<Eigen::Vector2d> goal_point;
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < path_.size(); ++i)
    {
        double dist = (path_[i] - current_position_.value()).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_point = path_[i];
            closest_idx = i;
        }
    }

    for (size_t i = closest_idx + 2; i < std::min(path_.size(), closest_idx + 10); ++i)
    {
        double dist = (path_[i] - current_position_.value()).norm();
        RCLCPP_INFO(this->get_logger(), "Distance to point %zu: %f", i, dist);
        if (dist > lookahead_distance_)
        {
            goal_point = path_[i];
            break;
        }
    }
    return {closest_point, goal_point};
}

double PurePursuitController::calculate_alpha(const Eigen::Vector2d &goal_point, double yaw)
{
    Eigen::Vector2d delta = goal_point - current_position_.value();
    double lx = delta.x() * std::cos(-yaw) - delta.y() * std::sin(-yaw);
    double ly = delta.x() * std::sin(-yaw) + delta.y() * std::cos(-yaw);
    return std::atan2(ly, lx);
}

double PurePursuitController::calculate_heading_angle(double alpha)
{
    return std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance_);
}

double PurePursuitController::calculate_curvature(double alpha)
{
    return 2.0 * std::sin(alpha) / lookahead_distance_;
}

//calculates the deviation in a sliding window fashion
void PurePursuitController::calculate_deviation(const Eigen::Vector2d &pos, const Eigen::Vector2d &closest)
{
    double deviation = (closest - pos).norm();
    if (previous_position_.has_value())
    {
        double dist_travel = (pos - previous_position_.value()).norm();
        double area_inc = (deviation + previous_deviation_) / 2.0 * dist_travel;
        area_window_.push_back(area_inc);
        if (area_window_.size() > window_size_)
            area_window_.pop_front();
        total_area_ = std::accumulate(area_window_.begin(), area_window_.end(), 0.0);
    }
    previous_position_ = pos;
    previous_deviation_ = deviation;
}

double PurePursuitController::calculate_max_velocity_pure_pursuit(double curvature)
{
    double max_vel = (curvature != 0.0) ? std::sqrt(1.0 / std::abs(curvature)) : max_speed_;
    return std::min(max_speed_, max_vel);
}

double PurePursuitController::calculate_min_deviation_pure_pursuit()
{
    return (total_area_ > 0.0) ? max_speed_ / (1.0 + total_area_) : max_speed_;
}

double PurePursuitController::adjust_beta(double current_speed)
{
    if (total_area_ < area_threshold_)
        return std::min(1.0, beta_ + 0.25);
    else if (current_speed > min_speed_)
        return std::max(0.0, beta_ - 0.25);
    return beta_;
}

double PurePursuitController::convex_combination(double max_v_pp, double min_d_pp, double cur_spd)
{
    beta_ = adjust_beta(cur_spd);
    double control_v = beta_ * max_v_pp + (1.0 - beta_) * min_d_pp;

    velocities_.push_back(control_v);
    if (velocities_.size() > vel_window)
        velocities_.erase(velocities_.begin());

    std::vector<double> weights;
    for (size_t i = 0; i < velocities_.size(); ++i)
        weights.push_back(std::pow(r_, i));
    double sum_w = std::accumulate(weights.begin(), weights.end(), 0.0);
    for (auto &w : weights)
        w /= sum_w;

    double moving_avg = 0.0;
    auto weight_it = weights.rbegin();
    for (auto vel_it = velocities_.rbegin(); vel_it != velocities_.rend(); ++vel_it, ++weight_it)
        moving_avg += (*vel_it) * (*weight_it);
    return moving_avg;
}

void PurePursuitController::publish_markers(const Eigen::Vector2d &closest_point, const Eigen::Vector2d &goal_point)
{
    auto create_marker = [&](const Eigen::Vector2d &point, float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.x();
        marker.pose.position.y = point.y();
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        return marker;
    };
    cp_pub_->publish(create_marker(closest_point, 0.0, 0.0, 1.0));
    goal_pub_->publish(create_marker(goal_point, 1.0, 0.0, 0.0));
}

void PurePursuitController::publish_raceline_visualization()
{
    visualization_msgs::msg::MarkerArray raceline_markers;
    int id = 0;
    for (const auto &point : path_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.x();
        marker.pose.position.y = point.y();
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.09;
        marker.scale.y = 0.09;
        marker.scale.z = 0.09;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.id = id++;
        raceline_markers.markers.push_back(marker);
    }
    race_pub_->publish(raceline_markers);
}

/**
 * Publish control command
 * s with PID-based throttle control
 * Converts desired velocity to normalized throttle command using PID
 */
void PurePursuitController::publish_control_commands()
{
    // Publish steering command
    std_msgs::msg::Float32 steer_cmd;
    steer_cmd.data = std::clamp(heading_angle_ * heading_scale_, -0.52359877559, 0.52359877559);
    steering_pub_->publish(steer_cmd);

    // Calculate time step for PID
    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_control_time_).seconds();

    last_control_time_ = current_time;

    // Use PID to compute throttle command
    double throttle_command = throttle_pid_.compute(control_velocity_, current_speed_, dt);

    // Publish normalized throttle command [-1.0, 1.0]
    std_msgs::msg::Float32 throttle_cmd;
    throttle_cmd.data = static_cast<float>(throttle_command);
    throttle_pub_->publish(throttle_cmd);

    // Debug output (throttled)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Control - Desired Speed: %.2f, Current Speed: %.2f, Throttle: %.3f, Steering: %.3f",
                         control_velocity_, current_speed_, throttle_command, steer_cmd.data);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
