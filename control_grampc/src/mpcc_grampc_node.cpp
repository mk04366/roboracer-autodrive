#include "control_grampc/mpcc_grampc_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

MPCCGrampcNode::MPCCGrampcNode()
    : Node("mpcc_grampc_node"), x_(0.0), y_(0.0), yaw_(0.0), v_(0.0), prev_steer_(0.0), prev_throttle_(0.0), t_(0.0)
{
    // Load path from CSV file
    std::string csv_file = this->declare_parameter<std::string>("path_csv",
                                                                "/home/ammar/ros2_ws/src/global-planning/outputs/map5/traj_race_cl.csv");
    path_ = std::make_shared<mpcc::Path>(mpcc::load_path_from_csv(csv_file));

    if (path_ && path_->getTotalLength() > 1e-3)
    {
        RCLCPP_INFO(this->get_logger(), "Path loaded successfully: %s, length: %f", csv_file.c_str(),
                    path_->getTotalLength());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load path from: %s", csv_file.c_str());
        return;
    }

    // ---------------------------------------- //
    vehicle_sub_ = this->create_subscription<autodrive_msgs::msg::Vehiclestate>(
        "/autodrive/f1tenth_1/vehicle_state", 10,
        std::bind(&MPCCGrampcNode::vehicleCallback, this, std::placeholders::_1));

    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
    steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mpcc_target", 10);

    path_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MPCCGrampcNode::publishPath, this));
    // ---------------------------------------- //

    initGrampcParams();
}

void MPCCGrampcNode::publishPath()
{
    if (!path_ || path_->getWaypointCount() == 0)
        return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map"; // keep consistent with RViz fixed frame

    path_msg.poses.reserve(path_->getWaypointCount());
    for (size_t i = 0; i < path_->getWaypointCount(); ++i)
    {
        Eigen::Vector2d wp = path_->getWaypoint(i);
        double heading = path_->getHeading(i);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = wp.x();
        pose.pose.position.y = wp.y();
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, heading);
        pose.pose.orientation = tf2::toMsg(q);

        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    // optional throttle info:
    RCLCPP_DEBUG(this->get_logger(), "Published path with %zu waypoints", path_->getWaypointCount());
}


void MPCCGrampcNode::publishTarget(const Eigen::Vector2d &point, double heading)
{
    if (!target_pub_)
        return;

    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = "map";
    target_msg.pose.position.x = point.x();
    target_msg.pose.position.y = point.y();
    target_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading);
    target_msg.pose.orientation = tf2::toMsg(q);

    target_pub_->publish(target_msg);
}

void MPCCGrampcNode::initGrampcParams()
{
    L_ = 0.32;
    v_max_ = 1.0;

    /********* Parameter definition *********/
    /* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints for the states and inputs */
    ctypeRNum x0[NX] = {0.0, 0.0, 0.0, 0.0, 0.5};
    ctypeRNum xdes[NX] = {0.0, 0.0, 0.0, 0.0, 1.0}; // [x, y, theta, kappa, v]

    /* Initial values, setpoints and limits of the inputs */
    ctypeRNum u0[NU] = {0.0, 0.0};
    ctypeRNum udes[NU] = {0.0, 0.0};
    ctypeRNum umax[NU] = {1.0, M_PI / 30};
    ctypeRNum umin[NU] = {0.01, -M_PI / 30};
    ctypeRNum Thor = 1; /* Prediction horizon */
    dt_ = 0.05;         // Default 50ms for 20Hz timer
    t_ = 0.0;           /* time at the current sampling step */
	const char* Integrator = "euler";

    /********* Option definition *********/
    ctypeInt Nhor = 20; /* Number of steps for the system integration */
    ctypeInt MaxGradIter = 5;
    ctypeRNum ConstraintsAbsTol[1] = {1e-2};

    /********* userparam *********/
    /* Use member param_ so it remains valid after this function returns.
       Passing a pointer to a local stack array caused use-after-return and
       NaNs in solver outputs. */
    param_[0] = L_;   // [0] Wheelbase length
    param_[1] = 1.0;  // [1] Velocity scaling factor (stabilization term)

    /* Running-state cost weights (Q) */
    param_[2] = 1.0;  // [2] Qx
    param_[3] = 1.0;  // [3] Qy
    param_[4] = 1.0;  // [4] Qtheta
    param_[5] = 1.0;  // [5] Qkappa
    param_[6] = 1.0;  // [6] Qv

    /* Terminal-state cost weights (P) */
    param_[7] = 1.0;  // [7] Px
    param_[8] = 1.0;  // [8] Py
    param_[9] = 1.0;  // [9] Ptheta
    param_[10] = 1.0; // [10] Pkappa
    param_[11] = 1.0; // [11] Pv

    /* Control cost weights (R) */
    param_[12] = 0.01; // [12] Ra
    param_[13] = 0.01; // [13] Rsteer_rate

    /********* grampc init *********/
    grampc_ = nullptr;
    /* pass pointer to member param_ so it remains valid for solver lifetime */
    grampc_init(&grampc_, (typeUSERPARAM *)param_);

    if (!grampc_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GRAMPC solver");
        return;
    }

    /********* set parameters *********/

    grampc_setparam_real_vector(grampc_, "x0", x0);
    grampc_setparam_real_vector(grampc_, "xdes", xdes);

    grampc_setparam_real_vector(grampc_, "udes", udes);
    grampc_setparam_real_vector(grampc_, "u0", u0);
    grampc_setparam_real_vector(grampc_, "umin", umin);
    grampc_setparam_real_vector(grampc_, "umax", umax);

    grampc_setparam_real(grampc_, "Thor", Thor);
    grampc_setparam_real(grampc_, "t0", t_);
    grampc_setparam_real(grampc_, "dt", dt_);

    grampc_setopt_int(grampc_, "Nhor", Nhor);
    grampc_setopt_int(grampc_, "MaxGradIter", MaxGradIter);
    grampc_setopt_real_vector(grampc_, "ConstraintsAbsTol", ConstraintsAbsTol);

    grampc_setopt_string(grampc_, "Integrator", Integrator);

    grampc_estim_penmin(grampc_, 1);
};

void MPCCGrampcNode::initializePathPosition()
{
    // Initialize with closest waypoint on first run
    Eigen::Vector2d vehicle_pos(x_, y_);
    current_path_idx_ = 0;
    double min_dist = (vehicle_pos - path_->getWaypoint(0)).norm();
    for (size_t i = 1; i < path_->getWaypointCount(); ++i)
    {
        double dist = (vehicle_pos - path_->getWaypoint(i)).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            current_path_idx_ = i;
        }
    }
    current_s_ = path_->getArcLength(current_path_idx_);
}

double MPCCGrampcNode::getYawFromImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    tf2::Quaternion q(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw; // in radians
}

double MPCCGrampcNode::computeCurvature(
    const geometry_msgs::msg::Point::ConstSharedPtr &ips_msg,
    double current_yaw)
{
    if (first_pose_)
    {
        last_pos_ = *ips_msg;
        last_yaw_ = current_yaw;
        first_pose_ = false;
        return 0.0;
    }

    double dx = ips_msg->x - last_pos_.x;
    double dy = ips_msg->y - last_pos_.y;
    double ds = std::sqrt(dx * dx + dy * dy);

    if (ds < 1e-5)
        return 0.0; // avoid division by zero

    double dyaw = current_yaw - last_yaw_;

    // normalize yaw difference to [-pi, pi]
    while (dyaw > M_PI)
        dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI)
        dyaw += 2.0 * M_PI;

    double kappa = dyaw / ds;

    last_pos_ = *ips_msg;
    last_yaw_ = current_yaw;

    return kappa;
}

void MPCCGrampcNode::vehicleCallback(const autodrive_msgs::msg::Vehiclestate::SharedPtr msg)
{
    // Extract IPS position
    x_ = msg->position.x;
    y_ = msg->position.y;

    // Extract speed
    v_ = static_cast<double>(msg->speed);

    // Extract yaw from IMU
    auto imu_ptr = std::make_shared<sensor_msgs::msg::Imu>(msg->imu);
    yaw_ = getYawFromImu(imu_ptr);

    // Compute curvature
    auto pos_ptr = std::make_shared<geometry_msgs::msg::Point>(msg->position);
    kappa_ = computeCurvature(pos_ptr, yaw_);

    // Run control loop
    controlLoop();
}

void MPCCGrampcNode::controlLoop()
{
    initializePathPosition();
    // Find current waypoint using arc length and get next waypoint
    size_t nextIdx = path_->findNextWaypointIdx(current_s_);
    RCLCPP_INFO(this->get_logger(), "next_idx=%zu",
                nextIdx);

    // Get target waypoint and reference states
    Eigen::Vector2d target_point = path_->getWaypoint(nextIdx);
    double target_heading = path_->getHeading(nextIdx);
    double target_curvature = path_->getCurvature(nextIdx);
    double ref_speed = path_->getVelocity(nextIdx) / 2.0; // Scale down for safety

    // publish target pose for RViz visualization
    // publish target pose
    publishTarget(target_point, target_heading);

    // Current state and target state
    std::vector<double> current_state = {x_, y_, yaw_, kappa_, v_};
    std::vector<double> target_state = {target_point.x(), target_point.y(), target_heading, target_curvature, ref_speed};

    // Set current state as initial & desired condition
    grampc_setparam_real_vector(grampc_, "x0", current_state.data());
    grampc_setparam_real_vector(grampc_, "xdes", target_state.data());

    // print target state for debugging
    RCLCPP_INFO(this->get_logger(), "Target State: x=%.2f, y=%.2f, yaw=%.2f, kappa=%.4f, v=%.2f",
                target_state[0], target_state[1], target_state[2], target_state[3], target_state[4]);
    RCLCPP_INFO(this->get_logger(), "Current State: x=%.2f, y=%.2f, yaw=%.2f, kappa=%.4f, v=%.2f",
                current_state[0], current_state[1], current_state[2], current_state[3], current_state[4]);
    double steer_cmd = 0.0;
    double throttle_cmd = 0.0;

    grampc_run(grampc_);

    // log grampc_>sol->status and sol->unext
    RCLCPP_INFO(this->get_logger(), "GRAMPC solver status: %d", grampc_->sol->status);
    RCLCPP_INFO(this->get_logger(), "Next control inputs: unext[0]=%.4f, unext[1]=%.4f",
                grampc_->sol->unext[0], grampc_->sol->unext[1]);
    // Check for any GRAMPC solver issues - treat all status > 0 as errors
    if (grampc_->sol->status > 0)
    {
        RCLCPP_INFO(this->get_logger(), "GRAMPC solver error with status: %d", grampc_->sol->status);

        // Use previous commands when solver fails
        steer_cmd = prev_steer_;
        throttle_cmd = prev_throttle_;
    }
    else
    {
        /* update state and time */
        t_ = t_ + dt_;

        double acceleration = grampc_->sol->unext[0]; // u[0] = acceleration [m/s^2]
        double kappa_dot = grampc_->sol->unext[1];    // u[1] = steering_rate (curvature rate) [1/s]

        throttle_cmd = acceleration;
        steer_cmd = kappa_;

        // Update previous commands for fallback case of next iteration
        prev_steer_ = steer_cmd;
        prev_throttle_ = throttle_cmd;
    }

    // Publish steering command
    auto s_msg = std_msgs::msg::Float32();
    s_msg.data = static_cast<float>(steer_cmd);
    steering_pub_->publish(s_msg);

    // Publish throttle command
    auto t_msg = std_msgs::msg::Float32();
    t_msg.data = static_cast<float>(throttle_cmd);
    throttle_pub_->publish(t_msg);
}

MPCCGrampcNode::~MPCCGrampcNode()
{
    grampc_free(&grampc_);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting GRAMPC MPCC Node..." << std::endl;

    rclcpp::init(argc, argv);
    std::cout << "ROS2 initialized successfully" << std::endl;

    try
    {
        std::cout << "Creating MPCCGrampcNode..." << std::endl;
        auto node = std::make_shared<MPCCGrampcNode>();
        std::cout << "Node created successfully" << std::endl;

        std::cout << "Starting ROS2 spin..." << std::endl;
        rclcpp::spin(node);
        std::cout << "ROS2 spin completed" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "Unknown exception caught" << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    std::cout << "ROS2 shutdown completed" << std::endl;
    return 0;
}
