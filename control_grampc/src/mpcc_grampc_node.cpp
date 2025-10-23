#include "control_grampc/mpcc_grampc_node.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include <Quaternion.hpp>
#include <Matrix3x3.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

MPCCGrampcNode::MPCCGrampcNode()
    : Node("mpcc_grampc_node"), x_(0.0), y_(0.0), yaw_(0.0), v_(0.0), prev_steer_(0.0), prev_throttle_(0.0), t(0.0)
{
    // Load path from CSV file
    std::string csv_file = this->declare_parameter<std::string>("path_csv",
                                                                "/home/ammar/ros2_ws/src/global-planning/outputs/map5/traj_race_cl.csv");
    path_ = std::make_shared<mpcc::Path>(mpcc::load_path_from_csv(csv_file));

    if (path_ && path_->total_length() > 1e-3)
    {
        RCLCPP_INFO(this->get_logger(), "Path loaded successfully: %s, length: %f", csv_file.c_str(),
                    path_->total_length());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load path from: %s", csv_file.c_str());
        return;
    }

    // Create message_filters subscribers
    ips_sub_.subscribe(this, "/autodrive/f1tenth_1/ips");
    speed_sub_.subscribe(this, "/autodrive/f1tenth_1/speed");
    imu_sub_.subscribe(this, "/autodrive/f1tenth_1/imu");

    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::Point,
        std_msgs::msg::Float32,
        sensor_msgs::msg::Imu>
        ApproxSyncPolicy;

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(10), ips_sub_, speed_sub_, imu_sub_);

    // Register the synchronized callback
    sync_->registerCallback(std::bind(&MPCCGrampcNode::controlLoop, this, _1, _2, _3));

    RCLCPP_INFO(this->get_logger(), "MPCCGrampcNode synchronized subscribers initialized.");

    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
    steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

    initGrampcParams();
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
    ctypeRNum umax[NU] = {0.5, 5.0};
    ctypeRNum umin[NU] = {-0.5, -5.0};
    ctypeRNum u0[NU] = {0.0, 0.0};
    ctypeRNum udes[NU] = {0.0, 0.0};
    ctypeRNum umax[NU] = {1.0, M_PI / 6};
    ctypeRNum umin[NU] = {-0.1, -M_PI / 6};
    ctypeRNum Thor = 1; /* Prediction horizon */
    dt_ = 0.05;         // Default 50ms for 20Hz timer
    typeRNum t = 0.0;   /* time at the current sampling step */

    /********* Option definition *********/
    ctypeInt Nhor = 20; /* Number of steps for the system integration */
    ctypeInt MaxGradIter = 5;
    ctypeRNum ConstraintsAbsTol[1] = {1e-2};

    /********* userparam *********/
    typeRNum pSys[14] = {L_, 50,
                         0.0, 1.0, 1.0, 1.0, 100.0,
                         0.0, 1.0, 1.0, 1.0, 100.0,
                         ((typeRNum)0.01), ((typeRNum)0.01)};
    typeUSERPARAM *userparam = pSys;

    /********* grampc init *********/
    grampc_ = nullptr;
    grampc_init(&grampc_, userparam);

    if (!grampc_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GRAMPC solver");
        return;
    }

    /********* set parameters *********/

    grampc_setparam_real_vector(grampc_, "x0", x0);
    grampc_setparam_real_vector(grampc_, "xdes", xdes);
    grampc_setparam_real_vector(grampc_, "u0", u0);
    grampc_setparam_real_vector(grampc_, "umin", umin);
    grampc_setparam_real_vector(grampc_, "umax", umax);

    grampc_setparam_real(grampc_, "Thor", Thor);
    grampc_setparam_real(grampc_, "t0", t);

    grampc_setopt_int(grampc_, "Nhor", Nhor);
    grampc_setopt_int(grampc_, "MaxGradIter", MaxGradIter);
    grampc_setopt_real_vector(grampc_, "ConstraintsAbsTol", ConstraintsAbsTol);

    grampc_setparam_real(grampc_, "dt", dt_);

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
  return yaw;  // in radians
}

double MPCCGrampcNode::computeCurvature(
    const geometry_msgs::msg::Point::ConstSharedPtr &ips_msg,
    double current_yaw)
{
  if (first_pose_) {
    last_pos_ = *ips_msg;
    last_yaw_ = current_yaw;
    first_pose_ = false;
    return 0.0;
  }

  double dx = ips_msg->x - last_pos_.x;
  double dy = ips_msg->y - last_pos_.y;
  double ds = std::sqrt(dx * dx + dy * dy);

  if (ds < 1e-5) return 0.0;  // avoid division by zero

  double dyaw = current_yaw - last_yaw_;

  // normalize yaw difference to [-pi, pi]
  while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
  while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

  double kappa = dyaw / ds;

  last_pos_ = *ips_msg;
  last_yaw_ = current_yaw;

  return kappa;
}

void MPCCGrampcNode::controlLoop(const geometry_msgs::msg::Point::ConstSharedPtr &ips_msg,
                                 const std_msgs::msg::Float32::ConstSharedPtr &speed_msg,
                                 const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{

    x_ = ips_msg->x;
    y_ = ips_msg->y;
    v_ = static_cast<double>(speed_msg->data);
    yaw_ = getYawFromImu(imu_msg);
    kappa_ = computeCurvature(ips_msg, yaw_);

    initializePathPosition();
    // Find current waypoint using arc length and get next waypoint
    size_t nextIdx = path_->findNextWaypointIdx(current_s_);
    RCLCPP_INFO(this->get_logger(), "next_idx=%zu",
                nextIdx);

    // Get target waypoint and reference states
    Eigen::Vector2d target_point = path_->getWaypoint(nextIdx);
    double target_heading = path_->getHeading(nextIdx);
    double target_curvature = path_->getCurvature(nextIdx);
    double ref_speed = 1.0; // Desired speed [m/s] //TODO:

    // Current state and target state
    std::vector<double> current_state = {x_, y_, yaw_, kappa_, v_};
    std::vector<double> target_state = {target_point.x(), target_point.y(), target_heading, target_curvature, ref_speed};

    // Set current state as initial condition
    grampc_setparam_real_vector(grampc_, "xdes", target_state.data());

    double steer_cmd = 0.0;
    double throttle_cmd = 0.0;

    grampc_run(grampc_);

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

        /* reference integration of the system via heun scheme since grampc->sol->xnext is only an interpolated value */
		ffct(rwsReferenceIntegration, t, grampc_->param->x0, grampc_->sol->unext, grampc_->sol->pnext, grampc_->userparam);
		for (auto i = 0; i < NX; i++)
		{
			grampc_->sol->xnext[i] = grampc_->param->x0[i] + dt_ * rwsReferenceIntegration[i];
		}
		ffct(rwsReferenceIntegration + NX, t + dt_, grampc_->sol->xnext, grampc_->sol->unext, grampc_->sol->pnext, grampc_->userparam);
		for (auto i = 0; i < NX; i++)
		{
			grampc_->sol->xnext[i] = grampc_->param->x0[i] + dt_ * (rwsReferenceIntegration[i] + rwsReferenceIntegration[i + NX]) / 2;
		}

		/* update state and time */
		t = t + dt_;
		grampc_setparam_real_vector(grampc_, "x0", grampc_->sol->xnext);

        double acceleration = grampc_->sol->unext[0]; // u[0] = acceleration [m/s^2]
        double kappa_dot = grampc_->sol->unext[1];    // u[1] = steering_rate (curvature rate) [1/s]

        // Integration: kappa = kappa_prev + kappa_dot * dt
        kappa_ += kappa_dot * dt_;

        throttle_cmd = acceleration;
        steer_cmd = kappa_ * L_;
    }


    // Publish steering command
    auto s_msg = std_msgs::msg::Float32();
    s_msg.data = static_cast<float>(steer_cmd);
    steering_pub_->publish(s_msg);

    // Publish throttle command
    auto t_msg = std_msgs::msg::Float32();
    t_msg.data = static_cast<float>(throttle_cmd);
    throttle_pub_->publish(t_msg);

    // Update previous commands for next iteration
    prev_steer_ = steer_cmd;
    prev_throttle_ = throttle_cmd;

    // Update time for GRAMPC integration
    t += dt_;
    grampc_setparam_real(grampc_, "t0", t);
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
