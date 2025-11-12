#include "control_grampc/mpcc_grampc_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cstdint>

using std::placeholders::_1;

MPCCGrampcNode::MPCCGrampcNode()
    : Node("mpcc_grampc_node"), x_(0.0), y_(0.0), psi_(0.0), v_(0.0), steering_(0.0), prev_steer_(0.0), prev_throttle_(0.0)
{
  // Load path from CSV file
  std::string csv_file = this->declare_parameter<std::string>("path_csv",
                                                              "/home/ammar/ros2_ws/src/global-planning/outputs/map5/"
                                                              "traj_race_cl_low_sampled.csv");
  path_ = std::make_shared<mpcc::Path>(mpcc::load_path_from_csv(csv_file));

  if (path_ && path_->getTotalLength() > 1e-3)
  {
    RCLCPP_INFO(this->get_logger(), "Path loaded successfully: %s, length: %ld", csv_file.c_str(),
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

  path_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MPCCGrampcNode::publishPath, this));
  // ---------------------------------------- //

  initGrampcParams();
}

void MPCCGrampcNode::publishPath()
{
  if (!path_ || path_->getTotalLength() == 0)
    return;

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map";

  path_msg.poses.reserve(path_->getTotalLength());
  for (double i = 0; i < path_->getTotalLength(); ++i)
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
  RCLCPP_DEBUG(this->get_logger(), "Published path with %zu waypoints", path_->getTotalLength());
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
  // allocate vectors from Path
  double N = path_->getTotalLength();
  t_ref_.reserve(N);
  x_ref_.reserve(N);
  y_ref_.reserve(N);
  psi_ref_.reserve(N);
  delta_ref_.reserve(N);
  v_ref_.reserve(N);

  for (double i = 0; i < N; ++i)
  {
    t_ref_.push_back(path_->getTimeFromIndex(i));
    Eigen::Vector2d xy = path_->getWaypoint(i);
    x_ref_.push_back(xy.x());
    y_ref_.push_back(xy.y());
    psi_ref_.push_back(path_->getHeading(i));
    delta_ref_.push_back(path_->getSteering(i));
    v_ref_.push_back(path_->getVelocity(i));
  }
  L_ = 0.32;

  /********* Parameter definition *********/
  /* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints
   * for the states and inputs */
  ctypeRNum x0[NX] = {0.0, 0.0, 0.0, 0.0, 0.0};
  ctypeRNum xdes[NX] = {0.0, 0.0, 0.0, 0.0, 0.0};
  /* Initial values, setpoints and limits of the inputs */
  ctypeRNum u0[NU] = {0.0, 0.0};
  ctypeRNum udes[NU] = {0.0, 0.0};
  ctypeRNum umin[NU] = {-M_PI / 6, 0.0};
  ctypeRNum umax[NU] = {M_PI / 6, 1.0};
  Thor_ = 1.0;      /* Prediction horizon */
  dt_ = 1.0 / 17.0; /* Sampling time */

  static double pvals[21];

  // scalar parameters
  pvals[0] = L_;
  pvals[1] = 100.0;
  pvals[2] = 100.0;
  pvals[3] = 100.0;
  pvals[4] = 100.0;
  pvals[5] = 0.0;
  pvals[6] = 100.0;
  pvals[7] = 100.0;
  pvals[8] = 100.0;
  pvals[9] = 100.0;
  pvals[10] = 0.0;
  pvals[11] = 0.01;
  pvals[12] = 0.01;

  // store pointer bitpatterns (uintptr_t -> double)
  pvals[13] = static_cast<double>(reinterpret_cast<uintptr_t>(t_ref_.data()));
  pvals[14] = static_cast<double>(reinterpret_cast<uintptr_t>(x_ref_.data()));
  pvals[15] = static_cast<double>(reinterpret_cast<uintptr_t>(y_ref_.data()));
  pvals[16] = static_cast<double>(reinterpret_cast<uintptr_t>(psi_ref_.data()));
  pvals[17] = static_cast<double>(reinterpret_cast<uintptr_t>(delta_ref_.data()));
  pvals[18] = static_cast<double>(reinterpret_cast<uintptr_t>(v_ref_.data()));

  // N and current_time_
  pvals[19] = static_cast<double>(N);
  pvals[20] = static_cast<double>(current_time_);
  dt_ = 1.0 / 17.0;   /* Sampling time */
  ctypeInt Nhor = 15; /* Number of steps for the system integration */
  typeRNum t0 = 0.0;  /* time at the current sampling step */

  /********* Option definition *********/
  ctypeInt MaxGradIter = 5;
  ctypeRNum ConstraintsAbsTol[1] = {1e-2};

  // Build pointer array pSys which GRAMPC model will interpret as userparam (array of pointers)
  static void *pSys[21] = {nullptr};
  // Point scalar slots to pvals entries (indices 0..12 and 19..20)
  for (size_t i = 0; i <= 12; ++i)
  {
    pSys[i] = &pvals[i];
  }
  pSys[19] = &pvals[19];
  pSys[20] = &pvals[20];

  // Fill pointer slots with the reference array data pointers (indices 13..18)
  pSys[13] = const_cast<double *>(t_ref_.data());
  pSys[14] = const_cast<double *>(x_ref_.data());
  pSys[15] = const_cast<double *>(y_ref_.data());
  pSys[16] = const_cast<double *>(psi_ref_.data());
  pSys[17] = const_cast<double *>(delta_ref_.data());
  pSys[18] = const_cast<double *>(v_ref_.data());

  // Cast the void* array to the expected USERPARAM pointer type for GRAMPC
  typeUSERPARAM *userparam = reinterpret_cast<typeUSERPARAM *>(pSys);

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

  grampc_setparam_real_vector(grampc_, "udes", udes);
  grampc_setparam_real_vector(grampc_, "u0", u0);
  grampc_setparam_real_vector(grampc_, "umin", umin);
  grampc_setparam_real_vector(grampc_, "umax", umax);

  grampc_setparam_real(grampc_, "Thor", Thor_);
  grampc_setparam_real(grampc_, "t0", t0);
  grampc_setparam_real(grampc_, "dt", dt_);

  grampc_setopt_int(grampc_, "Nhor", Nhor);
  grampc_setopt_int(grampc_, "MaxGradIter", MaxGradIter);
  grampc_setopt_real_vector(grampc_, "ConstraintsAbsTol", ConstraintsAbsTol);
};

void MPCCGrampcNode::initializePathPosition()
{
  // Initialize with closest waypoint on first run
  Eigen::Vector2d vehicle_pos(x_, y_);
  current_path_idx_ = 0;
  double min_dist = (vehicle_pos - path_->getWaypoint(0)).norm();
  for (double i = 1; i < path_->getTotalLength(); ++i)
  {
    double dist = (vehicle_pos - path_->getWaypoint(i)).norm();
    if (dist < min_dist)
    {
      min_dist = dist;
      current_path_idx_ = i;
    }
  }
  current_time_ = path_->getTimeFromIndex(current_path_idx_);
  grampc_setparam_real(grampc_, "t0", current_time_);
  auto **userparamUpdate = reinterpret_cast<void **>(grampc_->userparam);
  userparamUpdate[20] = &current_time_;
  grampc_->userparam = reinterpret_cast<typeUSERPARAM *>(userparamUpdate);

  RCLCPP_INFO(this->get_logger(), "Initialized path position at index %.0f, time %.2f s", static_cast<double>(current_path_idx_),
              current_time_);
}

double MPCCGrampcNode::getYawFromImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
  tf2::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw; // in radians
}

void MPCCGrampcNode::vehicleCallback(const autodrive_msgs::msg::Vehiclestate::SharedPtr msg)
{
  // Extract IPS position
  x_ = msg->position.x;
  y_ = msg->position.y;

  // Extract speed
  v_ = static_cast<double>(msg->speed);

  // Extract steering angle
  steering_ = static_cast<double>(msg->steering_angle);

  // Extract yaw/psi from IMU
  auto imu_ptr = std::make_shared<sensor_msgs::msg::Imu>(msg->imu);
  psi_ = getYawFromImu(imu_ptr);

  // Compute curvature
  auto pos_ptr = std::make_shared<geometry_msgs::msg::Point>(msg->position);

  // Run control loop
  controlLoop();
}

void MPCCGrampcNode::controlLoop()
{
  initializePathPosition();
  // Find current waypoint using arc length and get next waypoint
  double nextIdx = path_->findNextWaypointIdx(current_time_, Thor_);

  // Get target waypoint and reference states
  Eigen::Vector2d target_point = path_->getWaypoint(nextIdx);
  double target_heading = path_->getHeading(nextIdx);
  double target_steering = path_->getSteering(nextIdx);
  double target_speed = path_->getVelocity(nextIdx);

  // publish target pose for visualization
  publishTarget(target_point, target_heading);

  // Current state and target state
  std::vector<double> current_state = {x_, y_, psi_, steering_, v_};
  std::vector<double> target_state = {target_point.x(), target_point.y(), target_heading, target_steering, target_speed};

  // Set current state as initial & desired condition
  grampc_setparam_real_vector(grampc_, "x0", current_state.data());
  grampc_setparam_real_vector(grampc_, "xdes", target_state.data());

  double steer_cmd = 0.0;
  double throttle_cmd = 0.0;

  // print target state for debugging
  // RCLCPP_INFO(this->get_logger(), "Target State: x=%.2f, y=%.2f, psi=%.2f, v=%.2f, steering=%.2f", target_state[0],
  //             target_state[1], target_state[2], target_state[3], target_state[4]);
  // RCLCPP_INFO(this->get_logger(), "Current State: x=%.2f, y=%.2f, psi=%.2f, v=%.4f, steering=%.2f", current_state[0],
  //             current_state[1], current_state[2], current_state[3], current_state[4]);

  grampc_run(grampc_);

  // Check for any GRAMPC solver issues - treat all status > 0 as errors
  if (grampc_->sol->status > 0)
  {
    RCLCPP_ERROR(this->get_logger(), "GRAMPC solver error with status: %d", grampc_->sol->status);

    // Use previous commands when solver fails
    steer_cmd = prev_steer_;
    throttle_cmd = prev_throttle_;
  }
  else
  {
    /* update state and time */
    throttle_cmd = grampc_->sol->unext[1]; // scale down acceleration command
    double steering_rate_cmd = grampc_->sol->unext[0];
    steer_cmd = grampc_->sol->xnext[3] + steering_rate_cmd * dt_;

    // RCLCPP_INFO(this->get_logger(), "Computed Commands: Throttle=%.4f, Steering=%.4f", throttle_cmd,
    //             steer_cmd);

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
