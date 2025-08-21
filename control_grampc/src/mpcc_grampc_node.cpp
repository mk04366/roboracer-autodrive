#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "control_grampc/path_utils.hpp"
#include "control_grampc/mpcc_model.h"
extern "C" {
#include "grampc.h"
}
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class MPCCGrampcNode : public rclcpp::Node
{
public:
    MPCCGrampcNode()
        : Node("mpcc_grampc_node"), x_(0.0), y_(0.0), yaw_(0.0), v_(0.0), s_(0.0),
          prev_steer_(0.0), prev_throttle_(0.0)
    {
        // Load path from CSV
        std::string csv_file = this->declare_parameter<std::string>("path_csv", "path.csv");
        path_ = std::make_shared<mpcc::Path>(mpcc::load_path_from_csv(csv_file));

        // ROS interfaces
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/autodrive/f1tenth_1/odom", 10, std::bind(&MPCCGrampcNode::odomCallback, this, _1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MPCCGrampcNode::controlLoop, this));

        // GRAMPC setup (v2.2 API) with user context
        ctx_.L = 0.33;
        ctx_.delta_max = 0.8;      // ~45.8 deg steering (increase authority)
        ctx_.a_min = 0.0;          // avoid reverse accel
        ctx_.a_max = 2.5;          // limit longitudinal accel
        ctx_.v_max = 2.5;          // cap speed
        ctx_.a_lat_max = 2.0;      // lateral accel cap
        ctx_.w_xy = 80.0;          // position weight
        ctx_.w_yaw = 8.0;          // heading weight
        ctx_.w_v = 2.0;            // speed tracking weight
        ctx_.w_u = 0.15;           // input magnitude
        ctx_.w_du = 1.0;           // input rate
        ctx_.w_term = 50.0;        // terminal weight
        ctx_.u_prev[0] = 0.0; ctx_.u_prev[1] = 0.0;

        grampc_ = nullptr;
        grampc_init(&grampc_, (void*)&ctx_);

        grampc_->param->Nx = 4;  // [x, y, yaw, v]
        grampc_->param->Nu = 2;  // [steer, throttle]

        // Horizon and step sizes
        grampc_->param->Thor = 0.6;   // shorter horizon -> faster solve
        grampc_->param->dt = 0.05;

        // Initial guess and fast options
        grampc_->opt->Nhor = 8;        // fewer shooting intervals
        grampc_->opt->MaxGradIter = 4; // fewer gradient iterations per tick
        grampc_->opt->Integrator = 0;  // Euler (fastest)
        grampc_->opt->LineSearchType = 0; // explicit line search
        grampc_->opt->TimeDiscretization = 0; // equidistant

        // Input bounds via API
        double umin[2] = {-ctx_.delta_max, ctx_.a_min};
        double umax[2] = { ctx_.delta_max, 0.30}; // slightly lower throttle cap
        grampc_setparam_real_vector(grampc_, "umin", umin);
        grampc_setparam_real_vector(grampc_, "umax", umax);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        double qw = msg->pose.pose.orientation.w;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));

        v_ = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) +
                       std::pow(msg->twist.twist.linear.y, 2));
    }

    void controlLoop()
    {
        if (!path_ || path_->total_length() < 1e-3)
            return;

        // Find target point along path
        double lookahead = 2.0;
        double s_target = std::min(s_ + lookahead, path_->total_length());

        Eigen::Vector2d xy = path_->interpolate(s_target);
        double yaw_ref = path_->heading(s_target);
        double kappa = path_->curvature(s_target);

        // Prepare state vector
        std::vector<double> x0 = {x_, y_, yaw_, v_};
        std::vector<double> u0 = {0.0, 0.0};

        grampc_setparam_real_vector(grampc_, "x0", x0.data());
        grampc_setparam_real_vector(grampc_, "u0", u0.data());

        // Reference (goal point, yaw, and curvature-based speed)
        double v_curve = std::sqrt(std::max(0.0, ctx_.a_lat_max / std::max(std::abs(kappa), 1e-3)));
        double v_ref = std::clamp(v_curve * 0.9, 0.6, ctx_.v_max);
        std::vector<double> xref = {xy.x(), xy.y(), yaw_ref, v_ref};
        grampc_setparam_real_vector(grampc_, "xdes", xref.data());

        // Run MPC step
        grampc_run(grampc_);

        double steer_cmd = grampc_->sol->unext[0];
        double throttle_cmd = grampc_->sol->unext[1];

        // Post-saturation for safety and requested behavior
        steer_cmd = std::clamp(steer_cmd, -0.6, 0.6);
        throttle_cmd = std::clamp(throttle_cmd, 0.0, 0.4);
        // Slightly reduce throttle overall
        throttle_cmd *= 0.85;

        // Update previous control for rate penalty
        ctx_.u_prev[0] = prev_steer_ = steer_cmd;
        ctx_.u_prev[1] = prev_throttle_ = throttle_cmd;

        // Publish commands
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = steer_cmd;
        drive_msg.drive.speed = throttle_cmd;
        drive_pub_->publish(drive_msg);

        auto s_msg = std_msgs::msg::Float32();
        s_msg.data = static_cast<float>(steer_cmd);
        steering_pub_->publish(s_msg);

        auto t_msg = std_msgs::msg::Float32();
        t_msg.data = static_cast<float>(throttle_cmd);
        throttle_pub_->publish(t_msg);

        // Update arc length (progress along path)
        s_ = std::min(s_target, path_->total_length());
    }

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    double x_, y_, yaw_, v_, s_;

    // Path
    std::shared_ptr<mpcc::Path> path_;

    // GRAMPC
    typeGRAMPC *grampc_;

    // User context and previous control for rate penalties
    mpcc_ctx_t ctx_;
    double prev_steer_;
    double prev_throttle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCCGrampcNode>());
    rclcpp::shutdown();
    return 0;
}
