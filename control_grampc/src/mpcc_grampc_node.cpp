#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "grampc.h"
#include "control_grampc/mpcc_model.h"

#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

// ---------- Reference context and interpolation ----------
struct ReferenceContext
{
    std::vector<double> xs, ys, thetas, kappas, ss;
};

// linear interpolation along arc length s
static void ref_interp_bridge(double s, double *xr, double *yr, double *thr,
                              double *kappa, void *ctx)
{
    auto *ref = reinterpret_cast<ReferenceContext *>(ctx);
    if (ref->ss.empty())
    {
        *xr = 0;
        *yr = 0;
        *thr = 0;
        *kappa = 0;
        return;
    }
    if (s <= ref->ss.front())
    {
        *xr = ref->xs.front();
        *yr = ref->ys.front();
        *thr = ref->thetas.front();
        *kappa = ref->kappas.front();
        return;
    }
    if (s >= ref->ss.back())
    {
        *xr = ref->xs.back();
        *yr = ref->ys.back();
        *thr = ref->thetas.back();
        *kappa = ref->kappas.back();
        return;
    }

    // find segment
    auto it = std::lower_bound(ref->ss.begin(), ref->ss.end(), s);
    size_t idx = std::max<size_t>(1, it - ref->ss.begin()) - 1;
    double s0 = ref->ss[idx], s1 = ref->ss[idx + 1];
    double tau = (s - s0) / (s1 - s0);

    *xr = (1 - tau) * ref->xs[idx] + tau * ref->xs[idx + 1];
    *yr = (1 - tau) * ref->ys[idx] + tau * ref->ys[idx + 1];
    *thr = (1 - tau) * ref->thetas[idx] + tau * ref->thetas[idx + 1];
    *kappa = (1 - tau) * ref->kappas[idx] + tau * ref->kappas[idx + 1];
}

// ---------- Node ----------
class MPCCGrampcNode : public rclcpp::Node
{
public:
    MPCCGrampcNode() : Node("mpcc_controller")
    {
        // Publishers / Subscribers
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/autodrive/f1tenth_1/ips", 1,
            std::bind(&MPCCGrampcNode::pose_cb, this, std::placeholders::_1));
        sub_speed_ = create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/speed", 1,
            std::bind(&MPCCGrampcNode::speed_cb, this, std::placeholders::_1));
        pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/autodrive/f1tenth_1/drive", 1);

        // Fill reference track (dummy straight line for now)
        for (int i = 0; i < 200; i++)
        {
            double s = 0.1 * i;
            ref_ctx_.ss.push_back(s);
            ref_ctx_.xs.push_back(s);
            ref_ctx_.ys.push_back(0.0);
            ref_ctx_.thetas.push_back(0.0);
            ref_ctx_.kappas.push_back(0.0);
        }

        // Context
        ctx_.L = 0.33;
        ctx_.delta_max = 0.4;
        ctx_.a_min = 0.0; // forbid reverse
        ctx_.a_max = 3.0;
        ctx_.v_max = 3.0;
        ctx_.a_lat_max = 4.0;
        ctx_.w_c = 5.0;
        ctx_.w_l = 0.1;
        ctx_.w_v = 1.0;
        ctx_.w_u = 0.1;
        ctx_.w_du = 0.1;
        ctx_.w_term = 10.0;
        ctx_.kappa_gain = 0.5;
        ctx_.ref_interp = ref_interp_bridge;
        ctx_.ref_ctx = &ref_ctx_;
        ctx_.u_prev[0] = ctx_.u_prev[1] = 0.0;

        // Initial state [x,y,theta,v,s]
        std::vector<double> x0 = {0, 0, 0, 0, 0};

        // GRAMPC setup
        opt_ = grampc_init();
        grampc_setparam_double(opt_, "Thor", 2.0); // horizon [s]
        grampc_setparam_int(opt_, "Nhor", 20);
        grampc_setparam_double(opt_, "dt", 0.05); // sampling time
        grampc_setparam_double(opt_, "MaxGradIter", 20);
        grampc_setparam_double(opt_, "TolCost", 1e-3);

        // Dimensions: nx=5, nu=2
        grampc_alloc_structure(opt_, 5, 2, 0, 0, 0, 0);
        grampc_setfct_ptrs(opt_, mpcc_dynamics,
                           mpcc_stage_cost,
                           mpcc_terminal_cost);

        // Set bounds
        double umin[2] = {-ctx_.delta_max, ctx_.a_min};
        double umax[2] = {ctx_.delta_max, ctx_.a_max};
        grampc_setparam_doublearray(opt_, "umin", umin, 2);
        grampc_setparam_doublearray(opt_, "umax", umax, 2);

        // Set initial state
        grampc_setstate(opt_, x0.data());

        // Attach user context
        grampc_set_userparam(opt_, &ctx_);

        // Timer for control loop
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MPCCGrampcNode::control_loop, this));
    }

private:
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_ = *msg;
        pose_received_ = true;
    }
    void speed_cb(const std_msgs::msg::Float32::SharedPtr msg)
    {
        speed_ = msg->data;
    }

    void control_loop()
    {
        if (!pose_received_)
            return;

        // current state
        double x[5];
        x[0] = pose_.pose.position.x;
        x[1] = pose_.pose.position.y;
        double yaw = 2 * atan2(pose_.pose.orientation.z,
                               pose_.pose.orientation.w);
        x[2] = yaw;
        x[3] = speed_;
        x[4] += speed_ * 0.05; // progress (naive)

        grampc_setstate(opt_, x);

        // Run solver
        grampc_run(opt_);

        // get u0
        double u[2];
        grampc_getinput(opt_, u);
        ctx_.u_prev[0] = u[0];
        ctx_.u_prev[1] = u[1];

        // publish
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = now();
        cmd.drive.steering_angle = u[0];
        cmd.drive.acceleration = u[1];
        pub_drive_->publish(cmd);
    }

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
    rclcpp::TimerBase::SharedPtr timer_;

    // state
    geometry_msgs::msg::PoseStamped pose_;
    bool pose_received_ = false;
    double speed_ = 0.0;

    // GRAMPC
    typeGRAMPC *opt_;
    mpcc_ctx_t ctx_;
    ReferenceContext ref_ctx_;
    double last_s_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCCGrampcNode>());
    rclcpp::shutdown();
    return 0;
}
