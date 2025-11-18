#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <vector>
#include <cstring>
#include <iostream>
#include <iomanip>

extern "C"
{
}

#include "control_grampc/mpcc_model.h"
#include "autodrive_msgs/msg/vehiclestate.hpp"
#include "sensor_msgs/msg/imu.hpp"

// If your probfct.h doesn't define NX/NU as plain integer macros visible to C++ here,
// you can override here (keep consistent with your model).
#ifndef NX
#define NX 7
#endif
#ifndef NU
#define NU 2
#endif

using std::placeholders::_1;

class MPCCModelTesterNode : public rclcpp::Node
{
public:
    MPCCModelTesterNode()
        : Node("mpcc_model_tester_node"),
          dt_(0.01),
          current_time_(0.0)
    {
        // Params
        this->declare_parameter<std::string>("parent_frame", "map");
        this->declare_parameter<std::string>("child_frame", "base_link");
        dt_ = 1 / 17.0;
        parent_frame_ = this->get_parameter("parent_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();

        // initialize state (typeRNum from GRAMPC/probfct)
        x_.assign(NX, static_cast<typeRNum>(0.0));
        x_[0] = static_cast<typeRNum>(-0.852928);
        x_[1] = static_cast<typeRNum>(5.125907);
        x_[2] = static_cast<typeRNum>(-0.046640);
        x_[3] = static_cast<typeRNum>(-0.816011);
        x_[4] = static_cast<typeRNum>(3.355342);
        x_[5] = static_cast<typeRNum>(4.856792);
        x_[6] = static_cast<typeRNum>(-3.870758);
        
        // inputs (u) default to zero
        u_.assign(NU, static_cast<typeRNum>(0.0));
        // p (parameters) - if not used, leave zero
        p_.assign(1, static_cast<typeRNum>(0.0));

        // Subscribers
        steer_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering_rate_command", 10,
            std::bind(&MPCCModelTesterNode::steerCallback, this, _1));
        throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle_command", 10,
            std::bind(&MPCCModelTesterNode::throttleCallback, this, _1));

        // Publisher for simulated vehicle state (Vehiclestate)
        state_pub_ = this->create_publisher<autodrive_msgs::msg::Vehiclestate>("/autodrive/f1tenth_1/vehicle_state", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer for integration loop
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&MPCCModelTesterNode::updateState, this));

        RCLCPP_INFO(this->get_logger(), "MPCC Model Tester Node started (dt=%.4f).", dt_);
    }

    ~MPCCModelTesterNode() override = default;

private:
    // Subscribers' callbacks
    void steerCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(u_mutex_);
        u_[0] = static_cast<typeRNum>(msg->data);
    }

    void throttleCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(u_mutex_);
        u_[1] = static_cast<typeRNum>(msg->data);
    }

    // Main integration loop: compute f = ffct(...); x += dt * f
    void updateState()
    {
        // Copy inputs under lock
        typeRNum u_local[NU];
        {
            std::lock_guard<std::mutex> lock(u_mutex_);
            for (int i = 0; i < NU; ++i)
                u_local[i] = u_[i];
        }

        // Prepare arrays as expected by ffct (ctypeRNum for t, typeRNum for others)
        ctypeRNum t = static_cast<ctypeRNum>(current_time_);
        typeRNum x_c[NX];
        typeRNum f[NX];
        std::memset(f, 0, sizeof(f));

        for (int i = 0; i < NX; ++i)
            x_c[i] = x_[i];

        typeRNum L_val = static_cast<typeRNum>(0.32); // typical wheelbase [m], adjust if needed
        double cornering_stiffness_front = 4.718;
        double cornering_stiffness_rear = 5.4562;
        double mass = 3.47;
        double inertia_z = 0.04712;
        double rear_axle_distance = 0.15875;
        double front_axle_distance = 0.17145;
        
        void *pSys[21];
        // initialize all entries to nullptr
        for (int i = 0; i < 21; ++i)
            pSys[i] = nullptr;
        pSys[0] = &L_val;
        pSys[1] = &cornering_stiffness_front;
        pSys[2] = &cornering_stiffness_rear;
        pSys[3] = &mass;
        pSys[4] = &inertia_z;
        pSys[5] = &front_axle_distance;
        pSys[6] = &rear_axle_distance;
        // other parameters can be set similarly if needed

        ffct(f, t, x_c, u_local, p_.data(), nullptr, (typeUSERPARAM *)pSys);

        // Integrate forward Euler: x = x + dt * f
        for (int i = 0; i < NX; ++i)
        {
            x_[i] = static_cast<typeRNum>(x_c[i] + static_cast<typeRNum>(dt_) * f[i]);
        }

        current_time_ += dt_;

        // Publish pose and TF
        publishStateAndTF();
    }

    void publishStateAndTF()
    {
        // state layout: [x, y, psi, delta, v]
        double px = static_cast<double>(x_[0]);
        double py = static_cast<double>(x_[1]);
        double psi = static_cast<double>(x_[2]);
        double delta = static_cast<double>(x_[3]);
        double v = static_cast<double>(x_[4]);
        double v_y = static_cast<double>(x_[5]);
        double psi_rate = static_cast<double>(x_[6]);

        // Build Vehiclestate message
        autodrive_msgs::msg::Vehiclestate vs_msg;
        vs_msg.header.stamp = this->now();
        vs_msg.header.frame_id = parent_frame_;
        vs_msg.position.x = px;
        vs_msg.position.y = py;
        vs_msg.position.z = 0.0;
        // minimal IMU: leave default-initialized (zeros) unless you have sensor data
        vs_msg.imu = sensor_msgs::msg::Imu();
        vs_msg.imu.angular_velocity.z = static_cast<float>(psi_rate);
        vs_msg.speed = static_cast<float>(v);
        vs_msg.steering_angle = static_cast<float>(delta);

        // publish TF (map -> base_link)
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = vs_msg.header.stamp;
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = child_frame_;

        tf_msg.transform.translation.x = px;
        tf_msg.transform.translation.y = py;
        tf_msg.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, psi);
        tf_msg.transform.rotation = tf2::toMsg(q);

        // fill imu orientation so controller can read psi from msg->imu
        vs_msg.imu.orientation = tf2::toMsg(q);

        RCLCPP_INFO(this->get_logger(), "Publishing Vehiclestate: x=%.2f, y=%.2f, psi=%.2f, v=%.2f, delta=%.2f", px, py, psi, v, delta);
        state_pub_->publish(vs_msg);

        tf_broadcaster_->sendTransform(tf_msg);
    }

    // ROS handles
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;
    rclcpp::Publisher<autodrive_msgs::msg::Vehiclestate>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // model state & inputs (typeRNum from grampc/probfct)
    std::vector<typeRNum> x_; // size NX
    std::vector<typeRNum> u_; // size NU
    std::vector<typeRNum> p_; // parameters (unused here)

    std::mutex u_mutex_;

    double dt_;
    double current_time_;

    // TF frames
    std::string parent_frame_;
    std::string child_frame_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCCModelTesterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
