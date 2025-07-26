#pragma once
#include "control/stanley_controller_node.hpp"

class VelocityPID
{
public:
    VelocityPID(double kp = 1.0, double ki = 0.0, double kd = 0.0)
        : kp_(kp), ki_(ki), kd_(kd),
          integral_(0.0), prev_error_(0.0), prev_time_set_(false)
    {}

    double compute(double target, double current, const rclcpp::Time& now)
    {
        double error = target - current;
        double dt = 0.01;  // default to 10 ms if first call

        if (prev_time_set_)
        {
            dt = (now - prev_time_).seconds();
            if (dt <= 0.0 || dt > 1.0) {
                dt = 0.01; // fallback to safe delta
            }
        }

        // Save time for next iteration
        prev_time_ = now;
        prev_time_set_ = true;

        // Integral and derivative terms
        integral_ += error * dt;
        double derivative = (dt > 0) ? (error - prev_error_) / dt : 0.0;
        prev_error_ = error;

        // PID output
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_time_set_ = false;
    }

private:
    double kp_, ki_, kd_;
    double integral_;
    double prev_error_;
    rclcpp::Time prev_time_;
    bool prev_time_set_;
};
