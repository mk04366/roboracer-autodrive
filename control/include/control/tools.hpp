#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <deque>
#include <cmath>
#include <optional>
#include <algorithm>
#include <numeric>
#include "autodrive_msgs/msg/vehiclestate.hpp"

/**
 * PID Controller for throttle control
 * Converts desired velocity to normalized throttle command
 */
class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double output_min, double output_max)
        : kp_(kp), ki_(ki), kd_(kd),
          output_min_(output_min), output_max_(output_max),
          integral_(0.0), previous_error_(0.0), first_run_(true) {}

    double compute(double setpoint, double measured_value, double dt)
    {
        // Calculate error
        double error = setpoint - measured_value;

        // Proportional term
        double p_term = kp_ * error;

        // Integral term with anti-windup
        integral_ += error * dt;
        // Clamp integral to prevent windup
        double max_integral = 1.0 / ki_; // Limit integral contribution
        integral_ = std::clamp(integral_, -max_integral, max_integral);
        double i_term = ki_ * integral_;

        // Derivative term
        double d_term = 0.0;
        if (!first_run_)
        {
            double derivative = (error - previous_error_) / dt;
            d_term = kd_ * derivative;
        }
        else
        {
            first_run_ = false;
        }

        // Compute output
        double output = p_term + i_term + d_term;

        // Clamp output to valid range
        output = std::clamp(output, output_min_, output_max_);

        // Store error for next iteration
        previous_error_ = error;

        return output;
    }

    void reset()
    {
        integral_ = 0.0;
        previous_error_ = 0.0;
        first_run_ = true;
    }

    void setGains(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double integral_;
    double previous_error_;
    bool first_run_;
};

/**
 * Extended Kalman Filter for IMU-based position estimation
 * State vector: [px, py, vx, vy, ax, ay] (position, velocity, acceleration bias)
 * This helps compensate for IMU drift and bias
 */
class IMUKalmanFilter
{
public:
    IMUKalmanFilter()
    {
        // Initialize state vector: [px, py, vx, vy, ax_bias, ay_bias]
        state_ = Eigen::VectorXd::Zero(6);

        // Initialize covariance matrix (uncertainty in our estimates)
        P_ = Eigen::MatrixXd::Identity(6, 6);
        P_(0, 0) = 0.001;
        P_(1, 1) = 0.001; // Position uncertainty
        P_(2, 2) = 0.01;
        P_(3, 3) = 0.01; // Velocity uncertainty
        P_(4, 4) = 0.1;
        P_(5, 5) = 0.1; // Acceleration bias uncertainty

        // Process noise covariance (how much we trust the model)
        Q_ = Eigen::MatrixXd::Identity(6, 6);
        Q_(0, 0) = 0.001;
        Q_(1, 1) = 0.001; // Position process noise
        Q_(2, 2) = 0.01;
        Q_(3, 3) = 0.01; // Velocity process noise
        Q_(4, 4) = 0.001;
        Q_(5, 5) = 0.001; // Bias process noise (small - biases change slowly)

        // Measurement noise covariance for IMU accelerations
        R_imu_ = Eigen::MatrixXd::Identity(2, 2);
        R_imu_(0, 0) = 0.05;
        R_imu_(1, 1) = 0.05; // IMU acceleration noise

        // Measurement noise covariance for speed measurements
        R_speed_ = Eigen::MatrixXd::Identity(1, 1);
        R_speed_(0, 0) = 0.01; // Speed sensor noise

        dt_ = 0.01; // 100Hz update rate
        initialized_ = false;
    }

    void initialize(double x, double y)
    {
        state_(0) = x;
        state_(1) = y;
        state_(2) = 0.0; // Initial velocity x
        state_(3) = 0.0; // Initial velocity y
        state_(4) = 0.0; // Initial acceleration bias x
        state_(5) = 0.0; // Initial acceleration bias y
        initialized_ = true;
    }

    /**
     * Prediction step: Use kinematic model to predict next state
     * Position update: p = p + v*dt + 0.5*a*dt^2
     * Velocity update: v = v + a*dt
     * Bias remains constant in prediction
     */
    void predict(double ax_world, double ay_world)
    {
        if (!initialized_)
            return;

        // State transition matrix
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 2) = dt_; // px depends on vx
        F(1, 3) = dt_; // py depends on vy

        // Control input matrix (for accelerations)
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 2);
        B(2, 0) = dt_;             // vx affected by ax
        B(3, 1) = dt_;             // vy affected by ay
        B(0, 0) = 0.5 * dt_ * dt_; // px affected by ax
        B(1, 1) = 0.5 * dt_ * dt_; // py affected by ay

        // Control input (corrected accelerations)
        Eigen::Vector2d u;
        u << ax_world - state_(4), ay_world - state_(5); // Remove bias

        // Predict state
        state_ = F * state_ + B * u;

        // Predict covariance
        P_ = F * P_ * F.transpose() + Q_;
    }

    /**
     * Update step with IMU acceleration measurements
     * Corrects the predicted state based on acceleration observations
     */
    void updateWithIMU(double ax_measured, double ay_measured)
    {
        if (!initialized_)
            return;

        // Measurement matrix (we observe accelerations)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
        H(0, 4) = -1.0; // ax_observed = ax_true - ax_bias
        H(1, 5) = -1.0; // ay_observed = ay_true - ay_bias

        // Innovation (measurement residual)
        Eigen::Vector2d z;
        z << ax_measured, ay_measured;
        Eigen::Vector2d z_pred;
        z_pred << -state_(4), -state_(5); // Predicted measurement
        Eigen::Vector2d y = z - z_pred;

        // Innovation covariance
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_imu_;

        // Kalman gain
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // Update state
        state_ = state_ + K * y;

        // Update covariance
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    }

    /**
     * Update step with speed measurement from encoder
     * Uses speed to correct velocity estimates
     */
    void updateWithSpeed(double speed, double yaw)
    {
        if (!initialized_)
            return;

        // Current velocity magnitude from state
        double v_mag_est = std::sqrt(state_(2) * state_(2) + state_(3) * state_(3));

        if (v_mag_est > 0.01)
        { // Only update if we have significant velocity
            // Measurement matrix for speed (1x6)
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
            H(0, 2) = state_(2) / v_mag_est; // dvmag/dvx
            H(0, 3) = state_(3) / v_mag_est; // dvmag/dvy

            // Innovation
            Eigen::VectorXd y(1);
            y(0) = speed - v_mag_est;

            // Innovation covariance
            Eigen::MatrixXd S = H * P_ * H.transpose() + R_speed_;

            // Kalman gain
            Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

            // Update state
            state_ = state_ + K * y;

            // Update covariance
            P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
        }
        else
        {
            // If estimated velocity is near zero, directly set velocity based on speed and heading
            state_(2) = speed * std::cos(yaw);
            state_(3) = speed * std::sin(yaw);
        }
    }

    Eigen::Vector2d getPosition() const
    {
        return Eigen::Vector2d(state_(0), state_(1));
    }

    Eigen::Vector2d getVelocity() const
    {
        return Eigen::Vector2d(state_(2), state_(3));
    }

    Eigen::Vector2d getAccelerationBias() const
    {
        return Eigen::Vector2d(state_(4), state_(5));
    }

private:
    Eigen::VectorXd state_;   // State vector
    Eigen::MatrixXd P_;       // State covariance
    Eigen::MatrixXd Q_;       // Process noise
    Eigen::MatrixXd R_imu_;   // IMU measurement noise
    Eigen::MatrixXd R_speed_; // Speed measurement noise
    double dt_;
    bool initialized_;
};
