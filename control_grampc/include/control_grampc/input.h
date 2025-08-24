#ifndef CONTROL_GRAMPC_INPUT_H
#define CONTROL_GRAMPC_INPUT_H

#include <Eigen/Dense>

namespace control_grampc {

class Input {
public:
    Input();
    Input(double velocity, double steering_angle);
    virtual ~Input() = default;

    // Convert input to Eigen vector
    Eigen::VectorXd toVector() const;
    
    // Setters
    void setVelocity(double v) { velocity_ = v; }
    void setSteeringAngle(double delta) { steering_angle_ = delta; }
    
    // Getters
    double velocity() const { return velocity_; }
    double steeringAngle() const { return steering_angle_; }
    
    int size() const { return size_; }

private:
    double velocity_;
    double steering_angle_;
    static const int size_ = 2;  // velocity, steering_angle
};

} // namespace control_grampc

#endif // CONTROL_GRAMPC_INPUT_H
