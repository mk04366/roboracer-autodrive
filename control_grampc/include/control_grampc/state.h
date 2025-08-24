#ifndef CONTROL_GRAMPC_STATE_H
#define CONTROL_GRAMPC_STATE_H

#include <Eigen/Dense>

namespace control_grampc {

class State {
public:
    State();
    State(double x, double y, double theta);
    State(double x, double y, double theta, double v);
    virtual ~State() = default;

    // Convert state to Eigen vector
    Eigen::VectorXd toVector() const;
    
    // Setters
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }
    void setTheta(double theta) { theta_ = theta; }
    void setV(double v) { v_ = v; }
    
    // Getters
    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }
    double v() const { return v_; }
    
    int size() const { return size_; }
    
    std::pair<double, double> position() const { return {x_, y_}; }

private:
    double x_;
    double y_;
    double theta_;
    double v_;
    static const int size_ = 4;  // x, y, theta, v
};

} // namespace control_grampc

#endif // CONTROL_GRAMPC_STATE_H
