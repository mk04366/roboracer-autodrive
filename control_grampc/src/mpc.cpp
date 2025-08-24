#include "control_grampc/mpc.h"
#include "control_grampc/mpcc_model.h"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>

namespace control_grampc {

MPC::MPC() 
    : grampc_(nullptr)
    , ctx_(nullptr)
    , horizon_(20)
    , state_size_(4)
    , input_size_(2)
    , dt_(0.05)
    , solver_initialized_(false)
    , has_solution_(false) {
    
    // Initialize state and input sizes
    state_size_ = 4;  // x, y, theta, v
    input_size_ = 2;  // velocity, steering
    
    // Default cost matrices
    Eigen::DiagonalMatrix<double, 4> Q;
    Eigen::DiagonalMatrix<double, 2> R;
    Q.diagonal() << 10.0, 10.0, 1.0, 1.0;  // x, y, theta, v
    R.diagonal() << 0.1, 1.0;               // velocity, steering
    cost_ = Cost(Q, R);
}

MPC::MPC(rclcpp::Node::SharedPtr node) 
    : node_(node)
    , grampc_(nullptr)
    , ctx_(nullptr)
    , solver_initialized_(false)
    , has_solution_(false) {
    
    if (!node_) {
        // Fall back to default constructor behavior
        MPC();
        return;
    }
    
    // Get parameters from node
    horizon_ = node_->declare_parameter<int>("mpc.horizon", 20);
    dt_ = node_->declare_parameter<double>("mpc.dt", 0.05);
    
    // Set up cost matrices
    double q0 = node_->declare_parameter<double>("mpc.q0", 10.0);  // x position
    double q1 = node_->declare_parameter<double>("mpc.q1", 10.0);  // y position  
    double q2 = node_->declare_parameter<double>("mpc.q2", 1.0);   // orientation
    double q3 = node_->declare_parameter<double>("mpc.q3", 1.0);   // velocity
    
    double r0 = node_->declare_parameter<double>("mpc.r0", 0.1);   // velocity input
    double r1 = node_->declare_parameter<double>("mpc.r1", 1.0);   // steering input
    
    Eigen::DiagonalMatrix<double, 4> Q;
    Eigen::DiagonalMatrix<double, 2> R;
    Q.diagonal() << q0, q1, q2, q3;
    R.diagonal() << r0, r1;
    cost_ = Cost(Q, R);
    
    // Initialize state and input sizes
    state_size_ = 4;  // x, y, theta, v
    input_size_ = 2;  // velocity, steering
    
    // Set up visualization publisher
    if (node_) {
        mpc_viz_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("mpc_trajectory", 10);
    }
    
    // Initialize GRAMPC
    initializeGrampc();
    
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "MPC initialized with horizon=%d, dt=%f", horizon_, dt_);
    }
}

void MPC::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    if (node_) {
        mpc_viz_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("mpc_trajectory", 10);
        
        // Re-read parameters if node is available
        horizon_ = node_->declare_parameter<int>("mpc.horizon", horizon_);
        dt_ = node_->declare_parameter<double>("mpc.dt", dt_);
        
        // Update cost matrices
        double q0 = node_->declare_parameter<double>("mpc.q0", 10.0);
        double q1 = node_->declare_parameter<double>("mpc.q1", 10.0);
        double q2 = node_->declare_parameter<double>("mpc.q2", 1.0);
        double q3 = node_->declare_parameter<double>("mpc.q3", 1.0);
        double r0 = node_->declare_parameter<double>("mpc.r0", 0.1);
        double r1 = node_->declare_parameter<double>("mpc.r1", 1.0);
        
        Eigen::DiagonalMatrix<double, 4> Q;
        Eigen::DiagonalMatrix<double, 2> R;
        Q.diagonal() << q0, q1, q2, q3;
        R.diagonal() << r0, r1;
        cost_ = Cost(Q, R);
        
        // Initialize GRAMPC solver now that we have the node
        if (!solver_initialized_) {
            initializeGrampc();
        }
        
        RCLCPP_INFO(node_->get_logger(), "MPC node pointer set and parameters updated");
    }
}
MPC::~MPC() {
    if (grampc_) {
        grampc_free(&grampc_);
    }
    if (ctx_) {
        free(ctx_);
        ctx_ = nullptr;
    }
}

void MPC::initializeGrampc() {
    if (grampc_ != nullptr) return;

    // Allocate user context with calloc to avoid new/delete mismatch
    ctx_ = calloc(1, sizeof(mpcc_ctx_t));
    mpcc_ctx_t* ctx = reinterpret_cast<mpcc_ctx_t*>(ctx_);
    ctx->L = 0.33;
    ctx->delta_max = 0.4;
    ctx->a_min = -2.0;
    ctx->a_max = 2.0;
    ctx->v_max = 5.0;
    ctx->path_ptr = nullptr;

    grampc_init(&grampc_, ctx_);

    if (!grampc_ || !grampc_->param || !grampc_->opt || !grampc_->sol) {
        if (node_) RCLCPP_FATAL(node_->get_logger(), "GRAMPC init failed (null pointers)");
        return;
    }

    const int Nx = grampc_->param->Nx;
    const int Nu = grampc_->param->Nu;
    if (Nx != 4 || Nu != 2) {
        if (node_) RCLCPP_FATAL(node_->get_logger(), "Unexpected GRAMPC dimensions Nx=%d Nu=%d (expected 4,2)", Nx, Nu);
        return;
    }

    // Adjust horizon only through exposed fields (GRAMPC has no setter for Nhor); safe to modify opt->Nhor before first run
    grampc_->opt->Nhor = horizon_;
    grampc_->param->Thor = horizon_ * dt_;
    grampc_->param->dt = dt_;

    if (node_) RCLCPP_INFO(node_->get_logger(), "Post-init dims Nx=%d Nu=%d Nhor=%d", Nx, Nu, grampc_->opt->Nhor);

    // Bounds & initial values
    double umin[2] = {0.0, -0.4};
    double umax[2] = {5.0,  0.4};
    grampc_setparam_real_vector(grampc_, "umin", umin);
    grampc_setparam_real_vector(grampc_, "umax", umax);

    double x0[4] = {0,0,0,0};
    double u0[2] = {1.0,0.0};
    double xdes[4] = {0,0,0,2.0};
    grampc_setparam_real_vector(grampc_, "x0", x0);
    grampc_setparam_real_vector(grampc_, "u0", u0);
    grampc_setparam_real_vector(grampc_, "xdes", xdes);

    // Skip penalty estimation first to isolate crash
    // grampc_estim_penmin(grampc_, 1);

    solver_initialized_ = true;
    if (node_) RCLCPP_INFO(node_->get_logger(), "GRAMPC init OK Nx=%d Nu=%d Nhor=%d Thor=%.3f dt=%.3f", Nx, Nu, grampc_->opt->Nhor, grampc_->param->Thor, grampc_->param->dt);
}

void MPC::update(const State& current_state, const Input& previous_input, 
                 const std::vector<State>& reference_trajectory) {
    
    if (!solver_initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "GRAMPC solver not initialized");
        return;
    }
    
    current_state_ = current_state;
    reference_trajectory_ = reference_trajectory;
    previous_input_ = previous_input;
    
    // Update model linearization
    model_.linearize(current_state, previous_input, dt_);
    
    // Update constraints with current state
    constraints_.setState(current_state);
    
    // Set current state in GRAMPC
    std::vector<double> x0 = {current_state.x(), current_state.y(), 
                             current_state.theta(), current_state.v()};
    
    // Validate current state
    if (!std::isfinite(x0[0]) || !std::isfinite(x0[1]) || !std::isfinite(x0[2]) || 
        !std::isfinite(x0[3]) || x0[3] < 0.0) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid current state: [%.3f, %.3f, %.3f, %.3f]", 
                    x0[0], x0[1], x0[2], x0[3]);
        has_solution_ = false;
        return;
    }
    
    grampc_setparam_real_vector(grampc_, "x0", x0.data());
    
    // Improved reference: average a window ahead within lookahead time
    if (!reference_trajectory.empty()) {
        double accum_x=0, accum_y=0, accum_theta_sin=0, accum_theta_cos=0, accum_v=0; int count=0;
        const double max_time = ref_lookahead_sec_;
        for (int i=0; i< (int)reference_trajectory.size(); ++i) {
            double t = i * dt_;
            if (t > max_time) break;
            const State& r = reference_trajectory[i];
            accum_x += r.x(); accum_y += r.y();
            accum_theta_sin += std::sin(r.theta());
            accum_theta_cos += std::cos(r.theta());
            accum_v += r.v();
            ++count;
        }
        if (count>0) {
            double ref_x = accum_x / count;
            double ref_y = accum_y / count;
            double ref_theta = std::atan2(accum_theta_sin / count, accum_theta_cos / count);
            double ref_v = accum_v / count;
            double xdes_arr[4] = {ref_x, ref_y, ref_theta, ref_v};
            grampc_setparam_real_vector(grampc_, "xdes", xdes_arr);
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Empty reference trajectory, using current state as reference");
        std::vector<double> xdes = {current_state.x(), current_state.y(), 
                                  current_state.theta(), 2.0};
        grampc_setparam_real_vector(grampc_, "xdes", xdes.data());
    }
    
    // Solve MPC problem
    RCLCPP_DEBUG(node_->get_logger(), "Running GRAMPC solver...");
    grampc_run(grampc_);
    
    // Check solver status
    RCLCPP_DEBUG(node_->get_logger(), "GRAMPC solver status: %d", grampc_->sol->status);
    if (grampc_->sol->status == 0) {
        has_solution_ = true;
        extractSolution();
        RCLCPP_DEBUG(node_->get_logger(), "MPC solved successfully");
    } else {
        has_solution_ = false;
        RCLCPP_WARN(node_->get_logger(), "MPC solver failed with status: %d", grampc_->sol->status);
        
        // Print additional debug information
        RCLCPP_WARN(node_->get_logger(), "Current state: [%.3f, %.3f, %.3f, %.3f]", 
                   x0[0], x0[1], x0[2], x0[3]);
        if (!reference_trajectory.empty()) {
            const State& ref = reference_trajectory[0];
            RCLCPP_WARN(node_->get_logger(), "Reference: [%.3f, %.3f, %.3f, %.3f]", 
                       ref.x(), ref.y(), ref.theta(), ref.v());
        }
        
        // Provide fallback solution
        solved_trajectory_.clear();
        solved_trajectory_.emplace_back(1.0, 0.0);  // Default: move forward, no steering
    }
}

void MPC::extractSolution() {
    solved_trajectory_.clear();
    if (!grampc_ || !grampc_->sol || !grampc_->sol->unext) return;
    const int Nu = grampc_->param->Nu;
    const int Nhor_alloc = grampc_->opt->Nhor;
    const int steps = std::min(horizon_, Nhor_alloc);
    // Safety: unext length should be Nu * Nhor
    for (int i = 0; i < steps; ++i) {
        const int idx = i * Nu;
        double vel = grampc_->sol->unext[idx + 0];
        double steer = grampc_->sol->unext[idx + 1];
        solved_trajectory_.emplace_back(vel, steer);
    }
}

void MPC::updateScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg) {
    scan_msg_ = *scan_msg;
    constraints_.findHalfSpaces(current_state_, scan_msg_);
}

void MPC::visualize() {
    if (!mpc_viz_pub_ || !has_solution_) {
        return;
    }
    
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "mpc_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.scale.x = 0.05;  // Line width
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    // Simulate trajectory forward using solved inputs
    State sim_state = current_state_;
    marker.points.emplace_back();
    marker.points.back().x = sim_state.x();
    marker.points.back().y = sim_state.y();
    marker.points.back().z = 0.0;
    
    for (const auto& input : solved_trajectory_) {
        State next_state;
        model_.simulateDynamics(sim_state, input, dt_, next_state);
        
        geometry_msgs::msg::Point point;
        point.x = next_state.x();
        point.y = next_state.y();
        point.z = 0.0;
        marker.points.push_back(point);
        
        sim_state = next_state;
    }
    
    mpc_viz_pub_->publish(marker);
}

} // namespace control_grampc
