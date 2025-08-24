#include "control_grampc/mpc.h"
#include "control_grampc/mpcc_model.h"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>

namespace control_grampc
{

    MPC::MPC()
        : grampc_(nullptr), ctx_(nullptr), horizon_(20), state_size_(4), input_size_(2), dt_(0.05), solver_initialized_(false), has_solution_(false)
    {
        // Initialize state and input sizes
        state_size_ = 4; // x, y, theta, v
        input_size_ = 2; // velocity, steering

        // Default cost matrices
        Eigen::DiagonalMatrix<double, 4> Q;
        Eigen::DiagonalMatrix<double, 2> R;
        Q.diagonal() << 10.0, 10.0, 1.0, 1.0; // x, y, theta, v
        R.diagonal() << 0.1, 1.0;             // velocity, steering
        cost_ = Cost(Q, R);

        ref_lookahead_sec_ = 0.5; // default
    }

    MPC::MPC(rclcpp::Node::SharedPtr node)
        : node_(node), grampc_(nullptr), ctx_(nullptr), solver_initialized_(false), has_solution_(false)
    {
        if (!node_)
        {
            // Fall back to default constructor behavior
            MPC();
            return;
        }

        // Get parameters from node
        horizon_ = node_->declare_parameter<int>("mpc.horizon", 20);
        dt_ = node_->declare_parameter<double>("mpc.dt", 0.05);

        // Set up cost matrices
        double q0 = node_->declare_parameter<double>("mpc.q0", 10.0); // x position
        double q1 = node_->declare_parameter<double>("mpc.q1", 10.0); // y position
        double q2 = node_->declare_parameter<double>("mpc.q2", 1.0);  // orientation
        double q3 = node_->declare_parameter<double>("mpc.q3", 1.0);  // velocity

        double r0 = node_->declare_parameter<double>("mpc.r0", 0.1); // velocity input
        double r1 = node_->declare_parameter<double>("mpc.r1", 1.0); // steering input

        Eigen::DiagonalMatrix<double, 4> Q;
        Eigen::DiagonalMatrix<double, 2> R;
        Q.diagonal() << q0, q1, q2, q3;
        R.diagonal() << r0, r1;
        cost_ = Cost(Q, R);

        // Initialize state and input sizes
        state_size_ = 4; // x, y, theta, v
        input_size_ = 2; // velocity, steering

        // Set up visualization publisher
        if (node_)
        {
            mpc_viz_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("mpc_trajectory", 10);
        }

        // Initialize GRAMPC
        initializeGrampc();

        if (node_)
        {
            RCLCPP_INFO(node_->get_logger(), "MPC initialized with horizon=%d, dt=%f", horizon_, dt_);
        }
    }

    void MPC::setNode(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        if (node_)
        {
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
            if (!solver_initialized_)
            {
                initializeGrampc();
            }

            RCLCPP_INFO(node_->get_logger(), "MPC node pointer set and parameters updated");
        }
    }
    MPC::~MPC()
    {
        if (grampc_)
        {
            grampc_free(&grampc_);
        }
        if (ctx_)
        {
            free(ctx_);
            ctx_ = nullptr;
        }
    }

    void MPC::initializeGrampc()
    {
        if (grampc_ != nullptr)
            return;

        // Allocate user context with calloc to avoid new/delete mismatch
        ctx_ = calloc(1, sizeof(mpcc_ctx_t));
        mpcc_ctx_t *ctx = reinterpret_cast<mpcc_ctx_t *>(ctx_);
        ctx->L = 0.33;
        ctx->delta_max = 0.4;
        ctx->a_min = -2.0;
        ctx->a_max = 2.0;
        ctx->v_max = 5.0;
        ctx->path_ptr = nullptr;

        grampc_init(&grampc_, ctx_);

        if (!grampc_ || !grampc_->param || !grampc_->opt || !grampc_->sol)
        {
            if (node_)
                RCLCPP_FATAL(node_->get_logger(), "GRAMPC init failed (null pointers)");
            return;
        }

        const int Nx = grampc_->param->Nx;
        const int Nu = grampc_->param->Nu;
        if (Nx != 4 || Nu != 2)
        {
            if (node_)
                RCLCPP_FATAL(node_->get_logger(), "Unexpected GRAMPC dimensions Nx=%d Nu=%d (expected 4,2)", Nx, Nu);
            return;
        }

        // Adjust horizon only through exposed fields (GRAMPC has no setter for Nhor); safe to modify opt->Nhor before first
        // run

        // Use reduced horizon for better convergence - TEST
        int test_horizon = 10;  // Much shorter horizon to test if that helps
        grampc_->opt->Nhor = test_horizon;
        grampc_->param->Thor = test_horizon * dt_;
        grampc_->param->dt = dt_;

        if (node_)
            RCLCPP_WARN(node_->get_logger(), "TESTING: Using short horizon Nhor=%d for better convergence", test_horizon);

        // Bounds & initial values
        double umin[2] = {0.0, -0.4};
        double umax[2] = {5.0, 0.4};
        grampc_setparam_real_vector(grampc_, "umin", umin);
        grampc_setparam_real_vector(grampc_, "umax", umax);

        double x0[4] = {0, 0, 0, 0};
        double u0[2] = {1.0, 0.0};
        double xdes[4] = {0, 0, 0, 2.0};
        grampc_setparam_real_vector(grampc_, "x0", x0);
        grampc_setparam_real_vector(grampc_, "u0", u0);
        grampc_setparam_real_vector(grampc_, "xdes", xdes);

        // Set more conservative solver settings to avoid status 4
        grampc_setopt_real(grampc_, "LineSearchMin", 1e-12);            // Much smaller minimum step
        grampc_setopt_real(grampc_, "LineSearchMax", 0.1);              // Much smaller maximum step
        grampc_setopt_real(grampc_, "LineSearchInit", 0.01);            // Very small initial step
        grampc_setopt_string(grampc_, "LineSearchExpAutoFallback", "on");
        grampc_setopt_string(grampc_, "ShiftControl", "on");
        
        grampc_setopt_int(grampc_, "MaxGradIter", 5);                   // Fewer gradient iterations
        grampc_setopt_int(grampc_, "MaxMultIter", 1);                   // Minimal multiplier iterations
        grampc_setopt_real(grampc_, "ConvergenceGradientRelTol", 1e-1); // Very relaxed convergence
        grampc_setopt_real(grampc_, "MultiplierMax", 10.0);             // Lower multiplier limit

        // Very conservative penalty tuning to avoid numerical issues
        grampc_setopt_real(grampc_, "PenaltyMin", 1e-1);
        grampc_setopt_real(grampc_, "PenaltyMax", 10.0);
        grampc_setopt_real(grampc_, "PenaltyIncreaseFactor", 1.1);
        
        solver_initialized_ = true;
        if (node_)
            RCLCPP_INFO(node_->get_logger(), "GRAMPC init OK Nx=%d Nu=%d Nhor=%d Thor=%.3f dt=%.3f", Nx, Nu, grampc_->opt->Nhor,
                        grampc_->param->Thor, grampc_->param->dt);
    }

    void MPC::update(const State &current_state, const Input &previous_input,
                     const std::vector<State> &reference_trajectory)
    {
        if (!solver_initialized_)
        {
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
        std::vector<double> x0 = {current_state.x(), current_state.y(), current_state.theta(), current_state.v()};

        // Validate current state
        if (!std::isfinite(x0[0]) || !std::isfinite(x0[1]) || !std::isfinite(x0[2]) || !std::isfinite(x0[3]) || x0[3] < 0.0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid current state: [%.3f, %.3f, %.3f, %.3f]", x0[0], x0[1], x0[2], x0[3]);
            has_solution_ = false;
            return;
        }

        grampc_setparam_real_vector(grampc_, "x0", x0.data());

        // Improved reference: use lookahead for better tracking
        if (!reference_trajectory.empty())
        {
            // TEST: Try trivial reference to isolate GRAMPC issues
            static int debug_counter = 0;
            debug_counter++;
            bool use_trivial = (debug_counter % 20 < 5); // Use trivial reference 25% of the time for testing
            
            if (use_trivial) {
                // Trivial reference: just maintain current state with slight forward motion
                double xdes_trivial[4] = {current_state.x() + 0.1, current_state.y(), current_state.theta(), 1.0};
                grampc_setparam_real_vector(grampc_, "xdes", xdes_trivial);
                RCLCPP_WARN(node_->get_logger(), "DEBUG: Using trivial reference - should easily converge");
            } else {
                // Use a single lookahead state for proper curvature tracking
                int look_idx = static_cast<int>(ref_lookahead_sec_ / dt_);
                if (look_idx >= static_cast<int>(reference_trajectory.size()))
                {
                    look_idx = static_cast<int>(reference_trajectory.size()) - 1;
                }
                if (look_idx < 0)
                    look_idx = 0;
                const State &r = reference_trajectory[look_idx];
                double xdes_arr[4] = {r.x(), r.y(), r.theta(), r.v()};
                grampc_setparam_real_vector(grampc_, "xdes", xdes_arr);
            }

            // Provide better initial control guess when far from reference
            if (!use_trivial) {
                int look_idx = static_cast<int>(ref_lookahead_sec_ / dt_);
                if (look_idx >= static_cast<int>(reference_trajectory.size()))
                {
                    look_idx = static_cast<int>(reference_trajectory.size()) - 1;
                }
                if (look_idx < 0)
                    look_idx = 0;
                const State &r = reference_trajectory[look_idx];
                
                double dx = r.x() - current_state.x();
                double dy = r.y() - current_state.y();
                double dist_error = std::sqrt(dx * dx + dy * dy);

                if (dist_error > 1.5)
                {
                    // Calculate desired heading and provide steering hint
                    double desired_heading = std::atan2(dy, dx);
                    double heading_error = std::atan2(std::sin(desired_heading - current_state.theta()),
                                                      std::cos(desired_heading - current_state.theta()));

                    double vel_hint = std::min(3.0, std::max(1.0, r.v()));
                    double steer_hint = std::max(-0.3, std::min(0.3, heading_error * 0.5));

                    double u0_hint[2] = {vel_hint, steer_hint};
                    grampc_setparam_real_vector(grampc_, "u0", u0_hint);

                    RCLCPP_DEBUG(node_->get_logger(), "Using control hint: v=%.2f, steer=%.2f", vel_hint, steer_hint);
                } else if (!solved_trajectory_.empty()) {
                    // Use previous solution as warm start
                    double u0_warm[2] = {solved_trajectory_[0].velocity(), solved_trajectory_[0].steeringAngle()};
                    grampc_setparam_real_vector(grampc_, "u0", u0_warm);
                    RCLCPP_DEBUG(node_->get_logger(), "Using warm start from previous solution");
                }
            } else {
                // For trivial reference, use simple control guess
                double u0_simple[2] = {1.0, 0.0};
                grampc_setparam_real_vector(grampc_, "u0", u0_simple);
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Empty reference trajectory, using current state as reference");
            std::vector<double> xdes = {current_state.x(), current_state.y(), current_state.theta(), 2.0};
            grampc_setparam_real_vector(grampc_, "xdes", xdes.data());
        }

        // Solve MPC problem
        RCLCPP_DEBUG(node_->get_logger(), "Running GRAMPC solver...");
        grampc_run(grampc_);

        // Calculate distance to reference for debugging
        double dist_error = 0.0;
        if (!reference_trajectory.empty())
        {
            const State &ref = reference_trajectory[0];
            double dx = ref.x() - current_state.x();
            double dy = ref.y() - current_state.y();
            dist_error = std::sqrt(dx * dx + dy * dy);
        }

        if (grampc_->sol->status == 0)
        {
            has_solution_ = true;
            extractSolution();
            RCLCPP_INFO(node_->get_logger(), "GRAMPC SUCCESS: status=%d, iter=%d, cfct=%.2e, J=%.3f, dist=%.2fm", 
                       grampc_->sol->status, *(grampc_->sol->iter), grampc_->sol->cfct, *(grampc_->sol->J), dist_error);
        }
        else
        {
            has_solution_ = false;
            
            // COMPREHENSIVE DEBUG OUTPUT as requested
            RCLCPP_ERROR(node_->get_logger(), "GRAMPC FAILURE ANALYSIS:");
            RCLCPP_ERROR(node_->get_logger(), "  status=%d, iter=%d, cfct=%.6e, J=%.6f", 
                        grampc_->sol->status, *(grampc_->sol->iter), grampc_->sol->cfct, *(grampc_->sol->J));
            RCLCPP_ERROR(node_->get_logger(), "  dist_to_ref=%.3fm", dist_error);
            
            // Decode status meaning
            std::string status_meaning;
            switch(grampc_->sol->status) {
                case 1: status_meaning = "MAX_ITERATIONS_REACHED"; break;
                case 2: status_meaning = "CONVERGED"; break;
                case 4: status_meaning = "LINE_SEARCH_MIN (optimization stuck at minimum step)"; break;
                case 8: status_meaning = "LINE_SEARCH_MAX (optimization hit maximum step)"; break;
                case 12: status_meaning = "LINE_SEARCH_MIN+MAX (both limits hit)"; break;
                default: status_meaning = "UNKNOWN_STATUS"; break;
            }
            RCLCPP_ERROR(node_->get_logger(), "  Status meaning: %s", status_meaning.c_str());
            
            // Print current state and reference for analysis
            RCLCPP_ERROR(node_->get_logger(), "  Current: [x=%.3f, y=%.3f, θ=%.3f, v=%.3f]", 
                        x0[0], x0[1], x0[2], x0[3]);
            if (!reference_trajectory.empty()) {
                const State& ref = reference_trajectory[0];
                RCLCPP_ERROR(node_->get_logger(), "  Reference: [x=%.3f, y=%.3f, θ=%.3f, v=%.3f]", 
                           ref.x(), ref.y(), ref.theta(), ref.v());
                double dx = ref.x() - x0[0];
                double dy = ref.y() - x0[1];
                double dtheta = ref.theta() - x0[2];
                double dv = ref.v() - x0[3];
                RCLCPP_ERROR(node_->get_logger(), "  Error: [Δx=%.3f, Δy=%.3f, Δθ=%.3f, Δv=%.3f]", 
                           dx, dy, dtheta, dv);
            }

            // Provide intelligent fallback solution - Pure Pursuit Controller
            solved_trajectory_.clear();

            if (!reference_trajectory.empty())
            {
                const State &ref = reference_trajectory[0];
                double dx = ref.x() - current_state.x();
                double dy = ref.y() - current_state.y();
                double dist_error = std::sqrt(dx * dx + dy * dy);

                // Pure pursuit control law
                double desired_heading = std::atan2(dy, dx);
                double heading_error = std::atan2(std::sin(desired_heading - current_state.theta()),
                                                  std::cos(desired_heading - current_state.theta()));

                // Velocity control: slower when far from path, match reference when close
                double vel_cmd;
                if (dist_error > 2.0) {
                    vel_cmd = std::min(2.0, std::max(0.5, ref.v() * 0.6));
                } else if (dist_error > 0.5) {
                    vel_cmd = std::min(ref.v() * 1.1, std::max(1.0, ref.v() * 0.8));
                } else {
                    vel_cmd = ref.v();
                }

                // Steering control with lookahead
                double L = 0.33; // wheelbase
                double lookahead_dist = std::max(1.0, vel_cmd * 0.5); // adaptive lookahead
                
                // Find lookahead point
                int lookahead_idx = 0;
                for (size_t i = 0; i < reference_trajectory.size(); ++i) {
                    double dist_to_point = std::sqrt(
                        std::pow(reference_trajectory[i].x() - current_state.x(), 2) +
                        std::pow(reference_trajectory[i].y() - current_state.y(), 2)
                    );
                    if (dist_to_point >= lookahead_dist) {
                        lookahead_idx = i;
                        break;
                    }
                }
                
                if (lookahead_idx < reference_trajectory.size()) {
                    const State& lookahead_point = reference_trajectory[lookahead_idx];
                    double ld_x = lookahead_point.x() - current_state.x();
                    double ld_y = lookahead_point.y() - current_state.y();
                    double alpha = std::atan2(ld_y, ld_x) - current_state.theta();
                    double steer_cmd = std::atan2(2.0 * L * std::sin(alpha), lookahead_dist);
                    steer_cmd = std::max(-0.4, std::min(0.4, steer_cmd)); // Clamp to limits
                    
                    solved_trajectory_.emplace_back(vel_cmd, steer_cmd);
                    RCLCPP_DEBUG(node_->get_logger(), "Pure pursuit: vel=%.2f, steer=%.3f, dist=%.2f", 
                               vel_cmd, steer_cmd, dist_error);
                } else {
                    // Simple proportional control if lookahead fails
                    double steer_cmd = std::max(-0.3, std::min(0.3, heading_error * 2.0));
                    solved_trajectory_.emplace_back(vel_cmd, steer_cmd);
                }
            }
            else
            {
                solved_trajectory_.emplace_back(1.0, 0.0); // Default: move forward, no steering
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                    "No reference trajectory available");
            }
        }
    }

    void MPC::extractSolution()
    {
        solved_trajectory_.clear();
        if (!grampc_ || !grampc_->sol || !grampc_->sol->unext)
            return;
        const int Nu = grampc_->param->Nu;
        const int Nhor_alloc = grampc_->opt->Nhor;
        const int steps = std::min(horizon_, Nhor_alloc);
        // Safety: unext length should be Nu * Nhor
        for (int i = 0; i < steps; ++i)
        {
            const int idx = i * Nu;
            double vel = grampc_->sol->unext[idx + 0];
            double steer = grampc_->sol->unext[idx + 1];
            solved_trajectory_.emplace_back(vel, steer);
        }
    }

    void MPC::updateScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
    {
        scan_msg_ = *scan_msg;
        constraints_.findHalfSpaces(current_state_, scan_msg_);
    }

    void MPC::visualize()
    {
        if (!mpc_viz_pub_ || !has_solution_)
        {
            return;
        }

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "mpc_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05; // Line width
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

        for (const auto &input : solved_trajectory_)
        {
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
