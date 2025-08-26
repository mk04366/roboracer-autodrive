#include "control_grampc/mpc.h"
#include "control_grampc/mpcc_model.h"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>
#include <algorithm>

namespace control_grampc
{
    namespace {
        template<typename T>
        inline T clamp(T v, T lo, T hi){ return std::max(lo, std::min(hi, v)); }

        // Linear interpolation helper for State assuming uniform sampling at dt
        inline State lerpState(const State& a, const State& b, double alpha){
            // Normalize heading difference for smooth interpolation
            auto wrap = [](double ang){
                while (ang > M_PI) ang -= 2.0*M_PI;
                while (ang < -M_PI) ang += 2.0*M_PI;
                return ang;
            };
            double dx = b.x() - a.x();
            double dy = b.y() - a.y();
            double dth = wrap(b.theta() - a.theta());
            double dv = b.v() - a.v();
            return State(
                a.x()     + alpha * dx,
                a.y()     + alpha * dy,
                wrap(a.theta() + alpha * dth),
                a.v()     + alpha * dv
            );
        }

        // Interpolate a lookahead reference at fractional index idx_f
        inline State interpolateReference(const std::vector<State>& ref, double idx_f){
            if (ref.empty()) return State();
            if (idx_f <= 0.0) return ref.front();
            if (idx_f >= static_cast<double>(ref.size()-1)) return ref.back();
            int i0 = static_cast<int>(std::floor(idx_f));
            int i1 = i0 + 1;
            double alpha = idx_f - static_cast<double>(i0);
            return lerpState(ref[static_cast<size_t>(i0)], ref[static_cast<size_t>(i1)], alpha);
        }
    }

    MPC::MPC()
        : grampc_(nullptr), ctx_(nullptr), horizon_(20), state_size_(4), input_size_(2), dt_(0.05),
          solver_initialized_(false), has_solution_(false), failure_count_(0), last_cost_(0.0)
    {
        state_size_ = 4; // x, y, theta, v
        input_size_ = 2; // velocity, steering

        Eigen::DiagonalMatrix<double, 4> Q;
        Eigen::DiagonalMatrix<double, 2> R;
        Q.diagonal() << 80.0, 80.0, 25.0, 150.0; // x, y, theta, v
        R.diagonal() << 0.2, 1.5;                // velocity, steering
        cost_ = Cost(Q, R);

        ref_lookahead_sec_ = 0.5; // default
    }

    MPC::MPC(rclcpp::Node::SharedPtr node)
        : node_(node), grampc_(nullptr), ctx_(nullptr), solver_initialized_(false), has_solution_(false), 
          failure_count_(0), last_cost_(0.0)
    {
        if (!node_)
        {
            horizon_ = 20;
            dt_ = 0.05;
            state_size_ = 4;
            input_size_ = 2;
            ref_lookahead_sec_ = 0.5;
            
            Eigen::DiagonalMatrix<double, 4> Q;
            Eigen::DiagonalMatrix<double, 2> R;
            Q.diagonal() << 10.0, 10.0, 1.0, 1.0; // x, y, theta, v
            R.diagonal() << 0.1, 1.0;             // velocity, steering
            cost_ = Cost(Q, R);
            return;
        }

        horizon_ = node_->declare_parameter<int>("mpc.horizon", 20);  
        dt_      = node_->declare_parameter<double>("mpc.dt", 0.05);      
        ref_lookahead_sec_ = node_->declare_parameter<double>("mpc.ref_lookahead_sec", 0.5);

        double q0 = node_->declare_parameter<double>("mpc.q0", 10.0); 
        double q1 = node_->declare_parameter<double>("mpc.q1", 10.0); 
        double q2 = node_->declare_parameter<double>("mpc.q2", 5.0);
        double q3 = node_->declare_parameter<double>("mpc.q3", 20.0); 
        double r0 = node_->declare_parameter<double>("mpc.r0", 5.0);  
        double r1 = node_->declare_parameter<double>("mpc.r1", 10.0);   

        Eigen::DiagonalMatrix<double, 4> Q;
        Eigen::DiagonalMatrix<double, 2> R;
        Q.diagonal() << q0, q1, q2, q3;
        R.diagonal() << r0, r1;
        cost_ = Cost(Q, R);

        state_size_ = 4;
        input_size_ = 2;

        if (node_)
            mpc_viz_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("mpc_trajectory", 10);

        initializeGrampc();

        if (node_)
            RCLCPP_INFO(node_->get_logger(), "MPC initialized with horizon=%d, dt=%f", horizon_, dt_);
    }

    void MPC::setNode(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        if (node_)
        {
            mpc_viz_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("mpc_trajectory", 10);

            // Re-read parameters
            horizon_ = node_->declare_parameter<int>("mpc.horizon", horizon_);
            dt_      = node_->declare_parameter<double>("mpc.dt", dt_);
            
            // Get ref_lookahead_sec without declaring (already declared in node constructor)
            ref_lookahead_sec_ = node_->get_parameter_or("mpc.ref_lookahead_sec", ref_lookahead_sec_);

            double q0 = node_->declare_parameter<double>("mpc.q0", 10.0);   
            double q1 = node_->declare_parameter<double>("mpc.q1", 10.0);  
            double q2 = node_->declare_parameter<double>("mpc.q2", 5.0);   
            double q3 = node_->declare_parameter<double>("mpc.q3", 20.0); 
            double r0 = node_->declare_parameter<double>("mpc.r0", 5.0); 
            double r1 = node_->declare_parameter<double>("mpc.r1", 10.0);

            Eigen::DiagonalMatrix<double, 4> Q;
            Eigen::DiagonalMatrix<double, 2> R;
            Q.diagonal() << q0, q1, q2, q3;
            R.diagonal() << r0, r1;
            cost_ = Cost(Q, R);

            if (!solver_initialized_)
                initializeGrampc();

            // Make sure solver timing matches parameters immediately
            if (grampc_) {
                grampc_->opt->Nhor   = horizon_;
                grampc_->param->dt   = dt_;
                grampc_->param->Thor = horizon_ * dt_;
            }

            RCLCPP_INFO(node_->get_logger(), "MPC node set; params updated (Nhor=%d, dt=%.3f)", horizon_, dt_);
        }
    }

    MPC::~MPC()
    {
        if (grampc_)
            grampc_free(&grampc_);
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
        ctx->L         = 0.33;
        ctx->delta_max = 0.4;
        ctx->a_min     = -2.0;
        ctx->a_max     =  2.0;
        ctx->v_max     =  5.0;
        ctx->path_ptr  = nullptr;

        grampc_init(&grampc_, ctx_);

        if (!grampc_ || !grampc_->param || !grampc_->opt || !grampc_->sol)
        {
            if (node_) RCLCPP_FATAL(node_->get_logger(), "GRAMPC init failed (null pointers)");
            return;
        }

        const int Nx = grampc_->param->Nx;
        const int Nu = grampc_->param->Nu;
        if (Nx != 4 || Nu != 2)
        {
            if (node_) RCLCPP_FATAL(node_->get_logger(), "Unexpected GRAMPC dimensions Nx=%d Nu=%d (expected 4,2)", Nx, Nu);
            return;
        }

        // Use the configured horizon and dt (no test override)
        grampc_->opt->Nhor   = horizon_;
        grampc_->param->Thor = horizon_ * dt_;
        grampc_->param->dt   = dt_;

        // Bounds & initial values
        const double umin[2] = {0.0,  -0.4};
        const double umax[2] = {5.0,   0.4};
        grampc_setparam_real_vector(grampc_, "umin", const_cast<double*>(umin));
        grampc_setparam_real_vector(grampc_, "umax", const_cast<double*>(umax));

        double x0[4]   = {0, 0, 0, 0};
        double u0[2]   = {1.0, 0.0};
        double xdes[4] = {0, 0, 0, 2.0};
        grampc_setparam_real_vector(grampc_, "x0",   x0);
        grampc_setparam_real_vector(grampc_, "u0",   u0);
        grampc_setparam_real_vector(grampc_, "xdes", xdes);

        // More robust solver settings for startup and large errors
        grampc_setopt_real  (grampc_, "LineSearchMin", 1e-5);            
        grampc_setopt_real  (grampc_, "LineSearchMax", 0.5);             
        grampc_setopt_real  (grampc_, "LineSearchInit", 0.01);           
        grampc_setopt_int   (grampc_, "MaxGradIter", 15);               
        grampc_setopt_int   (grampc_, "MaxMultIter", 10);                
        grampc_setopt_real  (grampc_, "ConvergenceGradientRelTol", 1e-4); 
        grampc_setopt_string(grampc_, "LineSearchExpAutoFallback", "on");
        grampc_setopt_string(grampc_, "ShiftControl", "on");

        grampc_setopt_int   (grampc_, "MaxGradIter", 200);               
        grampc_setopt_int   (grampc_, "MaxMultIter", 3);                
        grampc_setopt_real  (grampc_, "ConvergenceGradientRelTol", 1e-3); 
        grampc_setopt_real  (grampc_, "MultiplierMax", 100.0);           

        grampc_setopt_real  (grampc_, "PenaltyMin", 1e-2);             
        grampc_setopt_real  (grampc_, "PenaltyMax", 100.0);              
        grampc_setopt_real  (grampc_, "PenaltyIncreaseFactor", 2.0);

        solver_initialized_ = true;
        if (node_)
            RCLCPP_INFO(node_->get_logger(), "GRAMPC init OK Nx=%d Nu=%d Nhor=%d Thor=%.3f dt=%.3f",
                        Nx, Nu, grampc_->opt->Nhor, grampc_->param->Thor, grampc_->param->dt);
    }

    void MPC::update(const State &current_state, const Input &previous_input,
                     const std::vector<State> &reference_trajectory)
    {
        if (!solver_initialized_)
        {
            RCLCPP_ERROR(node_->get_logger(), "GRAMPC solver not initialized");
            return;
        }

        // Keep solver timing consistent with node parameters at runtime
        if (grampc_) {
            if (grampc_->opt->Nhor != horizon_) {
                grampc_->opt->Nhor = horizon_;
                grampc_->param->Thor = horizon_ * dt_;
            }
            if (std::abs(grampc_->param->dt - dt_) > 1e-12) {
                grampc_->param->dt = dt_;
                grampc_->param->Thor = grampc_->opt->Nhor * dt_;
            }
        }

        current_state_        = current_state;
        reference_trajectory_ = reference_trajectory;
        previous_input_       = previous_input;

        constraints_.setState(current_state);

        // Set current state in GRAMPC
        std::vector<double> x0 = {current_state.x(), current_state.y(), current_state.theta(), current_state.v()};
        
        // Track the actual reference that gets set in GRAMPC for error logging
        State actual_set_reference(0, 0, 0, 0);

        // Validate current state (keeps non-negative v; relax if reverse driving is desired)
        if (!std::isfinite(x0[0]) || !std::isfinite(x0[1]) || !std::isfinite(x0[2]) ||
            !std::isfinite(x0[3]) || x0[3] < 0.0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid current state: [%.3f, %.3f, %.3f, %.3f]",
                         x0[0], x0[1], x0[2], x0[3]);
            has_solution_ = false;
            return;
        }
        grampc_setparam_real_vector(grampc_, "x0", x0.data());

        if (!reference_trajectory.empty())
        {
            const State& target_ref = reference_trajectory.front();
            double dx = target_ref.x() - current_state.x();
            double dy = target_ref.y() - current_state.y();
            double dist_to_target = std::sqrt(dx * dx + dy * dy);
            
            State r;
            if (dist_to_target > 0.8) { 
                double scale = std::min(1.0, 0.3 / dist_to_target);  
                double vel_ramp = std::min(0.1, current_state.v() + 0.1); 
                r = State(
                    current_state.x() + dx * scale,
                    current_state.y() + dy * scale,
                    current_state.theta() + (target_ref.theta() - current_state.theta()) * scale,
                    vel_ramp  // Start very slow and ramp up
                );
                RCLCPP_INFO(node_->get_logger(), "PROGRESSIVE REF: target=[%.3f, %.3f] current=[%.3f, %.3f] dist=%.3f scale=%.3f → inter=[%.3f, %.3f] vel=%.3f", 
                           target_ref.x(), target_ref.y(), current_state.x(), current_state.y(), dist_to_target, scale, r.x(), r.y(), vel_ramp);
            } else {
                const double idx_f = ref_lookahead_sec_ / dt_;
                r = interpolateReference(reference_trajectory, idx_f);
                double max_vel_step = current_state.v() + 1.0;
                r = State(r.x(), r.y(), r.theta(), std::min(max_vel_step, std::min(3.0, r.v())));
            }
            
            double xdes_arr[4] = {r.x(), r.y(), r.theta(), r.v()};
            grampc_setparam_real_vector(grampc_, "xdes", xdes_arr);
            
            actual_set_reference = r;
            
            RCLCPP_INFO(node_->get_logger(), "SET REFERENCE: [x=%.3f, y=%.3f, θ=%.3f, v=%.3f] | Error: [Δx=%.3f, Δy=%.3f, Δθ=%.3f, Δv=%.3f]",
                        r.x(), r.y(), r.theta(), r.v(),
                        r.x() - current_state.x(), r.y() - current_state.y(), 
                        r.theta() - current_state.theta(), r.v() - current_state.v());

            double u0_init[2];
            bool set_u0 = false;

            if (!solved_trajectory_.empty()) {
                u0_init[0] = solved_trajectory_.front().velocity();
                u0_init[1] = solved_trajectory_.front().steeringAngle();
                set_u0 = true;
                RCLCPP_DEBUG(node_->get_logger(), "Warm start from last solution: v=%.2f, steer=%.3f",
                             u0_init[0], u0_init[1]);
            } else {
                if (std::isfinite(previous_input.velocity()) && std::isfinite(previous_input.steeringAngle()) && 
                    previous_input.velocity() > 0.1) {
                    u0_init[0] = previous_input.velocity();
                    u0_init[1] = previous_input.steeringAngle();
                    set_u0 = true;
                    RCLCPP_DEBUG(node_->get_logger(), "Warm start from previous_input: v=%.2f, steer=%.3f",
                                 u0_init[0], u0_init[1]);
                }
            }

            if (!set_u0) {
                const State& ref = reference_trajectory.front();
                double dx = ref.x() - current_state.x();
                double dy = ref.y() - current_state.y();
                double dist_to_ref = std::sqrt(dx * dx + dy * dy);
                
                double target_vel = std::min(2.0, std::max(0.5, ref.v() * 0.5));  // Lower velocity for large errors
                
                double heading_error = ref.theta() - current_state.theta();
                while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
                while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
                
                double steer_scaling = std::min(1.0, 2.0 / std::max(2.0, dist_to_ref));  // Scale down for large distances
                double target_steer = std::max(-0.2, std::min(0.2, heading_error * 0.3 * steer_scaling));
                
                u0_init[0] = target_vel;
                u0_init[1] = target_steer;
                RCLCPP_DEBUG(node_->get_logger(), "Intelligent warm start: v=%.2f, steer=%.3f (dist=%.2f, h_err=%.3f)", 
                           target_vel, target_steer, dist_to_ref, heading_error);
            }

            u0_init[0] = clamp(u0_init[0], 0.0, 5.0);
            u0_init[1] = clamp(u0_init[1], -0.4, 0.4);
            grampc_setparam_real_vector(grampc_, "u0", u0_init);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Empty reference trajectory, using current state as reference");
            std::vector<double> xdes = {current_state.x(), current_state.y(), current_state.theta(), 2.0};
            grampc_setparam_real_vector(grampc_, "xdes", xdes.data());
            
            actual_set_reference = State(current_state.x(), current_state.y(), current_state.theta(), 2.0);

            double u0_init[2] = { std::max(1.0, current_state.v()), 0.0 };
            u0_init[0] = clamp(u0_init[0], 0.0, 5.0);
            grampc_setparam_real_vector(grampc_, "u0", u0_init);
        }

        // Check distance to path for debugging large error cases
        if (!reference_trajectory.empty()) {
            const State &ref = reference_trajectory.front();
            double dx = ref.x() - current_state.x();
            double dy = ref.y() - current_state.y();
            double distance_to_path = std::sqrt(dx * dx + dy * dy);
            
            if (distance_to_path > 5.0) {
                RCLCPP_WARN(node_->get_logger(), "Large tracking error detected: %.2f m - GRAMPC may struggle", distance_to_path);
            }
        }

        // Solve MPC problem
        RCLCPP_DEBUG(node_->get_logger(), "Running GRAMPC solver...");
        grampc_run(grampc_);

        // Debug distance to first ref point
        double dist_error = 0.0;
        if (!reference_trajectory.empty())
        {
            const State &ref = reference_trajectory.front();
            double dx = ref.x() - current_state.x();
            double dy = ref.y() - current_state.y();
            dist_error = std::sqrt(dx * dx + dy * dy);
        }

        if (grampc_->sol->status == 0)
        {
            has_solution_ = true;
            extractSolution();
            failure_count_ = 0;
            
            double current_cost = *(grampc_->sol->J);
            last_cost_ = current_cost;
            
            RCLCPP_INFO(node_->get_logger(),
                        "GRAMPC SUCCESS: status=%d, iter=%d, cfct=%.2e, J=%.3f, dist=%.2fm",
                        grampc_->sol->status, *(grampc_->sol->iter), grampc_->sol->cfct,
                        current_cost, dist_error);
            
            // DEBUG: Print solution details
            if (grampc_->sol->unext && !solved_trajectory_.empty()) {
                RCLCPP_INFO(node_->get_logger(), "GRAMPC direct: u[0]=%.3f, u[1]=%.3f | Final: v=%.3f, steer=%.3f", 
                           grampc_->sol->unext[0], grampc_->sol->unext[1],
                           solved_trajectory_[0].velocity(), solved_trajectory_[0].steeringAngle());
            }
        }
        else
        {
            has_solution_ = false;
            failure_count_++;
            solved_trajectory_.clear();

            // Detailed failure analysis
            std::string status_meaning;
            switch(grampc_->sol->status) {
                case 1:  status_meaning = "MAX_ITERATIONS_REACHED"; break;
                case 2:  status_meaning = "CONVERGED"; break;
                case 4:  status_meaning = "LINE_SEARCH_MIN"; break;
                case 8:  status_meaning = "LINE_SEARCH_MAX"; break;
                case 12: status_meaning = "LINE_SEARCH_MIN+MAX"; break;
                default: status_meaning = "UNKNOWN_STATUS"; break;
            }

            RCLCPP_ERROR(node_->get_logger(), "GRAMPC FAILURE: status=%d (%s), iter=%d, cfct=%.6e, J=%.6f, dist=%.3f",
                         grampc_->sol->status, status_meaning.c_str(),
                         *(grampc_->sol->iter), grampc_->sol->cfct, *(grampc_->sol->J), dist_error);

            RCLCPP_ERROR(node_->get_logger(), "  Current: [x=%.3f, y=%.3f, θ=%.3f, v=%.3f]",
                         x0[0], x0[1], x0[2], x0[3]);

            // Show the actual reference that was set in GRAMPC (not the original from trajectory)
            double dx = actual_set_reference.x() - x0[0];
            double dy = actual_set_reference.y() - x0[1];
            double dtheta = actual_set_reference.theta() - x0[2];
            double dv = actual_set_reference.v() - x0[3];
            RCLCPP_ERROR(node_->get_logger(),
                "  Actual Set Reference: [x=%.3f, y=%.3f, θ=%.3f, v=%.3f] | Error: [Δx=%.3f, Δy=%.3f, Δθ=%.3f, Δv=%.3f]",
                actual_set_reference.x(), actual_set_reference.y(), actual_set_reference.theta(), actual_set_reference.v(), 
                dx, dy, dtheta, dv);

            RCLCPP_ERROR(node_->get_logger(), "GRAMPC failed - no solution available (failure #%d)", failure_count_);
        }
    }

    void MPC::extractSolution()
    {
        solved_trajectory_.clear();
        if (!grampc_ || !grampc_->sol || !grampc_->sol->unext)
        {
            RCLCPP_ERROR(node_->get_logger(), "GRAMPC solution pointers are null");
            return;
        }

        const int Nu         = grampc_->param->Nu;
        const int Nhor_alloc = grampc_->opt->Nhor;
        const int steps      = std::min(horizon_, Nhor_alloc);
        
        RCLCPP_DEBUG(node_->get_logger(), "Extracting solution: Nu=%d, Nhor=%d, steps=%d", Nu, Nhor_alloc, steps);

        for (int i = 0; i < steps; ++i)
        {
            const int idx = i * Nu;
            double vel   = grampc_->sol->unext[idx + 0];
            double steer = grampc_->sol->unext[idx + 1];
            
            RCLCPP_DEBUG(node_->get_logger(), "Step %d: idx=%d, vel=%.3f, steer=%.3f", i, idx, vel, steer);
            
            solved_trajectory_.emplace_back(vel, steer);
        }
        
        RCLCPP_DEBUG(node_->get_logger(), "Extracted %zu trajectory points", solved_trajectory_.size());
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
            if (node_)
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "MPC visualization publisher not available or no solution");
            return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp    = node_->now();
        marker.ns   = "mpc_trajectory";
        marker.id   = 0;
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

}
