#include "control_grampc/mpcc_model.h"
#include <math.h>
#include <stddef.h>

// Use GRAMPC standard definitions for consistency
#if USE_typeRNum == USE_FLOAT
#define SIN(a) sinf(a)
#define COS(a) cosf(a)
#define TAN(a) tanf(a)
#else
#define SIN(a) sin(a)
#define COS(a) cos(a)
#define TAN(a) tan(a)
#endif

// Square macro following GRAMPC convention
#define POW2(a) ((a) * (a))

static inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline double wrap_angle(double a)
{
    while (a > 3.14159265358979323846)
        a -= 2.0 * 3.14159265358979323846;
    while (a < -3.14159265358979323846)
        a += 2.0 * 3.14159265358979323846;
    return a;
}

void mpcc_dynamics(double t, const double *x, const double *u, double *xdot, void *user)
{
    (void)t;

    // Safety checks
    if (!x || !u || !xdot)
    {
        if (xdot)
        {
            xdot[0] = xdot[1] = xdot[2] = xdot[3] = 0.0;
        }
        return;
    }

    mpcc_ctx_t *C = (mpcc_ctx_t *)user;

    // Vehicle parameters with proper defaults
    double L = (C ? C->L : 0.33);
    double delta_max = (C ? C->delta_max : 0.4);
    double v_cmd_max = (C ? C->v_max : 5.0);
    double tau_v = 0.1; // velocity time constant

    // 4D state: [x, y, theta, v]
    double X = x[0], Y = x[1], THETA = x[2], V = x[3];
    double v_cmd = clamp(u[0], 0.0, v_cmd_max);        // velocity command
    double delta = clamp(u[1], -delta_max, delta_max); // steering angle

    // Bicycle model dynamics - following GRAMPC Vehicle example structure
    xdot[0] = V * COS(THETA);       // x_dot = v*cos(theta)
    xdot[1] = V * SIN(THETA);       // y_dot = v*sin(theta)
    xdot[2] = (V / L) * TAN(delta); // theta_dot = (v/L)*tan(delta)
    xdot[3] = (v_cmd - V) / tau_v;  // v_dot = (v_cmd - v)/tau (first-order velocity tracking)
}

double mpcc_stage_cost(double t, const double *x, const double *u, const double *xdes, void *user)
{
    (void)t;

    // Safety checks
    if (!x || !u)
    {
        return 0.0;
    }

    mpcc_ctx_t *C = (mpcc_ctx_t *)user;

    // Cost weights - following GRAMPC parameter structure
    // These should match the gradients exactly
    double w_x = 5.0;      // x position tracking
    double w_y = 5.0;      // y position tracking
    double w_theta = 10.0; // heading tracking
    double w_v = 1.0;      // velocity tracking
    double w_v_cmd = 0.1;  // velocity command effort
    double w_delta = 0.05; // steering effort

    // 4D state: [x, y, theta, v]
    double vehicle_x = x[0], vehicle_y = x[1], vehicle_theta = x[2], vehicle_v = x[3];

    // Reference values: [ref_x, ref_y, ref_theta, ref_v]
    double ref_x = xdes ? xdes[0] : 0.0;
    double ref_y = xdes ? xdes[1] : 0.0;
    double ref_theta = xdes ? xdes[2] : 0.0;
    double ref_v = xdes ? xdes[3] : 1.0;

    // Control inputs: [velocity_cmd, steering_angle]
    double v_cmd = u[0];
    double delta = u[1];

    // Calculate curvature-based feedforward steering (for control effort reference)
    double delta_ff = 0.0;
    if (C && xdes)
    {
        // Simple feedforward based on reference heading error
        double heading_error = wrap_angle(ref_theta - vehicle_theta);
        delta_ff = 0.5 * heading_error; // Proportional feedforward
        delta_ff = clamp(delta_ff, -0.4, 0.4);
    }

    // GRAMPC-style quadratic cost using POW2 macro
    double J = 0.0;

    // State tracking costs
    J += w_x * POW2(vehicle_x - ref_x);
    J += w_y * POW2(vehicle_y - ref_y);
    J += w_theta * POW2(wrap_angle(vehicle_theta - ref_theta));
    J += w_v * POW2(vehicle_v - ref_v);

    // Control effort costs
    J += w_v_cmd * POW2(v_cmd - ref_v);    // penalize deviation from reference velocity
    J += w_delta * POW2(delta - delta_ff); // penalize deviation from feedforward steering

    return J;
}

double mpcc_terminal_cost(const double *x, const double *xdes, void *user)
{
    (void)user;

    // Terminal cost weights - higher than stage cost for terminal constraint
    double w_x_T = 10.0;
    double w_y_T = 10.0;
    double w_theta_T = 5.0; // reduced weight for terminal heading
    double w_v_T = 1.0;

    // Reference values
    double ref_x = xdes ? xdes[0] : 0.0;
    double ref_y = xdes ? xdes[1] : 0.0;
    double ref_theta = xdes ? xdes[2] : 0.0;
    double ref_v = xdes ? xdes[3] : 1.0;

    // Calculate terminal cost using GRAMPC POW2 macro
    double V = w_x_T * POW2(x[0] - ref_x) + w_y_T * POW2(x[1] - ref_y) + w_theta_T * POW2(wrap_angle(x[2] - ref_theta)) + w_v_T * POW2(x[3] - ref_v);

    return V;
}

void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    (void)userparam;
    *Nx = 4; // x, y, theta, v
    *Nu = 2; // velocity_cmd, steering_angle
    *Np = 0;
    *Ng = 0; // No constraints for debugging
    *Nh = 0;
    *NgT = 0;
    *NhT = 0;
}

void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)p;
    (void)userparam;

    // Ultra-simple dynamics for debugging - just copy state
    if (!out || !x || !u)
    {
        return;
    }

    // Simple integrator dynamics for testing
    out[0] = x[3] * 0.5;  // x_dot = v/2 (slow motion)
    out[1] = 0.0;         // y_dot = 0
    out[2] = u[1] * 0.1;  // theta_dot = steering/10
    out[3] = u[0] - x[3]; // v_dot = v_cmd - v
}

void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;

    // Ultra-simple Jacobian for debugging - zero for now
    if (!out || !vec)
    {
        if (out)
        {
            for (int i = 0; i < 4; i++)
                out[i] = 0.0;
        }
        return;
    }

    // df/dx is zero for simple integrator dynamics
    for (int i = 0; i < 4; i++)
    {
        out[i] = 0.0;
    }
}

void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;

    // Ultra-simple control Jacobian for debugging
    if (!out || !vec)
    {
        if (out)
        {
            out[0] = 0.0;
            out[1] = 0.0;
        }
        return;
    }

    // df/du for simple dynamics: f = [v/2, 0, u[1]/10, u[0] - v]
    // df/du = [[0, 0], [0, 0], [0, 1/10], [1, 0]]
    out[0] = vec[3];       // d/d(v_cmd) of v_dot = 1
    out[1] = vec[2] * 0.1; // d/d(steering) of theta_dot = 1/10
}

void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)vec;
    (void)u;
    (void)p;
    (void)userparam;
    // No parameters
    (void)out;
}

void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t;
    (void)p;
    (void)udes;
    (void)userparam;

    // Ultra-simple quadratic cost for debugging
    if (!out || !x || !u || !xdes)
    {
        if (out)
            out[0] = 0.0;
        return;
    }

    // Simple tracking cost: ||x - xdes||^2 + ||u||^2
    double cost = 0.0;
    for (int i = 0; i < 4; i++)
    {
        cost += (x[i] - xdes[i]) * (x[i] - xdes[i]);
    }
    for (int i = 0; i < 2; i++)
    {
        cost += 0.1 * u[i] * u[i];
    }
    out[0] = cost;
}

void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t;
    (void)u;
    (void)p;
    (void)udes;
    (void)userparam;

    // Ultra-simple gradients for debugging
    if (!out || !x || !xdes)
    {
        if (out)
        {
            for (int i = 0; i < 4; i++)
                out[i] = 0.0;
        }
        return;
    }

    // dJ/dx = 2*(x - xdes)
    for (int i = 0; i < 4; i++)
    {
        out[i] = 2.0 * (x[i] - xdes[i]);
    }
}

void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)p;
    (void)xdes;
    (void)udes;
    (void)userparam;

    // Ultra-simple control gradients for debugging
    if (!out || !u)
    {
        if (out)
        {
            out[0] = 0.0;
            out[1] = 0.0;
        }
        return;
    }

    // dJ/du = 2*0.1*u (simple quadratic control penalty)
    out[0] = 2.0 * 0.1 * u[0];
    out[1] = 2.0 * 0.1 * u[1];
}

void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)xdes;
    (void)udes;
    (void)userparam;
    (void)out;
}

void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T;
    (void)p;
    (void)userparam;

    // Ultra-simple terminal cost for debugging
    if (!out || !x || !xdes)
    {
        if (out)
            out[0] = 0.0;
        return;
    }

    // Simple terminal cost: ||x - xdes||^2
    double cost = 0.0;
    for (int i = 0; i < 4; i++)
    {
        cost += (x[i] - xdes[i]) * (x[i] - xdes[i]);
    }
    out[0] = cost;
}

void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T;
    (void)p;
    (void)userparam;

    // Ultra-simple terminal gradients for debugging
    if (!out || !x || !xdes)
    {
        if (out)
        {
            for (int i = 0; i < 4; i++)
                out[i] = 0.0;
        }
        return;
    }

    // dV/dx = 2*(x - xdes)
    for (int i = 0; i < 4; i++)
    {
        out[i] = 2.0 * (x[i] - xdes[i]);
    }
}

void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)xdes;
    (void)userparam;
    (void)out;
}

void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)xdes;
    (void)userparam;
    out[0] = 0.0;
}

void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)p;
    (void)u;

    mpcc_ctx_t *ctx = (mpcc_ctx_t *)userparam;

    // Safety checks
    if (!x || !out)
    {
        if (out)
        {
            out[0] = 0.0;
        }
        return;
    }

    // Vehicle parameters with defaults
    double v_max = (ctx ? ctx->v_max : 5.0);

    // State variables
    double v = x[3]; // velocity

    // Single simple constraint: Maximum velocity
    out[0] = v_max - v; // v <= v_max (g >= 0 means constraint satisfied)
}

void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;

    // Safety checks
    if (!out || !vec)
    {
        if (out)
        {
            for (int i = 0; i < 4; ++i)
                out[i] = 0.0;
        }
        return;
    }

    // Gradient of single constraint g = v_max - v w.r.t. state x, multiplied by vec
    // ∂g/∂v = -1, all other gradients are 0

    out[0] = 0.0;             // ∂g/∂x
    out[1] = 0.0;             // ∂g/∂y
    out[2] = 0.0;             // ∂g/∂θ
    out[3] = vec[0] * (-1.0); // ∂g/∂v = -1
}

void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;

    // Safety checks
    if (!out || !vec)
    {
        if (out)
        {
            out[0] = out[1] = 0.0;
        }
        return;
    }

    // Single constraint g = v_max - v doesn't depend on control inputs
    out[0] = 0.0; // No dependence on velocity command u[0]
    out[1] = 0.0; // No dependence on steering angle u[1]
}

void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;
    (void)out;
}

void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)userparam;
    (void)out;
    // No terminal constraints for now
}

void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)userparam;

    // Safety checks
    if (!out || !vec)
    {
        if (out)
        {
            for (int i = 0; i < 4; ++i)
                out[i] = 0.0;
        }
        return;
    }

    // Gradient of terminal constraint gT w.r.t. state x, multiplied by vec
    // gT = v_max - v, so ∂gT/∂v = -1

    out[0] = 0.0;             // ∂gT/∂x
    out[1] = 0.0;             // ∂gT/∂y
    out[2] = 0.0;             // ∂gT/∂θ
    out[3] = vec[0] * (-1.0); // ∂gT/∂v = -1
}

void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)userparam;
    (void)out;
}

void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

// RODAS-related stubs (not used)
void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;
    for (int i = 0; i < 16; ++i)
        out[i] = 0.0;
}
void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;
    for (int i = 0; i < 16; ++i)
        out[i] = 0.0;
}
void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;
    for (int i = 0; i < 4; ++i)
        out[i] = 0.0;
}
void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *adj, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)adj;
    (void)p;
    (void)userparam;
    for (int i = 0; i < 4; ++i)
        out[i] = 0.0;
}
void Mfct(typeRNum *out, typeUSERPARAM *userparam)
{
    (void)userparam;
    (void)out;
}
void Mtrans(typeRNum *out, typeUSERPARAM *userparam)
{
    (void)userparam;
    (void)out;
}