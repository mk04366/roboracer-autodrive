#include "control_grampc/mpcc_model.h"
#include <math.h>
#include <stddef.h>

static inline double sq(double a) { return a * a; }
static inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline double wrap_angle(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

void mpcc_dynamics(double t, const double *x, const double *u, double *xdot, void *user)
{
    (void)t;
    mpcc_ctx_t *C = (mpcc_ctx_t *)user;
    // Defaults if no user context provided
    double L = (C ? C->L : 0.33);
    double delta_max = (C ? C->delta_max : 0.4);
    double v_cmd_max = (C ? C->v_max : 5.0);

    // 4D state: [x, y, theta, v]
    double X = x[0], Y = x[1], THETA = x[2], V = x[3];
    double v_cmd = clamp(u[0], 0.0, v_cmd_max);        // velocity command
    double delta = clamp(u[1], -delta_max, delta_max); // steering angle

    // Bicycle model dynamics with velocity control
    xdot[0] = V * cos(THETA);       // x_dot
    xdot[1] = V * sin(THETA);       // y_dot
    xdot[2] = (V / L) * tan(delta); // theta_dot
    xdot[3] = (v_cmd - V) / 0.1;    // v_dot (first-order velocity tracking)
}

double mpcc_stage_cost(double t, const double *x, const double *u, const double *xdes, void *user)
{
    (void)t;
    mpcc_ctx_t *C = (mpcc_ctx_t *)user;

    // MPC tracking weights
    double w_x = 10.0;    // x position tracking
    double w_y = 10.0;    // y position tracking
    double w_theta = 1.0; // heading tracking
    double w_v = 1.0;     // velocity tracking
    double w_u = 0.1;     // control effort

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

    // MPC tracking cost
    double J = 0.0;

    // Position tracking
    J += w_x * sq(vehicle_x - ref_x);
    J += w_y * sq(vehicle_y - ref_y);

    // Heading tracking
    J += w_theta * sq(wrap_angle(vehicle_theta - ref_theta));

    // Velocity tracking
    J += w_v * sq(vehicle_v - ref_v);

    // Control effort
    J += w_u * (sq(v_cmd - ref_v) + sq(delta));

    return J;
}

double mpcc_terminal_cost(const double *x, const double *xdes, void *user)
{
    (void)user;

    // Terminal cost: track reference state more strongly
    double ref_x = xdes ? xdes[0] : 0.0;
    double ref_y = xdes ? xdes[1] : 0.0;
    double ref_theta = xdes ? xdes[2] : 0.0;
    double ref_v = xdes ? xdes[3] : 1.0;

    double dx = x[0] - ref_x;
    double dy = x[1] - ref_y;
    double dtheta = wrap_angle(x[2] - ref_theta);
    double dv = x[3] - ref_v;

    return 5.0 * (sq(dx) + sq(dy) + 0.5 * sq(dtheta) + 0.1 * sq(dv));
}

// ===== GRAMPC v2.2 required callbacks (signatures per probfct.h) =====

void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    (void)userparam;
    *Nx = 4; // x, y, theta, v
    *Nu = 2; // velocity_cmd, steering_angle
    *Np = 0;
    *Ng = 0;
    *Nh = 0;
    *NgT = 0;
    *NhT = 0;
}

void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)p;
    // Cast userparam to the correct type for mpcc_dynamics
    mpcc_ctx_t *ctx = (mpcc_ctx_t *)userparam;
    mpcc_dynamics(t, x, u, out, ctx);
}

void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)p;

    mpcc_ctx_t *ctx = (mpcc_ctx_t *)userparam;
    if (!ctx)
    {
        for (int i = 0; i < 4; ++i)
            out[i] = 0.0; // 4D state
        return;
    }

    double v = x[3];
    double theta = x[2];
    double delta = u[1]; // steering angle is second input

    // df/dx * vec for the 4D bicycle model
    // xdot = v*cos(theta), ydot = v*sin(theta), thetadot = (v/L)*tan(delta), vdot = (v_cmd - v)/tau
    out[0] = 0.0;                                                                                       // d/dx (no direct dependency)
    out[1] = 0.0;                                                                                       // d/dy (no direct dependency)
    out[2] = -v * sin(theta) * vec[0] + v * cos(theta) * vec[1];                                        // d/dtheta of xdot and ydot
    out[3] = cos(theta) * vec[0] + sin(theta) * vec[1] + (tan(delta) / ctx->L) * vec[2] - vec[3] / 0.1; // d/dv
}

void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t;
    (void)p;

    mpcc_ctx_t *ctx = (mpcc_ctx_t *)userparam;
    if (!ctx)
    {
        for (int i = 0; i < 2; ++i)
            out[i] = 0.0;
        return;
    }

    double v = x[3];
    double delta = u[1]; // steering angle is second input

    // df/du * vec for the bicycle model with velocity control
    out[0] = vec[3] / 0.1;                                      // d/d(v_cmd) of vdot
    out[1] = (v / (ctx->L * cos(delta) * cos(delta))) * vec[2]; // d/d(delta) of thetadot
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
    (void)p;
    (void)udes;
    // Cast userparam to the correct type for mpcc_stage_cost
    mpcc_ctx_t *ctx = (mpcc_ctx_t *)userparam;
    out[0] = mpcc_stage_cost(t, x, u, xdes, ctx);
}

void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t;
    (void)p;
    (void)udes;
    (void)userparam;

    // MPC tracking weights
    double w_x = 10.0;
    double w_y = 10.0;
    double w_theta = 1.0;
    double w_v = 1.0;

    // Reference values
    double ref_x = xdes ? xdes[0] : 0.0;
    double ref_y = xdes ? xdes[1] : 0.0;
    double ref_theta = xdes ? xdes[2] : 0.0;
    double ref_v = xdes ? xdes[3] : 1.0;

    // Gradients of MPC tracking cost
    out[0] = 2.0 * w_x * (x[0] - ref_x);                   // d/dx
    out[1] = 2.0 * w_y * (x[1] - ref_y);                   // d/dy
    out[2] = 2.0 * w_theta * wrap_angle(x[2] - ref_theta); // d/dtheta
    out[3] = 2.0 * w_v * (x[3] - ref_v);                   // d/dv
}
double w_v = 1.0;

void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)p;
    (void)udes;
    (void)userparam;

    double w_u = 0.1; // Control effort weight

    // Reference velocity for control effort calculation
    double ref_v = xdes ? xdes[3] : 1.0;

    // Control effort penalty gradients
    out[0] = 2.0 * w_u * (u[0] - ref_v); // d/d(v_cmd) - penalize deviation from reference velocity
    out[1] = 2.0 * w_u * u[1];           // d/d(delta) - penalize steering effort
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
    // Cast userparam to the correct type for mpcc_terminal_cost
    mpcc_ctx_t *ctx = (mpcc_ctx_t *)userparam;
    out[0] = mpcc_terminal_cost(x, xdes, ctx);
}

void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T;
    (void)p;
    (void)userparam;

    // Terminal cost gradients for 4D state
    double ref_x = xdes ? xdes[0] : 0.0;
    double ref_y = xdes ? xdes[1] : 0.0;
    double ref_theta = xdes ? xdes[2] : 0.0;
    double ref_v = xdes ? xdes[3] : 1.0;

    out[0] = 2.0 * 5.0 * (x[0] - ref_x);                     // d/dx
    out[1] = 2.0 * 5.0 * (x[1] - ref_y);                     // d/dy
    out[2] = 2.0 * 5.0 * 0.5 * wrap_angle(x[2] - ref_theta); // d/dtheta
    out[3] = 2.0 * 5.0 * 0.1 * (x[3] - ref_v);               // d/dv
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
    (void)x;
    (void)u;
    (void)p;
    (void)userparam;
    (void)out;
}

void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
}

void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t;
    (void)x;
    (void)u;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
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
}

void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T;
    (void)x;
    (void)p;
    (void)vec;
    (void)userparam;
    (void)out;
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