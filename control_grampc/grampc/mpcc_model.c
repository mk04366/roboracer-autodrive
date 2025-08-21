#include "control_grampc/mpcc_model.h"
#include <math.h>
#include <stddef.h>

static inline double sq(double a) { return a * a; }
static inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline double wrap_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void mpcc_dynamics(double t, const double *x, const double *u, double *xdot, void *user)
{
    (void)t;
    mpcc_ctx_t *C = (mpcc_ctx_t *)user;
    // Defaults if no user context provided
    double L = (C ? C->L : 0.33);
    double delta_max = (C ? C->delta_max : 0.5);
    double a_min = (C ? C->a_min : -3.0);
    double a_max = (C ? C->a_max : 3.0);

    double TH = x[2], V = x[3];
    double delta = clamp(u[0], -delta_max, delta_max);
    double acc = clamp(u[1], a_min, a_max);
    xdot[0] = V * cos(TH);
    xdot[1] = V * sin(TH);
    xdot[2] = (V / L) * tan(delta);
    xdot[3] = acc;
    // No progress state in Nx; path progress handled in node
}

double mpcc_stage_cost(double t, const double *x, const double *u, const double *xdes, void *user)
{
    (void)t;
    mpcc_ctx_t *C = (mpcc_ctx_t *)user;
    // Defaults if no user context provided
    double w_xy = (C ? C->w_xy : 1.0);
    double w_yaw = (C ? C->w_yaw : 0.5);
    // Encourage slower speeds modestly
    double w_v = (C ? C->w_v : 0.15);
    // Control effort penalty
    double w_u = (C ? C->w_u : 0.02);

    double xr = xdes ? xdes[0] : 0.0;
    double yr = xdes ? xdes[1] : 0.0;
    double thr = xdes ? xdes[2] : 0.0;
    double vr = xdes ? xdes[3] : 0.0;

    double dx = x[0] - xr;
    double dy = x[1] - yr;
    double dyaw = wrap_angle(x[2] - thr);

    double delta = u[0];
    double a = u[1];
    double du_delta = C ? (delta - C->u_prev[0]) : 0.0;
    double du_a = C ? (a - C->u_prev[1]) : 0.0;

    double J = 0.0;
    J += w_xy * (sq(dx) + sq(dy));
    J += w_yaw * sq(dyaw);
    J += w_v * sq(x[3] - vr);
    J += w_u * (sq(delta) + sq(a));
    if (C) {
        J += C->w_du * (sq(du_delta) + sq(du_a));
    }
    return J;
}

double mpcc_terminal_cost(const double *x, const double *xdes, void *user)
{
    (void)user;
    double xr = xdes ? xdes[0] : 0.0;
    double yr = xdes ? xdes[1] : 0.0;
    double thr = xdes ? xdes[2] : 0.0;
    double dx = x[0] - xr;
    double dy = x[1] - yr;
    double dyaw = wrap_angle(x[2] - thr);
    double w_term = 1.0;
    return w_term * (sq(dx) + sq(dy) + 0.5 * sq(dyaw));
}

// ===== GRAMPC v2.2 required callbacks (signatures per probfct.h) =====

void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    (void)userparam;
    *Nx = 4;  // x, y, yaw, v
    *Nu = 2;  // steer, throttle
    *Np = 0;
    *Ng = 0; *Nh = 0; *NgT = 0; *NhT = 0;
}

void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)p;
    mpcc_dynamics(t, x, u, out, userparam);
}

void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)vec; (void)u; (void)p; (void)userparam;
    // Provide zero Jacobian-vector product as a stub
    for (int i = 0; i < 4; ++i) out[i] = 0.0;
}

void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)vec; (void)u; (void)p; (void)userparam;
    for (int i = 0; i < 2; ++i) out[i] = 0.0;
}

void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)vec; (void)u; (void)p; (void)userparam;
    // No parameters
    (void)out;
}

void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)p; (void)udes;
    out[0] = mpcc_stage_cost(t, x, u, xdes, userparam);
}

void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)xdes; (void)udes; (void)userparam;
    for (int i = 0; i < 4; ++i) out[i] = 0.0;
}

void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)xdes; (void)udes; (void)userparam;
    for (int i = 0; i < 2; ++i) out[i] = 0.0;
}

void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)xdes; (void)udes; (void)userparam; (void)out;
}

void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T; (void)p;
    out[0] = mpcc_terminal_cost(x, xdes, userparam);
}

void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)xdes; (void)userparam;
    for (int i = 0; i < 4; ++i) out[i] = 0.0;
}

void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)xdes; (void)userparam; (void)out;
}

void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)xdes; (void)userparam;
    out[0] = 0.0;
}

void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)userparam; (void)out;
}

void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)vec; (void)userparam; (void)out;
}

void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)vec; (void)userparam; (void)out;
}

void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)vec; (void)userparam; (void)out;
}

void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)userparam; (void)out;
}

void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)vec; (void)userparam; (void)out;
}

void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)vec; (void)userparam; (void)out;
}

void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)vec; (void)userparam; (void)out;
}

void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)userparam; (void)out;
}

void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)vec; (void)userparam; (void)out;
}

void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)vec; (void)userparam; (void)out;
}

void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)vec; (void)userparam; (void)out;
}

void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)userparam; (void)out;
}

void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)vec; (void)userparam; (void)out;
}

void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)vec; (void)userparam; (void)out;
}

void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    (void)T; (void)x; (void)p; (void)vec; (void)userparam; (void)out;
}

// RODAS-related stubs (not used)
void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)userparam; for (int i = 0; i < 16; ++i) out[i] = 0.0;
}
void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)userparam; for (int i = 0; i < 16; ++i) out[i] = 0.0;
}
void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)p; (void)userparam; for (int i = 0; i < 4; ++i) out[i] = 0.0;
}
void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *adj, ctypeRNum *p, typeUSERPARAM *userparam)
{
    (void)t; (void)x; (void)u; (void)adj; (void)p; (void)userparam; for (int i = 0; i < 4; ++i) out[i] = 0.0;
}
void Mfct(typeRNum *out, typeUSERPARAM *userparam)
{
    (void)userparam; (void)out;
}
void Mtrans(typeRNum *out, typeUSERPARAM *userparam)
{
    (void)userparam; (void)out;
}