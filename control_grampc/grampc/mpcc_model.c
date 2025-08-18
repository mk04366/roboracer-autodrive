#include "control_grampc/mpcc_model.h"
#include <math.h>
#include <stddef.h>

/*
 * Minimal user-data bundle (filled from C++ node via pointer).
 * You don't need to expose this in the header; just ensure your C++ node
 * fills the exact layout before calling GRAMPC.
 */
typedef struct
{
    // vehicle / limits
    double L;         // wheelbase [m]
    double delta_max; // max steering [rad]
    double a_min;     // min accel  [m/s^2] (set 0 to forbid reverse)
    double a_max;     // max accel  [m/s^2]
    double v_max;     // max speed  [m/s]
    double a_lat_max; // max lateral accel [m/s^2]

    // cost weights
    double w_c;        // contour error
    double w_l;        // lag error
    double w_v;        // speed tracking
    double w_u;        // input magnitude
    double w_du;       // input rate (vs previous)
    double w_term;     // terminal error
    double kappa_gain; // curvature-based speed reduction

    // reference interpolation callback provided by C++:
    // given s, write xr, yr, thr, kappa using ref_ctx
    void (*ref_interp)(double s, double *xr, double *yr, double *thr, double *kappa, void *ref_ctx);
    void *ref_ctx;

    // previous control for rate penalty (updated by caller each MPC step)
    double u_prev[2];
} mpcc_ctx_t;

static inline double sq(double a) { return a * a; }
static inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline double wrap(double a)
{
    while (a > M_PI)
        a -= 2 * M_PI;
    while (a < -M_PI)
        a += 2 * M_PI;
    return a;
}

/*
 * Kinematic bicycle dynamics (rear-axle frame) with a progress state s:
 *   xdot = v cos(theta)
 *   ydot = v sin(theta)
 *   thetadot = (v/L) tan(delta)
 *   vdot = a
 *   sdot = v
 */
void mpcc_dynamics(double t, const double *x, const double *u, double *xdot, void *user)
{
    (void)t;
    mpcc_ctx_t *C = (mpcc_ctx_t *)user;

    const double X = x[0];
    const double Y = x[1];
    const double TH = x[2];
    const double V = x[3];
    (void)X;
    (void)Y; // not used directly in dynamics

    // clamp inputs to physical limits
    const double delta = clamp(u[0], -C->delta_max, C->delta_max);
    const double acc = clamp(u[1], C->a_min, C->a_max);

    xdot[0] = V * cos(TH);
    xdot[1] = V * sin(TH);
    xdot[2] = (V / C->L) * tan(delta);
    xdot[3] = acc;
    xdot[4] = V; // progress along path
}

/*
 * Running (stage) cost:
 *   - contour/lag errors relative to path at arc-length s
 *   - speed tracking to curvature-limited reference
 *   - input magnitude
 *   - input rate vs previous input (softness)
 */
double mpcc_stage_cost(double t, const double *x, const double *u, void *user)
{
    (void)t;
    mpcc_ctx_t *C = (mpcc_ctx_t *)user;

    // reference at current s
    double xr, yr, thr, kappa;
    C->ref_interp(x[4], &xr, &yr, &thr, &kappa, C->ref_ctx);

    const double dx = x[0] - xr;
    const double dy = x[1] - yr;
    const double ct = cos(thr);
    const double st = sin(thr);

    // Frenet-frame errors
    const double e_l = ct * dx + st * dy;  // lag
    const double e_c = -st * dx + ct * dy; // contour

    // curvature-based speed limit
    double v_lim = C->v_max;
    const double absk = fabs(kappa);
    if (absk > 1e-6)
    {
        const double v_curve = sqrt(C->a_lat_max / absk);
        if (v_curve < v_lim)
            v_lim = v_curve;
    }
    const double v_ref = v_lim / (1.0 + C->kappa_gain * absk);

    // clamped inputs
    const double delta = clamp(u[0], -C->delta_max, C->delta_max);
    const double a = clamp(u[1], C->a_min, C->a_max);

    double J = 0.0;
    J += C->w_c * sq(e_c) + C->w_l * sq(e_l);
    J += C->w_v * sq(x[3] - v_ref);
    J += C->w_u * (sq(delta) + sq(a));
    J += C->w_du * (sq(delta - C->u_prev[0]) + sq(a - C->u_prev[1]));
    return J;
}

/*
 * Terminal cost (stronger alignment near the end of the horizon).
 */
double mpcc_terminal_cost(const double *x, void *user)
{
    mpcc_ctx_t *C = (mpcc_ctx_t *)user;

    double xr, yr, thr, kappa;
    C->ref_interp(x[4], &xr, &yr, &thr, &kappa, C->ref_ctx);

    const double dx = x[0] - xr;
    const double dy = x[1] - yr;
    const double ct = cos(thr);
    const double st = sin(thr);

    const double e_l = ct * dx + st * dy;
    const double e_c = -st * dx + ct * dy;

    return C->w_term * (sq(e_c) + sq(e_l));
}
