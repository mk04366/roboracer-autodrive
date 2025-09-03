#include "control_grampc/mpcc_model.h"

#if USE_typeRNum == USE_FLOAT
#define SIN(a) sinf(a)
#define COS(a) cosf(a)
#else
#define SIN(a) sin(a)
#define COS(a) cos(a)
#endif

/* square macro */
#define POW2(a) ((a) * (a))

/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    *Nx = 5; // x, y, theta, kappa, v
    *Nu = 2; // acceleration, steering_rate
    *Np = 0;
    *Nh = 3; // velocity and curvature bounds, plus one for steering constraint
    *Ng = 0;
    *NgT = 0;
    *NhT = 0;
}

/** System function f(t,x,u,p,userparam)
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;

    // Improved bicycle model with velocity-dependent dynamics
    ctypeRNum L = param[0];    // wheelbase
    ctypeRNum v_ch = param[1]; // characteristic velocity
    ctypeRNum v = x[4];        // current velocity
    ctypeRNum kappa = x[3];    // current curvature
    ctypeRNum theta = x[2];    // current heading

    // Velocity-dependent factor for more realistic dynamics
    ctypeRNum velocity_factor = 1.0 / (1.0 + POW2(v / v_ch));

    out[0] = v * COS(theta);                    // x_dot = v * cos(theta)
    out[1] = v * SIN(theta);                    // y_dot = v * sin(theta)
    out[2] = (kappa * v * velocity_factor) / L; // theta_dot = kappa * v / L (with velocity dependence)
    out[3] = u[1];                              // kappa_dot = steering_rate
    out[4] = u[0];                              // v_dot = acceleration
}

/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;
    ctypeRNum L = param[0];
    ctypeRNum v_ch = param[1];
    ctypeRNum v = x[4];
    ctypeRNum kappa = x[3];
    ctypeRNum theta = x[2];

    ctypeRNum velocity_factor = 1.0 / (1.0 + POW2(v / v_ch));
    ctypeRNum dvf_dv = -2.0 * v / (POW2(v_ch) * POW2(1.0 + POW2(v / v_ch)));

    out[0] = 0;
    out[1] = 0;
    out[2] = (vec[1] * COS(theta) - vec[0] * SIN(theta)) * v;
    out[3] = vec[2] * v * velocity_factor / L;
    out[4] = vec[0] * COS(theta) + vec[1] * SIN(theta) + vec[2] * (kappa * velocity_factor / L + kappa * v * dvf_dv / L);
}

/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    out[0] = vec[4]; // d/d(acceleration) of v_dot = 1
    out[1] = vec[3]; // d/d(steering_rate) of kappa_dot = 1
}

/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}

/** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam)
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;
    typeRNum velocity_penalty = 0.0;

    // Add penalty for low velocity to encourage forward motion
    if (x[4] < 0.5)
    {
        typeRNum vel_diff = 0.5 - x[4];
        velocity_penalty = 10.0 * vel_diff * vel_diff; // Strong penalty below 0.5 m/s
    }

    out[0] = param[12] * POW2(u[0] - udes[0])   // acceleration effort
             + param[13] * POW2(u[1] - udes[1]) // steering rate effort
             + param[2] * POW2(x[0] - xdes[0])  // x position tracking
             + param[3] * POW2(x[1] - xdes[1])  // y position tracking
             + param[4] * POW2(x[2] - xdes[2])  // heading tracking
             + param[5] * POW2(x[3] - xdes[3])  // curvature tracking
             + param[6] * POW2(x[4] - xdes[4])  // velocity tracking
             + velocity_penalty;                // penalty for low velocity
}

/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;

    out[0] = 2 * param[2] * (x[0] - xdes[0]);
    out[1] = 2 * param[3] * (x[1] - xdes[1]);
    out[2] = 2 * param[4] * (x[2] - xdes[2]);
    out[3] = 2 * param[5] * (x[3] - xdes[3]);
    out[4] = 2 * param[6] * (x[4] - xdes[4]);
}

/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;

    out[0] = 2 * param[12] * (u[0] - udes[0]);
    out[1] = 2 * param[13] * (u[1] - udes[1]);
}

/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
}

/** Terminal cost V(T,x(T),p,xdes,userparam)
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;

    out[0] = param[7] * POW2(x[0] - xdes[0])     // terminal x position
             + param[8] * POW2(x[1] - xdes[1])   // terminal y position
             + param[9] * POW2(x[2] - xdes[2])   // terminal heading
             + param[10] * POW2(x[3] - xdes[3])  // terminal curvature
             + param[11] * POW2(x[4] - xdes[4]); // terminal velocity
}

/** Gradient dV/dx **/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;

    out[0] = 2 * param[7] * (x[0] - xdes[0]);
    out[1] = 2 * param[8] * (x[1] - xdes[1]);
    out[2] = 2 * param[9] * (x[2] - xdes[2]);
    out[3] = 2 * param[10] * (x[3] - xdes[3]);
    out[4] = 2 * param[11] * (x[4] - xdes[4]);
}

/** Gradient dV/dp **/
void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}
/** Gradient dV/dT **/
void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    out[0] = 0;
}

/** Equality constraints g(t,x(t),u(t),p,uperparam) = 0
    --------------------------------------------------- **/
void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}

/** Inequality constraints h(t,x(t),u(t),p,uperparam) <= 0
    ------------------------------------------------------ **/
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;
    ctypeRNum L = param[0];

    // Velocity constraints: 0.1 <= v <= 5.0 m/s (minimum speed to prevent reverse)
    out[0] = 0.1 - x[4]; // 0.1 - v <= 0 => v >= 0.1 (minimum forward speed)
    out[1] = x[4] - 5.0; // v - 5.0 <= 0 => v <= 5.0

    // Curvature constraint: |kappa| <= 1/L * tan(π/6) (max steering angle π/6 rad = 30°)
    ctypeRNum max_kappa = tan(3.14159265359 / 6.0) / L;
    out[2] = x[3] * x[3] - max_kappa * max_kappa; // kappa^2 <= max_kappa^2
}
/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 2.0 * x[3] * vec[2]; // d/dkappa of kappa^2 constraint
    out[4] = -vec[0] + vec[1];    // d/dv of velocity constraints
}
/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    out[0] = 0; // No control dependence in constraints
    out[1] = 0;
}
/** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}

/** Terminal equality constraints gT(T,x(T),p,uperparam) = 0
    -------------------------------------------------------- **/
void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}

/** Terminal inequality constraints hT(T,x(T),p,uperparam) <= 0
    ----------------------------------------------------------- **/
void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dx multiplied by vector vec, i.e. (dhT/dx)^T*vec or vec^T*(dhT/dx) **/
void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dp multiplied by vector vec, i.e. (dhT/dp)^T*vec or vec^T*(dhT/dp) **/
void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dT multiplied by vector vec, i.e. (dhT/dT)^T*vec or vec^T*(dhT/dT) **/
void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}

/** Additional functions required for semi-implicit systems
    M*dx/dt(t) = f(t0+t,x(t),u(t),p) using the solver RODAS
    ------------------------------------------------------- **/
/** Jacobian df/dx in vector form (column-wise) **/
void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dx in vector form (column-wise) **/
void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dt **/
void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian d(dH/dx)/dt  **/
void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *vec, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mfct(typeRNum *out, typeUSERPARAM *userparam)
{
}
/** Transposed mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mtrans(typeRNum *out, typeUSERPARAM *userparam)
{
}