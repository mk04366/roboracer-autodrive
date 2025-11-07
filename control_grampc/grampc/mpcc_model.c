#include "control_grampc/mpcc_model.h"
#include <stdio.h>
#if USE_typeRNum == USE_FLOAT
#define SIN(a) sinf(a)
#define COS(a) cosf(a)
#else
#define SIN(a) sin(a)
#define COS(a) cos(a)
#endif
#define DT 0.05
/* square macro */
#define POW2(a) ((a) * (a))

static void get_reference_at_time(const double *t_array,
                                  const double *x_ref,
                                  const double *y_ref,
                                  const double *theta_ref,
                                  const double *kappa_ref,
                                  const double *v_ref,
                                  int N,
                                  double t,
                                  double *x_out,
                                  double *y_out,
                                  double *theta_out,
                                  double *kappa_out,
                                  double *v_out)
{
    // Round current time to nearest 20 Hz step
    double t_currstep = round(t / DT) * DT;

    // Wrap around if time exceeds trajectory duration
    double t_max = t_array[N - 1];
    if (t_currstep > t_max)
        t_currstep = fmod(t_currstep, t_max);

    // 🔍 Find index in time array that matches t_currstep
    int idx = 0;
    for (int i = 0; i < N - 1; ++i)
    {
        // If t_currstep falls between t_array[i] and t_array[i+1], choose the closer one
        if (t_array[i] <= t_currstep && t_currstep < t_array[i + 1])
        {
            double d1 = fabs(t_currstep - t_array[i]);
            double d2 = fabs(t_currstep - t_array[i + 1]);
            idx = (d1 < d2) ? i : (i + 1);
            break;
        }
    }

    // Clamp to valid range
    if (idx >= N)
        idx = N - 1;

    // Output corresponding reference values
    *x_out = x_ref[idx];
    *y_out = y_ref[idx];
    *theta_out = theta_ref[idx];
    *kappa_out = kappa_ref[idx];
    *v_out = v_ref[idx];
}

/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    *Nx = 5; // x, y, theta, kappa, v
    *Nu = 2; // acceleration, steering_rate
    *Np = 0;
    *Nh = 0;
    *Ng = 0;
    *NgT = 0;
    *NhT = 0;
}

/** System Dynamics function f(t,x,u,p,userparam)
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{

    out[0] = COS(x[2]) * x[4]; // cos(theta) * v
    out[1] = SIN(x[2]) * x[4]; // sin(theta) * v
    out[2] = x[3] * x[4];      // kappa * v
    out[3] = u[0];             // kappa_dot
    out[4] = u[1];             // v_dot/acceleration
}

/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x,
              ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p,
              typeUSERPARAM *userparam)

{

    /*
Based on the matrix A=∂f/∂x: ​

|    | x | y | θ         | κ | v      |
| -- | - | - | --------- | - | ------ |
| ẋ  | 0 | 0 | −v sin(θ) | 0 | cos(θ) |
| ẏ  | 0 | 0 | v cos(θ)  | 0 | sin(θ) |
| θ̇  | 0 | 0 | 0         | v | κappa  |
| κ̇  | 0 | 0 | 0         | 0 | 0      |
| v̇  | 0 | 0 | 0         | 0 | 0      |


The equations below are a vector of 5x1 (out) i.e., A*v
*/
    out[0] = 0;
    out[1] = 0;
    out[2] = (vec[1] * COS(x[2]) - vec[0] * SIN(x[2])) * x[4];
    out[3] = vec[2] * x[4];
    out[4] = vec[0] * COS(x[2]) + vec[1] * SIN(x[2]) + vec[2] * x[3];
}

/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{

    /*
    | f component           | derivative wrt (u_0) | derivative wrt (u_1) |
    | --------------------- | -------------------- | -------------------- |
    | (f_0 = v\cos(\theta)) | 0                    | 0                    |
    | (f_1 = v\sin(\theta)) | 0                    | 0                    |
    | (f_2 = v\kappa)       | 0                    | 0                    |
    | (f_3 = u_0)           | 1                    | 0                    |
    | (f_4 = u_1)           | 0                    | 1                    |

    */
    out[0] = vec[3]; // derivative wrt u₀ (curvature rate) -> affects κ̇ = f₃
    out[1] = vec[4]; // derivative wrt u₁ (acceleration)   -> affects v̇ = f₄
}

/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}

/** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam)
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{

    double x_ref_t, y_ref_t, theta_ref_t, kappa_ref_t, v_ref_t;

    get_reference_at_time(userparam->t,
                          userparam->x,
                          userparam->y,
                          userparam->theta,
                          userparam->kappa,
                          userparam->v,
                          userparam->N,
                          userparam->current_time + t, // I am doing this here since the value t here is horizon time
                          &x_ref_t, &y_ref_t, &theta_ref_t,
                          &kappa_ref_t, &v_ref_t);

    out[0] =
        userparam->R[0] * POW2(u[0] - udes[0]) +     // control effort weight for u₀ = κ̇
        userparam->R[1] * POW2(u[1] - udes[1]) +     // control effort weight for u₁ = v̇
        userparam->Q[0] * POW2(x[0] - x_ref_t) +     // position x error weight
        userparam->Q[1] * POW2(x[1] - y_ref_t) +     // position y error weight
        userparam->Q[2] * POW2(x[2] - theta_ref_t) + // heading error weight
        userparam->Q[3] * POW2(x[3] - kappa_ref_t) + // curvature error weight
        userparam->Q[4] * POW2(x[4] - v_ref_t);      // velocity error weight
}

/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{

    double x_ref_t, y_ref_t, theta_ref_t, kappa_ref_t, v_ref_t;

    get_reference_at_time(userparam->t,
                          userparam->x,
                          userparam->y,
                          userparam->theta,
                          userparam->kappa,
                          userparam->v,
                          userparam->N,
                          userparam->current_time + t, // I am doing this here since the value t here is horizon time rather system time
                          &x_ref_t, &y_ref_t, &theta_ref_t,
                          &kappa_ref_t, &v_ref_t);

    // gradient of the stage cost w.r.t. state x:
    // dl/dx = 2Q(x - x_{des})
    out[0] = 2 * userparam->Q[0] * (x[0] - x_ref_t);
    out[1] = 2 * userparam->Q[1] * (x[1] - y_ref_t);
    out[2] = 2 * userparam->Q[2] * (x[2] - theta_ref_t);
    out[3] = 2 * userparam->Q[3] * (x[3] - kappa_ref_t);
    out[4] = 2 * userparam->Q[4] * (x[4] - v_ref_t);
}

/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{

    // gradient of the stage cost w.r.t. state u:
    // dl/dx = 2R(u - u_{des})
    out[0] = 2 * userparam->R[0] * (u[0] - udes[0]);
    out[1] = 2 * userparam->R[1] * (u[1] - udes[1]);
}

/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
}

/** Terminal cost V(T,x(T),p,xdes,userparam)
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    double x_ref_t, y_ref_t, theta_ref_t, kappa_ref_t, v_ref_t;

    get_reference_at_time(userparam->t,
                          userparam->x,
                          userparam->y,
                          userparam->theta,
                          userparam->kappa,
                          userparam->v,
                          userparam->N,
                          userparam->current_time + T, // I am doing this here since the value t here is horizon time rather system time
                          &x_ref_t, &y_ref_t, &theta_ref_t,
                          &kappa_ref_t, &v_ref_t);

    out[0] = userparam->P[0] * POW2(x[0] - x_ref_t) +
             userparam->P[1] * POW2(x[1] - y_ref_t) +
             userparam->P[2] * POW2(x[2] - theta_ref_t) +
             userparam->P[3] * POW2(x[3] - kappa_ref_t) +
             userparam->P[4] * POW2(x[4] - v_ref_t);
}

/** Gradient dV/dx : Terminal Cost Function V(x(T))**/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    double x_ref_t, y_ref_t, theta_ref_t, kappa_ref_t, v_ref_t;

    get_reference_at_time(userparam->t,
                          userparam->x,
                          userparam->y,
                          userparam->theta,
                          userparam->kappa,
                          userparam->v,
                          userparam->N,
                          userparam->current_time + T, // I am doing this here since the value t here is horizon time rather system time
                          &x_ref_t, &y_ref_t, &theta_ref_t,
                          &kappa_ref_t, &v_ref_t);

    // V(x(T)) = (x(T) - x_des) * P * (x(T) - x_des)
    out[0] = 2 * userparam->P[0] * (x[0] - x_ref_t);
    out[1] = 2 * userparam->P[1] * (x[1] - y_ref_t);
    out[2] = 2 * userparam->P[2] * (x[2] - theta_ref_t);
    out[3] = 2 * userparam->P[3] * (x[3] - kappa_ref_t);
    out[4] = 2 * userparam->P[4] * (x[4] - v_ref_t);
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
}
/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}

/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
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