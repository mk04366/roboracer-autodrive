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
    *Nh = 0;
    *Ng = 0;
    *NgT = 0;
    *NhT = 0;
}

/** System Dynamics function f(t,x,u,p,userparam)
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;

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
    ctypeRNum *param = (ctypeRNum *)userparam;

    out[0] =
        param[12] * POW2(u[0] - udes[0]) + // control effort weight for u₀ = κ̇
        param[13] * POW2(u[1] - udes[1]) + // control effort weight for u₁ = v̇
        param[2] * POW2(x[0] - xdes[0]) +  // position x error weight
        param[3] * POW2(x[1] - xdes[1]) +  // position y error weight
        param[4] * POW2(x[2] - xdes[2]) +  // heading error weight
        param[5] * POW2(x[3] - xdes[3]) +  // curvature error weight
        param[6] * POW2(x[4] - xdes[4]);   // velocity error weight
}

/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;

    // gradient of the stage cost w.r.t. state x:
    // dl/dx = 2Q(x - x_{des})
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

    // gradient of the stage cost w.r.t. state u:
    // dl/dx = 2R(u - u_{des})
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

    out[0] =
        param[7] * POW2(x[0] - xdes[0]) +
        param[8] * POW2(x[1] - xdes[1]) +
        param[9] * POW2(x[2] - xdes[2]) +
        param[10] * POW2(x[3] - xdes[3]) +
        param[11] * POW2(x[4] - xdes[4]);
}

/** Gradient dV/dx : Terminal Cost Function V(x(T))**/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    ctypeRNum *param = (ctypeRNum *)userparam;

    // V(x(T)) = (x(T) - x_des) * P * (x(T) - x_des)
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