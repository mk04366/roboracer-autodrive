#include "control_grampc/mpcc_model.h"
#include <stdio.h>
#include <stdint.h>
#if USE_typeRNum == USE_FLOAT
#define SIN(a) sinf(a)
#define COS(a) cosf(a)
#define TAN(a) tanf(a)
#else
#define SIN(a) sin(a)
#define COS(a) cos(a)
#define TAN(a) tan(a)
#endif
/* square macro */
#define POW2(a) ((a) * (a))
#define M_PI 3.14159265358979323846
#define THOR 1.5
static double wrapAngle(double a)
{
    // wrap angle to [-pi, pi)
    while (a >= M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

/** Linear interpolation with angle-safe interpolation for psi */
static void get_reference_at_time(const double *t_array,
                                  const double *x_ref,
                                  const double *y_ref,
                                  const double *psi_ref,
                                  const double *delta_ref,
                                  const double *vx_ref,
                                  const double *vy_ref,
                                  const double *yaw_rate_ref,
                                  int N,
                                  double t,
                                  double *x_out,
                                  double *y_out,
                                  double *psi_out,
                                  double *delta_out,
                                  double *vx_out,
                                  double *vy_out,
                                  double *yaw_rate_out)
{

    double t_max = t_array[N - 1];
    // wrap t into [0, t_max)
    double t_wrapped = fmod(t, t_max);
    if (t_wrapped < 0.0)
        t_wrapped += t_max;

    // find index i such that t_array[i] <= t_wrapped < t_array[i+1]
    int i = 0;
    while (i < N - 1 && t_array[i + 1] <= t_wrapped)
        ++i;

    if (i >= N - 1)
        i = N - 2;

    double t0 = t_array[i];
    double t1 = t_array[i + 1];

    double alpha = (t_wrapped - t0) / (t1 - t0);

    // linear interpolation for x,y,delta,v
    *x_out = x_ref[i] + alpha * (x_ref[i + 1] - x_ref[i]);
    *y_out = y_ref[i] + alpha * (y_ref[i + 1] - y_ref[i]);
    *delta_out = delta_ref[i] + alpha * (delta_ref[i + 1] - delta_ref[i]);
    *vx_out = vx_ref[i] + alpha * (vx_ref[i + 1] - vx_ref[i]);
    *vy_out = vy_ref[i] + alpha * (vy_ref[i + 1] - vy_ref[i]);
    *yaw_rate_out = yaw_rate_ref[i] + alpha * (yaw_rate_ref[i + 1] - yaw_rate_ref[i]);
    *psi_out = psi_ref[i] + alpha * (psi_ref[i + 1] - psi_ref[i]);
    // fprintf(stderr, "t: %.4f, t_wrapped: %.4f, i: %d, t0: %.4f, t1: %.4f, alpha: %.4f\n", t, t_wrapped, i, t0, t1, alpha);
    // fprintf(stderr, "x_out: %.4f, y_out: %.4f, psi_out: %.4f, delta_out: %.4f, v_out: %.4f\n",
    //         *x_out, *y_out, *psi_out, *delta_out, *v_out);
}

/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    *Nx = 7; // [x, y, psi, delta(steering), vx, vy, r]
    *Nu = 2; // [delta_dot, a]
    *Np = 0;
    *Nh = 3; // keep steering bounds + lateral accel bound
    *Ng = 0;
    *NgT = 0;
    *NhT = 0;
}

/** System Dynamics function f(t,x,u,p,userparam)
    ------------------------------------ **/
/** dynamic bicycle ffct using linear tire model **/
/* Dynamic bicycle model ffct
   States: x[0]=x, x[1]=y, x[2]=psi, x[3]=delta, x[4]=vx, x[5]=vy, x[6]=r
   Controls: u[0]=delta_dot, u[1]=a (longitudinal accel, m/s^2)
   userparam: pSys[] should contain pointers to:
     [0] L, [1] m, [2] Iz, [3] a (front), [4] b (rear), [5] Cf, [6] Cr, [7] mu, [8] g, [9] Fz_front, [10] Fz_rear
     (adjust indexing if you store differently)
*/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    void **pSys = (void **)userparam;
    double L = *((double *)pSys[0]);
    double m = *((double *)pSys[23]);
    double Iz = *((double *)pSys[24]);
    double Cf = *((double *)pSys[25]); // cornering stiffness front
    double Cr = *((double *)pSys[26]); // cornering stiffness rear
    double a = *((double *)pSys[27]);  // distance from CG to front axle
    double b = *((double *)pSys[28]);  // distance from CG to rear axle
    double g = *((double *)pSys[29]);
    double mu = *((double *)pSys[30]);   // friction coef
    double Fz_f = (m * g * b) / (a + b); // optionally use passed normal loads
    double Fz_r = (m * g * a) / (a + b);

    /* states */
    double psi = x[2];
    double delta = x[3];
    double vx = x[4];
    double vy = x[5];
    double r = x[6];

    /* controls */
    double delta_dot = u[0];
    double a_long = u[1];

    /* avoid divide-by-zero when vx is very small */
    double vx_eps = (vx > 0.1) ? vx : 0.1;

    /* slip angles (small-angle approximations not required) */
    double alpha_f = delta - ((vy + a * r) / vx_eps);
    double alpha_r = -((vy - b * r) / vx_eps);

    /* linear tire model forces (with simple saturation) */
    double Fy_f = -Cf * alpha_f;
    double Fy_r = -Cr * alpha_r;

    /* friction saturation: limit lateral force by mu * Fz */
    double Fy_f_max = mu * Fz_f;
    double Fy_r_max = mu * Fz_r;
    if (Fy_f > Fy_f_max)
        Fy_f = Fy_f_max;
    if (Fy_f < -Fy_f_max)
        Fy_f = -Fy_f_max;
    if (Fy_r > Fy_r_max)
        Fy_r = Fy_r_max;
    if (Fy_r < -Fy_r_max)
        Fy_r = -Fy_r_max;

    /* equations */
    out[0] = vx * COS(psi) - vy * SIN(psi); /* x_dot */
    out[1] = vx * SIN(psi) + vy * COS(psi); /* y_dot */
    out[2] = r;                             /* psi_dot */
    out[3] = delta_dot;                     /* delta_dot (steering dynamics) */

    /* longitudinal dynamics: simple a_long - coupling term (can be refined) */
    out[4] = a_long + ((-(Fy_f * SIN(delta))) / m) - r * vy; /* vx_dot */

    /* lateral velocity dynamics */
    out[5] = (Fy_f * COS(delta) + Fy_r) / m + r * vx; /* vy_dot */

    /* yaw acceleration */
    out[6] = (a * Fy_f * COS(delta) - b * Fy_r) / Iz; /* r_dot */
}

/** Jacobian df/dx multiplied by vector vec: out = (df/dx)^T * vec **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec,
              const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    void **pSys = (void **)userparam;
    double m = *((double *)pSys[23]);
    double Iz = *((double *)pSys[24]);
    double Cf = *((double *)pSys[25]);
    double Cr = *((double *)pSys[26]);
    double a = *((double *)pSys[27]);
    double b = *((double *)pSys[28]);

    double psi = x[2];
    double delta = x[3];
    double vx = x[4];
    double vy = x[5];
    double r = x[6];

    double vx_eps = (vx > 0.1) ? vx : 0.1;

    // Slip angles
    double alpha_f = delta - (vy + a * r) / vx_eps;
    double alpha_r = -(vy - b * r) / vx_eps;

    // Lateral forces
    double Fy_f = -Cf * alpha_f;
    double Fy_r = -Cr * alpha_r;

    // initialize output
    for (int i = 0; i < 7; i++)
        out[i] = 0.0;

    // --------------------------
    // x_dot = vx*cos(psi) - vy*sin(psi)
    // y_dot = vx*sin(psi) + vy*cos(psi)
    // --------------------------
    out[2] += (-vx*sin(psi) - vy*cos(psi)) * vec[0]; // ∂x_dot/∂psi * vec[0]
    out[4] += cos(psi) * vec[0];                     // ∂x_dot/∂vx * vec[0]
    out[5] += -sin(psi) * vec[0];                    // ∂x_dot/∂vy * vec[0]

    out[2] += (vx*cos(psi) - vy*sin(psi)) * vec[1];  // ∂y_dot/∂psi * vec[1]
    out[4] += sin(psi) * vec[1];                     // ∂y_dot/∂vx * vec[1]
    out[5] += cos(psi) * vec[1];                     // ∂y_dot/∂vy * vec[1]

    // psi_dot = r
    out[6] += vec[2];

    // delta_dot = u0 → no x dependency

    // vx_dot = a_long - Fy_f*sin(delta)/m - r*vy
    double dFyf_ddelta = -Cf; // ∂Fy_f/∂delta
    double dFyf_dvy = -Cf * (-1.0 / vx_eps); // ∂Fy_f/∂vy
    double dFyf_dr = -Cf * (-a / vx_eps);    // ∂Fy_f/∂r

    out[3] += (-dFyf_ddelta * sin(delta) - Fy_f * cos(delta)) / m * vec[4];
    out[4] += (-dFyf_dvy * sin(delta)) / m * vec[4];
    out[5] += (-r) * vec[4] + (-dFyf_dvy * sin(delta)) / m * vec[4]; // lateral cross-term
    out[6] += (-dFyf_dr * sin(delta)) / m * vec[4];

    // vy_dot = (Fy_f*cos(delta) + Fy_r)/m + r*vx
    double dFyf_ddelta_vy = -Cf * cos(delta);
    double dFyf_dvy_vy = -Cf * (-1.0 / vx_eps);
    double dFyr_dvy = -Cr * (-1.0 / vx_eps);
    double dFyf_dr_vy = -Cf * (-a / vx_eps);
    double dFyr_dr = -Cr * (b / vx_eps);

    out[3] += (dFyf_ddelta_vy) / m * vec[5];
    out[4] += 0; // vy_dot wrt vx is only r? already included below
    out[5] += (dFyf_dvy_vy + dFyr_dvy) / m * vec[5];
    out[6] += (dFyf_dr_vy - dFyr_dr) / m * vec[5];
    out[4] += r * vec[5];  // cross-term
    out[5] += vx * vec[5]; // cross-term

    // r_dot = (a*Fy_f*cos(delta) - b*Fy_r)/Iz
    out[3] += (a * dFyf_ddelta - 0) / Iz * vec[6];  // only delta
    out[4] += 0;
    out[5] += (a * dFyf_dvy - b * dFyr_dvy) / Iz * vec[6];
    out[6] += (a * dFyf_dr - b * dFyr_dr) / Iz * vec[6];
}


/** Jacobian df/du multiplied by vector vec: out = (df/du)^T * vec **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p,
              ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    /* u0 = delta_dot  -> appears in f3
       u1 = a          -> appears in f4
       So (df/du)^T * vec = [vec[3]; vec[4]]
    */
    out[0] = vec[3]; // contribution from delta_dot to f3
    out[1] = vec[4]; // contribution from a to f4
}
/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam)
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{

    double x_ref_t, y_ref_t, psi_ref_t, delta_ref_t, vx_ref_t, vy_ref_t, yaw_rate_ref_t;
    void **pSys = (void **)userparam;
    ctypeRNum *xdes = param->xdes;
    ctypeRNum *udes = param->udes;
    double time_scale = (THOR - t) / THOR;

    get_reference_at_time((const double *)pSys[13],
                          (const double *)pSys[14],
                          (const double *)pSys[15],
                          (const double *)pSys[16],
                          (const double *)pSys[17],
                          (const double *)pSys[18],
                          (const double *)pSys[19],
                          (const double *)pSys[20],
                          (int)(*((double *)pSys[21])),
                          t + *((double *)pSys[22]),
                          &x_ref_t, &y_ref_t, &psi_ref_t,
                          &delta_ref_t, &vx_ref_t, &vy_ref_t, &yaw_rate_ref_t);

    out[0] =
        ((*((double *)pSys[11])) * POW2(u[0] - udes[0]) +    // control effort weight for u₀ = κ̇
         (*((double *)pSys[12])) * POW2(u[1] - udes[1]) +    // control effort weight for u₁ = v̇
         (*((double *)pSys[1])) * POW2(x[0] - x_ref_t) +     // position x error weight
         (*((double *)pSys[2])) * POW2(x[1] - y_ref_t) +     // position y error weight
         (*((double *)pSys[3])) * POW2(x[2] - psi_ref_t) +   // heading error weight
         (*((double *)pSys[4])) * POW2(x[3] - delta_ref_t) + // steering error weight
         (*((double *)pSys[5])) * POW2(x[4] - vx_ref_t) +      // velocity_x error weight
         (*((double *)pSys[6])) * POW2(x[5] - vy_ref_t) +      // velocity_y error weight
         (*((double *)pSys[7])) * POW2(x[6] - yaw_rate_ref_t)) // yaw_rate error weight
        * time_scale;                                        // time-varying scaling
}

/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{

    double x_ref_t, y_ref_t, psi_ref_t, delta_ref_t, vx_ref_t, vy_ref_t, yaw_rate_ref_t;
    void **pSys = (void **)userparam;
    double time_scale = (THOR - t) / THOR;

    get_reference_at_time((const double *)pSys[13],
                          (const double *)pSys[14],
                          (const double *)pSys[15],
                          (const double *)pSys[16],
                          (const double *)pSys[17],
                          (const double *)pSys[18],
                          (const double *)pSys[19],
                          (const double *)pSys[20],
                          (int)(*((double *)pSys[21])),
                          t + *((double *)pSys[22]),
                          &x_ref_t, &y_ref_t, &psi_ref_t,
                          &delta_ref_t, &vx_ref_t, &vy_ref_t, &yaw_rate_ref_t);

    // gradient of the stage cost w.r.t. state x:
    // dl/dx = 2Q(x - x_{des})
    out[0] = time_scale * 2 * (*((double *)pSys[1])) * (x[0] - x_ref_t);
    out[1] = time_scale * 2 * (*((double *)pSys[2])) * (x[1] - y_ref_t);
    out[2] = time_scale * 2 * (*((double *)pSys[3])) * (x[2] - psi_ref_t);
    out[3] = time_scale * 2 * (*((double *)pSys[4])) * (x[3] - delta_ref_t);
    out[4] = time_scale * 2 * (*((double *)pSys[5])) * (x[4] - vx_ref_t);
    out[5] = time_scale * 2 * (*((double *)pSys[6])) * (x[5] - vy_ref_t);
    out[6] = time_scale * 2 * (*((double *)pSys[7])) * (x[6] - yaw_rate_ref_t);
}

/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    void **pSys = (void **)userparam;
    ctypeRNum *udes = param->udes;
    // gradient of the stage cost w.r.t. state u:
    // dl/dx = 2R(u - u_{des})
    out[0] = 2 * (*((double *)pSys[11])) * (u[0] - udes[0]);
    out[1] = 2 * (*((double *)pSys[12])) * (u[1] - udes[1]);
}

/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Terminal cost V(T,x(T),p,xdes,userparam)
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    double x_ref_t, y_ref_t, psi_ref_t, delta_ref_t, vx_ref_t, vy_ref_t, yaw_rate_ref_t;
    void **pSys = (void **)userparam;
    double time_scale = (THOR - T) / THOR;

    get_reference_at_time((const double *)pSys[13],
                          (const double *)pSys[14],
                          (const double *)pSys[15],
                          (const double *)pSys[16],
                          (const double *)pSys[17],
                          (const double *)pSys[18],
                          (const double *)pSys[19],
                          (const double *)pSys[20],
                          (int)(*((double *)pSys[21])),
                          T + *((double *)pSys[22]),
                          &x_ref_t, &y_ref_t, &psi_ref_t,
                          &delta_ref_t, &vx_ref_t, &vy_ref_t, &yaw_rate_ref_t);

    out[0] = time_scale * (*((double *)pSys[6])) * POW2(x[0] - x_ref_t) +
             time_scale * (*((double *)pSys[7])) * POW2(x[1] - y_ref_t) +
             time_scale * (*((double *)pSys[8])) * POW2(x[2] - psi_ref_t) +
             time_scale * (*((double *)pSys[9])) * POW2(x[3] - delta_ref_t) +
             time_scale * (*((double *)pSys[10])) * POW2(x[4] - vx_ref_t) +
             time_scale * (*((double *)pSys[11])) * POW2(x[5] - vy_ref_t) +
             time_scale * (*((double *)pSys[12])) * POW2(x[6] - yaw_rate_ref_t);
}

/** Gradient dV/dx : Terminal Cost Function V(x(T))**/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    double x_ref_t, y_ref_t, psi_ref_t, delta_ref_t, vx_ref_t, vy_ref_t, yaw_rate_ref_t;
    void **pSys = (void **)userparam;
    double time_scale = (THOR - T) / THOR;

    get_reference_at_time((const double *)pSys[13],
                          (const double *)pSys[14],
                          (const double *)pSys[15],
                          (const double *)pSys[16],
                          (const double *)pSys[17],
                          (const double *)pSys[18],
                          (const double *)pSys[19],
                          (const double *)pSys[20],
                          (int)(*((double *)pSys[21])),
                          T + *((double *)pSys[22]),
                          &x_ref_t, &y_ref_t, &psi_ref_t,
                          &delta_ref_t, &vx_ref_t, &vy_ref_t, &yaw_rate_ref_t);

    // V(x(T)) = (x(T) - x_des) * P * (x(T) - x_des)
    out[0] = time_scale * 2 * (*((double *)pSys[6])) * (x[0] - x_ref_t);
    out[1] = time_scale * 2 * (*((double *)pSys[7])) * (x[1] - y_ref_t);
    out[2] = time_scale * 2 * (*((double *)pSys[8])) * (x[2] - psi_ref_t);
    out[3] = time_scale * 2 * (*((double *)pSys[9])) * (x[3] - delta_ref_t);
    out[4] = time_scale * 2 * (*((double *)pSys[10])) * (x[4] - vx_ref_t);
    out[5] = time_scale * 2 * (*((double *)pSys[11])) * (x[5] - vy_ref_t);
    out[6] = time_scale * 2 * (*((double *)pSys[12])) * (x[6] - yaw_rate_ref_t);
}

/** Gradient dV/dp **/
void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Gradient dV/dT **/
void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    out[0] = 0;
}

/** Equality constraints g(t,x(t),u(t),p,param,userparam) = 0
    --------------------------------------------------- **/
void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Inequality constraints h(t,x(t),u(t),p,param,userparam) <= 0
    ------------------------------------------------------ **/
/** inequality constraints: modify hfct to include steering bounds + lateral accel bound **/
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    ctypeRNum delta = x[3];
    const double delta_max = M_PI / 4.0; // as before

    out[0] = delta - delta_max;
    out[1] = -delta - delta_max;

    void **pSys = (void **)userparam;

    double m = *((double *)pSys[23]);
    double Iz = *((double *)pSys[24]);
    double Cf = *((double *)pSys[25]); // cornering stiffness front
    double Cr = *((double *)pSys[26]); // cornering stiffness rear
    double a = *((double *)pSys[27]);  // distance from CG to front axle
    double b = *((double *)pSys[28]);  // distance from CG to rear axle
    double g = *((double *)pSys[29]);
    double mu = *((double *)pSys[30]); // friction coef

    double vx = x[4];
    double vy = x[5];
    double r = x[6];

    double vx_eps = (vx > 0.1) ? vx : 0.1;
    double alpha_f = delta - ((vy + a * r) / vx_eps);
    double alpha_r = -((vy - b * r) / vx_eps);
    double Fy_f = -Cf * alpha_f;
    double Fy_r = -Cr * alpha_r;
    double a_lat = (Fy_f * COS(delta) + Fy_r) / m; /* lateral accel m/s^2 */

    double a_lat_max = mu * g; // conservative limit
    /* constraint: |a_lat| - a_lat_max <= 0  -> we create out[2] = |a_lat| - a_lat_max */
    out[2] = fabs(a_lat) - a_lat_max;
}

/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    // h0 = delta - delta_max → ∂h0/∂delta = +1
    // h1 = -delta - delta_max → ∂h1/∂delta = -1
    // vec = [v0, v1]^T selecting combination

    out[0] = 0;                              // x
    out[1] = 0;                              // y
    out[2] = 0;                              // psi
    out[3] = vec[0] * 1.0 + vec[1] * (-1.0); // only delta component contributes
    out[4] = 0;                              // v
}
/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    out[0] = 0;
    out[1] = 0;
}
/** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Terminal equality constraints gT(T,x(T),p,param,userparam) = 0
    -------------------------------------------------------- **/
void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Terminal inequality constraints hT(T,x(T),p,param,userparam) <= 0
    ----------------------------------------------------------- **/
void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dx multiplied by vector vec, i.e. (dhT/dx)^T*vec or vec^T*(dhT/dx) **/
void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dp multiplied by vector vec, i.e. (dhT/dp)^T*vec or vec^T*(dhT/dp) **/
void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dT multiplied by vector vec, i.e. (dhT/dT)^T*vec or vec^T*(dhT/dT) **/
void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Additional functions required for semi-implicit systems
    M*dx/dt(t) = f(t,x(t),u(t),p,param,userparam) using the solver RODAS
    ------------------------------------------------------- **/
/** Jacobian df/dx in vector form (column-wise) **/
void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dx in vector form (column-wise) **/
void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dt **/
void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian d(dH/dx)/dt  **/
void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mfct(typeRNum *out, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Transposed mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mtrans(typeRNum *out, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
