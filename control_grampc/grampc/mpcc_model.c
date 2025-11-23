#include "control_grampc/mpcc_model.h"
#include <stdio.h>
#include <stdint.h>
#if USE_typeRNum == USE_FLOAT
#define SIN(a) sinf(a)
#define COS(a) cosf(a)
#define ATAN2(a, b) atan2f(a, b)
#else
#define SIN(a) sin(a)
#define COS(a) cos(a)
#define ATAN2(a, b) atan2(a, b)
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
                                  const double *v_x_ref,
                                  const double *v_y_ref,
                                  const double *psi_rate_ref,
                                  int N,
                                  double t,
                                  double *x_out,
                                  double *y_out,
                                  double *psi_out,
                                  double *delta_out,
                                  double *v_x_out,
                                  double *v_y_out,
                                  double *psi_rate_out)
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
    *psi_out = psi_ref[i] + alpha * (psi_ref[i + 1] - psi_ref[i]);
    *delta_out = delta_ref[i] + alpha * (delta_ref[i + 1] - delta_ref[i]);
    *v_x_out = v_x_ref[i] + alpha * (v_x_ref[i + 1] - v_x_ref[i]);
    *v_y_out = v_y_ref[i] + alpha * (v_y_ref[i + 1] - v_y_ref[i]);
    *psi_rate_out = psi_rate_ref[i] + alpha * (psi_rate_ref[i + 1] - psi_rate_ref[i]);
}

/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    *Nx = 7; // [x, y, psi(orientation), delta(steering), vx, vy, psi_rate]
    *Nu = 2; // [delta_dot, a]
    *Np = 0;
    *Nh = 2;
    *Ng = 0;
    *NgT = 0;
    *NhT = 0;
}

/** System Dynamics function f(t,x,u,p,userparam)
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    void **pSys = (void **)userparam;
    ctypeRNum L = *((ctypeRNum *)pSys[0]);
    ctypeRNum Cf = *((ctypeRNum *)pSys[1]);
    ctypeRNum Cr = *((ctypeRNum *)pSys[2]);
    ctypeRNum m = *((ctypeRNum *)pSys[3]);
    ctypeRNum Iz = *((ctypeRNum *)pSys[4]);
    ctypeRNum lf = *((ctypeRNum *)pSys[5]);
    ctypeRNum lr = *((ctypeRNum *)pSys[6]);

    ctypeRNum psi = x[2];
    ctypeRNum delta = x[3];
    ctypeRNum vx = x[4];
    ctypeRNum vy = x[5];
    ctypeRNum psi_rate = x[6];

    /* state ordering: [x, y, psi, delta, vx, vy, psi_rate] */
    out[0] = vx * COS(psi) - vy * SIN(psi);                                                                                          // ẋ = vx * cos(psi) - vy * sin(psi)
    out[1] = vx * SIN(psi) + vy * COS(psi);                                                                                          // ẏ = vx * sin(psi) + vy * cos(psi)
    out[2] = psi_rate;                                                                                                               // ψ̇ = psi_rate
    out[3] = u[0];                                                                                                                   // δ̇ = delta_dot
    out[4] = u[1] + (psi_rate * vy);                                                                                                 // vẋ = a + (psi_rate * vy) [approximation]
    out[5] = ((Cf * COS(delta) * (delta - (vy + lf * psi_rate) / vx)) / m - psi_rate * vx - (Cr * (vy - lr * psi_rate)) / (m * vx)); // vẏ = -(psi_rate * vx) + Fy_total/mass
    out[6] = ((Cf * lf * COS(delta) * (delta - (vy + lf * psi_rate) / vx)) / Iz + (Cr * lr * (vy - lr * psi_rate)) / (Iz * vx));     // psi_ratė = front_axle_distance*F_front_y - rear_axle_distance*F_rear_y / I_z
}

/** Jacobian df/dx multiplied by vector vec: out = (df/dx)^T * vec **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p,
              ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    void **pSys = (void **)userparam;
    ctypeRNum L = *((ctypeRNum *)pSys[0]);
    ctypeRNum Cf = *((ctypeRNum *)pSys[1]);
    ctypeRNum Cr = *((ctypeRNum *)pSys[2]);
    ctypeRNum m = *((ctypeRNum *)pSys[3]);
    ctypeRNum Iz = *((ctypeRNum *)pSys[4]);
    ctypeRNum lf = *((ctypeRNum *)pSys[5]);
    ctypeRNum lr = *((ctypeRNum *)pSys[6]);

    ctypeRNum psi = x[2];
    ctypeRNum delta = x[3];
    ctypeRNum vx = x[4];
    ctypeRNum vy = x[5];
    ctypeRNum psi_rate = x[6];

    out[0] = 0; // df(.)/dx * vec
    out[1] = 0; // df(.)/dy * vec
    out[2] = vec[0] * (-vx * SIN(psi) - vy * COS(psi)) +
             vec[1] * (vx * COS(psi) - vy * SIN(psi)); // df(.)/dpsi * vec

    out[3] = vec[5] * ((Cf * COS(delta)) / m - (Cf * SIN(delta) * (delta - (vy + lf * psi_rate) / vx)) / m) +
             vec[6] * ((Cf * lf * COS(delta)) / Iz - (Cf * lf * SIN(delta) * (delta - (vy + lf * psi_rate) / vx)) / Iz); // df(.)/ddelta * vec

    out[4] = vec[0] * COS(psi) +
             vec[1] * SIN(psi) +
             vec[5] * ((Cr * (vy - lr * psi_rate)) / (m * POW2(vx)) - psi_rate + (Cf * COS(delta) * (vy + lf * psi_rate)) / (m * POW2(vx))) +
             vec[6] * ((Cf * lf * COS(delta) * (vy + lf * psi_rate)) / (Iz * POW2(vx)) - (Cr * lr * (vy - lr * psi_rate)) / (Iz * POW2(vx))); // df(.)/dvx * vec

    out[5] = vec[0] * (-SIN(psi)) +
             vec[1] * COS(psi) +
             vec[4] * (psi_rate) +
             vec[5] * (-Cr / (m * vx) - (Cf * COS(delta)) / (m * vx)) +
             vec[6] * ((Cr * lr) / (Iz * vx) - (Cf * lf * COS(delta)) / (Iz * vx)); // df(.)/dvy * vec

    out[6] = vec[2] +
             vec[4] * vy +
             vec[5] * ((Cr * lr) / (m * vx) - vx - (Cf * lf * COS(delta)) / (m * vx)) +
             vec[6] * (-(Cr * POW2(lr)) / (Iz * vx) - (Cf * POW2(lf) * COS(delta)) / (Iz * vx)); // df(.)/dpsi_rate * vec
}

/** Jacobian df/du multiplied by vector vec: out = (df/du)^T * vec **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p,
              ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    /* u0 = delta_dot  -> appears in f3
       u1 = a          -> appears in f4
       So (df/du)^T * vec = [vec[3]; vec[4]]
    */
    out[0] = vec[3];
    out[1] = vec[4];
}
/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam)
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{

    double x_ref_t, y_ref_t, psi_ref_t, delta_ref_t, v_x_ref_t, v_y_ref_t, psi_rate_ref_t;
    void **pSys = (void **)userparam;
    ctypeRNum *xdes = param->xdes;
    ctypeRNum *udes = param->udes;
    // double time_scale = (THOR - t) / THOR;
    double time_scale = 1.0;
    const double *t_array = (const double *)pSys[23];
    const double *x_ref = (const double *)pSys[24];
    const double *y_ref = (const double *)pSys[25];
    const double *psi_ref = (const double *)pSys[26];
    const double *delta_ref = (const double *)pSys[27];
    const double *v_x_ref = (const double *)pSys[28];
    const double *v_y_ref = (const double *)pSys[29];
    const double *psi_rate_ref = (const double *)pSys[30];

    const double N = (double)(*((double *)pSys[31]));
    const double current_time_offset = *((double *)pSys[32]);

    const double weight_u0 = *((double *)pSys[21]);
    const double weight_u1 = *((double *)pSys[22]);

    const double weight_q0 = *((double *)pSys[7]);
    const double weight_q1 = *((double *)pSys[8]);
    const double weight_q2 = *((double *)pSys[9]);
    const double weight_q3 = *((double *)pSys[10]);
    const double weight_q4 = *((double *)pSys[11]);
    const double weight_q5 = *((double *)pSys[12]);
    const double weight_q6 = *((double *)pSys[13]);

    get_reference_at_time(t_array,
                          x_ref,
                          y_ref,
                          psi_ref,
                          delta_ref,
                          v_x_ref,
                          v_y_ref,
                          psi_rate_ref,
                          N,
                          t + current_time_offset,
                          &x_ref_t, &y_ref_t, &psi_ref_t,
                          &delta_ref_t, &v_x_ref_t, &v_y_ref_t, &psi_rate_ref_t);

    out[0] =
        (weight_u0 * POW2(u[0] - udes[0]) +       // control effort weight for u₀ = κ̇
         weight_u1 * POW2(u[1] - udes[1]) +       // control effort weight for u₁ = v̇
         weight_q0 * POW2(x[0] - x_ref_t) +       // position x error weight
         weight_q1 * POW2(x[1] - y_ref_t) +       // position y error weight
         weight_q2 * POW2(x[2] - psi_ref_t) +     // heading error weight
         weight_q3 * POW2(x[3] - delta_ref_t) +   // steering error weight
         weight_q4 * POW2(x[4] - v_x_ref_t) +     // longitudinal velocity error weight
         weight_q5 * POW2(x[5] - v_y_ref_t) +     // lateral velocity error weight
         weight_q6 * POW2(x[6] - psi_rate_ref_t)) // yaw rate error weight
        * time_scale;                             // time-varying scaling
}

/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{

    double x_ref_t, y_ref_t, psi_ref_t, delta_ref_t, v_x_ref_t, v_y_ref_t, psi_rate_ref_t;
    void **pSys = (void **)userparam;
    ctypeRNum *xdes = param->xdes;
    ctypeRNum *udes = param->udes;
    // double time_scale = (THOR - t) / THOR;
    double time_scale = 1.0;
    const double *t_array = (const double *)pSys[23];
    const double *x_ref = (const double *)pSys[24];
    const double *y_ref = (const double *)pSys[25];
    const double *psi_ref = (const double *)pSys[26];
    const double *delta_ref = (const double *)pSys[27];
    const double *v_x_ref = (const double *)pSys[28];
    const double *v_y_ref = (const double *)pSys[29];
    const double *psi_rate_ref = (const double *)pSys[30];

    const double N = (double)(*((double *)pSys[31]));
    const double current_time_offset = *((double *)pSys[32]);

    const double weight_q0 = *((double *)pSys[7]);
    const double weight_q1 = *((double *)pSys[8]);
    const double weight_q2 = *((double *)pSys[9]);
    const double weight_q3 = *((double *)pSys[10]);
    const double weight_q4 = *((double *)pSys[11]);
    const double weight_q5 = *((double *)pSys[12]);
    const double weight_q6 = *((double *)pSys[13]);

    get_reference_at_time(t_array,
                          x_ref,
                          y_ref,
                          psi_ref,
                          delta_ref,
                          v_x_ref,
                          v_y_ref,
                          psi_rate_ref,
                          N,
                          t + current_time_offset,
                          &x_ref_t, &y_ref_t, &psi_ref_t,
                          &delta_ref_t, &v_x_ref_t, &v_y_ref_t, &psi_rate_ref_t);

    // gradient of the stage cost w.r.t. state x:
    // dl/dx = 2Q(x - x_{des})
    out[0] = time_scale * 2 * weight_q0 * (x[0] - x_ref_t);
    out[1] = time_scale * 2 * weight_q1 * (x[1] - y_ref_t);
    out[2] = time_scale * 2 * weight_q2 * (x[2] - psi_ref_t);
    out[3] = time_scale * 2 * weight_q3 * (x[3] - delta_ref_t);
    out[4] = time_scale * 2 * weight_q4 * (x[4] - v_x_ref_t);
    out[5] = time_scale * 2 * weight_q5 * (x[5] - v_y_ref_t);
    out[6] = time_scale * 2 * weight_q6 * (x[6] - psi_rate_ref_t);
}

/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    void **pSys = (void **)userparam;
    ctypeRNum *udes = param->udes;

    const double weight_u0 = *((double *)pSys[21]);
    const double weight_u1 = *((double *)pSys[22]);
    // gradient of the stage cost w.r.t. state u:
    // dl/dx = 2R(u - u_{des})
    out[0] = 2 * weight_u0 * (u[0] - udes[0]);
    out[1] = 2 * weight_u1 * (u[1] - udes[1]);
}

/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}

/** Terminal cost V(T,x(T),p,xdes,userparam)
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    double x_ref_t, y_ref_t, psi_ref_t, delta_ref_t, v_x_ref_t, v_y_ref_t, psi_rate_ref_t;
    void **pSys = (void **)userparam;
    ctypeRNum *xdes = param->xdes;
    ctypeRNum *udes = param->udes;
    // double time_scale = (THOR - t) / THOR;
    double time_scale = 1.0;
    const double *t_array = (const double *)pSys[23];
    const double *x_ref = (const double *)pSys[24];
    const double *y_ref = (const double *)pSys[25];
    const double *psi_ref = (const double *)pSys[26];
    const double *delta_ref = (const double *)pSys[27];
    const double *v_x_ref = (const double *)pSys[28];
    const double *v_y_ref = (const double *)pSys[29];
    const double *psi_rate_ref = (const double *)pSys[30];

    const double N = (double)(*((double *)pSys[31]));
    const double current_time_offset = *((double *)pSys[32]);

    const double weight_r0 = *((double *)pSys[14]);
    const double weight_r1 = *((double *)pSys[15]);
    const double weight_r2 = *((double *)pSys[16]);
    const double weight_r3 = *((double *)pSys[17]);
    const double weight_r4 = *((double *)pSys[18]);
    const double weight_r5 = *((double *)pSys[19]);
    const double weight_r6 = *((double *)pSys[20]);

    get_reference_at_time(t_array,
                          x_ref,
                          y_ref,
                          psi_ref,
                          delta_ref,
                          v_x_ref,
                          v_y_ref,
                          psi_rate_ref,
                          N,
                          T + current_time_offset,
                          &x_ref_t, &y_ref_t, &psi_ref_t,
                          &delta_ref_t, &v_x_ref_t, &v_y_ref_t, &psi_rate_ref_t);

    out[0] = time_scale * weight_r0 * POW2(x[0] - x_ref_t) +
             time_scale * weight_r1 * POW2(x[1] - y_ref_t) +
             time_scale * weight_r2 * POW2(x[2] - psi_ref_t) +
             time_scale * weight_r3 * POW2(x[3] - delta_ref_t) +
             time_scale * weight_r4 * POW2(x[4] - v_x_ref_t) +
             time_scale * weight_r5 * POW2(x[5] - v_y_ref_t) +
             time_scale * weight_r6 * POW2(x[6] - psi_rate_ref_t);
}

/** Gradient dV/dx : Terminal Cost Function V(x(T))**/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    double x_ref_t, y_ref_t, psi_ref_t, delta_ref_t, v_x_ref_t, v_y_ref_t, psi_rate_ref_t;
    void **pSys = (void **)userparam;
    ctypeRNum *xdes = param->xdes;
    ctypeRNum *udes = param->udes;
    // double time_scale = (THOR - t) / THOR;
    double time_scale = 1.0;
    const double *t_array = (const double *)pSys[23];
    const double *x_ref = (const double *)pSys[24];
    const double *y_ref = (const double *)pSys[25];
    const double *psi_ref = (const double *)pSys[26];
    const double *delta_ref = (const double *)pSys[27];
    const double *v_x_ref = (const double *)pSys[28];
    const double *v_y_ref = (const double *)pSys[29];
    const double *psi_rate_ref = (const double *)pSys[30];

    const double N = (double)(*((double *)pSys[31]));
    const double current_time_offset = *((double *)pSys[32]);

    const double weight_r0 = *((double *)pSys[14]);
    const double weight_r1 = *((double *)pSys[15]);
    const double weight_r2 = *((double *)pSys[16]);
    const double weight_r3 = *((double *)pSys[17]);
    const double weight_r4 = *((double *)pSys[18]);
    const double weight_r5 = *((double *)pSys[19]);
    const double weight_r6 = *((double *)pSys[20]);

    get_reference_at_time(t_array,
                          x_ref,
                          y_ref,
                          psi_ref,
                          delta_ref,
                          v_x_ref,
                          v_y_ref,
                          psi_rate_ref,
                          N,
                          T + current_time_offset,
                          &x_ref_t, &y_ref_t, &psi_ref_t,
                          &delta_ref_t, &v_x_ref_t, &v_y_ref_t, &psi_rate_ref_t);

    // V(x(T)) = (x(T) - x_des) * P * (x(T) - x_des)
    out[0] = time_scale * 2 * weight_r0 * (x[0] - x_ref_t);
    out[1] = time_scale * 2 * weight_r1 * (x[1] - y_ref_t);
    out[2] = time_scale * 2 * weight_r2 * (x[2] - psi_ref_t);
    out[3] = time_scale * 2 * weight_r3 * (x[3] - delta_ref_t);
    out[4] = time_scale * 2 * weight_r4 * (x[4] - v_x_ref_t);
    out[5] = time_scale * 2 * weight_r5 * (x[5] - v_y_ref_t);
    out[6] = time_scale * 2 * weight_r6 * (x[6] - psi_rate_ref_t);
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
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    ctypeRNum delta = x[3];
    const double delta_max = M_PI / 4.0;

    // Constraint 1: delta - delta_max ≤ 0
    out[0] = delta - delta_max;

    // Constraint 2: -delta - delta_max ≤ 0
    out[1] = -delta - delta_max;
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
    out[5] = 0;                              // vy
    out[6] = 0;                              // psi_rate
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
