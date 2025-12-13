// This header is included by both C and C++ code
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "probfct.h"

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
    void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam);

    /** System function f(t,x,u,p,param,userparam)
        ------------------------------------ **/
    void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
    void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
    void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
    void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);

    /** Integral cost l(t,x(t),u(t),p,param,userparam)
        -------------------------------------------------- **/
    void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Gradient dl/dx **/
    void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Gradient dl/du **/
    void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Gradient dl/dp **/
    void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);

    /** Terminal cost V(T,x(T),p,param,userparam)
        ---------------------------------------- **/
    void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Gradient dV/dx **/
    void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Gradient dV/dp **/
    void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);

    /** Equality constraints g(t,x(t),u(t),p,param,userparam) = 0
        --------------------------------------------------- **/
    void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
    void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
    void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
    void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);

    /** Inequality constraints h(t,x(t),u(t),p,param,userparam) <= 0
        ------------------------------------------------------ **/
    void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
    void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
    void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
    void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);

    /** Terminal equality constraints gT(T,x(T),p,param,userparam) = 0
        -------------------------------------------------------- **/
    void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
    void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
    void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    /** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
    void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);

    /** Terminal inequality constraints hT(T,x(T),p,param,userparam) <= 0
        ----------------------------------------------------------- **/
    void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);

    /** Additional functions required for semi-implicit systems
        M*dx/dt(t) = f(t,x(t),u(t),p,param,userparam) using the solver RODAS
        ------------------------------------------------------- **/
    /** Jacobian df/dx in vector form (column-wise) **/
    void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void Mfct(typeRNum *out, const typeGRAMPCparam *param, typeUSERPARAM *userparam);
    void Mtrans(typeRNum *out, const typeGRAMPCparam *param, typeUSERPARAM *userparam);

#ifdef __cplusplus
};
#endif