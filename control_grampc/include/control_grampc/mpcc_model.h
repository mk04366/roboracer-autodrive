#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

    // State dimension: nx = 5  -> [x, y, theta, v, s]
    // Control dimension: nu = 2 -> [delta, a]

    // GRAMPC will be configured to call these via function pointers.
    // You'll pass a pointer to a user context (void* user) that we define in mpcc_model.c.

    void mpcc_dynamics(double t, const double *x, const double *u, double *xdot, void *user);

    double mpcc_stage_cost(double t, const double *x, const double *u, void *user);

    double mpcc_terminal_cost(const double *x, void *user);

#ifdef __cplusplus
}
#endif
