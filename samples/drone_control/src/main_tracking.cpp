// Quadrotor hovering example (API-converted)
// States: x, y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi
// Inputs: u1..u4 (0–1), Crazyflie order
// Note: phi/theta/psi are Rodrigues parameters (see the referenced paper)

#include <stdio.h>
#include <admm.hpp>

#include "problem_data/quadrotor_20hz_params.hpp"
#include "trajectory_data/quadrotor_20hz_y_axis_line.hpp"
#include <zephyr/sys/reboot.h>

extern "C" {

TinyCache cache;
TinyWorkspace work;
TinySettings settings;
TinySolver solver{&settings, &cache, &work};

int main(void)
{
    printf("Entered main!\n");

    enable_vector_operations();
    tiny_init(&solver);
    uint64_t start, end;

    // temporaries
    tiny_VectorNx v1, v2, x0, x1;
    init_VectorNx(&v1);
    init_VectorNx(&v2);
    init_VectorNx(&x0);
    init_VectorNx(&x1);

    // ---- Map arrays from problem_data (row-major) ----
    cache.rho = rho_value;

    matsetv(cache.Kinf.data,     Kinf_data,     cache.Kinf.outer,     cache.Kinf.inner);
    transpose(cache.Kinf.data,   cache.KinfT.data, NINPUTS, NSTATES);

    matsetv(cache.Pinf.data,     Pinf_data,     cache.Pinf.outer,     cache.Pinf.inner);
    transpose(cache.Pinf.data,   cache.PinfT.data, NSTATES, NSTATES);

    matsetv(cache.Quu_inv.data,  Quu_inv_data,  cache.Quu_inv.outer,  cache.Quu_inv.inner);
    matsetv(cache.AmBKt.data,    AmBKt_data,    cache.AmBKt.outer,    cache.AmBKt.inner);
    transpose(cache.AmBKt.data,  cache.AmBKtT.data, NSTATES, NSTATES);
    matsetv(cache.coeff_d2p.data,coeff_d2p_data,cache.coeff_d2p.outer,cache.coeff_d2p.inner);

    matsetv(work.Adyn.data,      Adyn_data,     work.Adyn.outer,      work.Adyn.inner);
    transpose(work.Adyn.data,    work.AdynT.data, NSTATES, NSTATES);
    matsetv(work.Bdyn.data,      Bdyn_data,     work.Bdyn.outer,      work.Bdyn.inner);
    transpose(work.Bdyn.data,    work.BdynT.data, NSTATES, NINPUTS);

    matsetv(work.Q.data,         Q_data,        work.Q.outer,         work.Q.inner);
    matsetv(work.R.data,         R_data,        work.R.outer,         work.R.inner);

    // ---- Valid ranges for inputs and states ----
    matset(work.u_min.data, -0.583f,        work.u_min.outer, work.u_min.inner);
    matset(work.u_max.data, 1.0f - 0.583f,  work.u_max.outer, work.u_max.inner);
    matset(work.x_min.data, -5.0f,          work.x_min.outer, work.x_min.inner);
    matset(work.x_max.data,  5.0f,          work.x_max.outer, work.x_max.inner);

    // ---- Optimization residuals and settings ----
    work.primal_residual_state = 0;
    work.primal_residual_input = 0;
    work.dual_residual_state   = 0;
    work.dual_residual_input   = 0;
    work.status = 0;
    work.iter   = 0;

    settings.abs_pri_tol      = 0.001f;
    settings.abs_dua_tol      = 0.001f;
    settings.max_iter         = 100;
    settings.check_termination= 1;
    settings.en_input_bound   = 1;
    settings.en_state_bound   = 1;

    // ---- Initialize Xref horizon from trajectory (k = 0) ----
    // Xref_data is a flat array with NTOTAL columns of size NSTATES.
    for (int j = 0; j < NHORIZON; ++j) {
        tinytype* src = Xref_data + j * NSTATES;
        matsetv(work.Xref.vector[j], src, 1, NSTATES);
    }
    TRACE_CHECKSUM(init_xref_total, work.Xref);

    // ---- Initial state x0: first state from Xref_data ----
    matsetv(x0.data, Xref_data, x0.outer, x0.inner);
    
    TRACE_CHECKSUM(init_x0, x0);

    for (int k = 0; k < 10; ++k) {
        // Tracking error: || x0 - Xref[:,1] ||
        matsub(x0.data, work.Xref.vector[1], v1.data, 1, NSTATES);
        TRACE_CHECKSUM(main_loop_x0,   x0);
        TRACE_CHECKSUM(main_loop_xref, work.Xref);
        TRACE_CHECKSUM(main_loop_v1,   v1);
        float norm = matnorm(v1.data, 1, NSTATES);
        printf("Tracking error: %0.7f\n", norm);

        // 1) Update measurement into column 0
        matsetv(work.x.vector[0], x0.data, 1, NSTATES);

        // 2) Update reference horizon to start at step k
        for (int j = 0; j < NHORIZON; ++j) {
            const int idx = k + j;
            tinytype* src = Xref_data + idx * NSTATES;
            matsetv(work.Xref.vector[j], src, 1, NSTATES);
        }

        // 3) Reset dual variables
        matset(work.y.data, 0.0f, work.y.outer, work.y.inner);
        matset(work.g.data, 0.0f, work.g.outer, work.g.inner);

        // 4) Solve MPC
        start = read_cycles();
        tiny_solve(&solver);
        end   = read_cycles();
        printf("Time for iter %d: %llu\n", k, (unsigned long long)(end - start));

        // 5) Simulate forward: x1 = A*x0 + B*u(:,0)
    #ifdef USE_MATVEC
        matvec(work.Adyn.data, x0.data,                  v1.data, NSTATES, NSTATES);
        matvec(work.Bdyn.data, work.u.vector[0],         v2.data, NSTATES, NINPUTS);
    #else
        matmul(x0.data,          work.Adyn.data, v1.data, 1, NSTATES, NSTATES);
        matmul(work.u.vector[0], work.Bdyn.data, v2.data, 1, NSTATES, NINPUTS);
    #endif
        matadd(v1.data, v2.data, x0.data, 1, NSTATES);

        TRACE_CHECKSUM(main_loop_u_0, work.u);
        TRACE_CHECKSUM(main_loop_v1,  v1);
        TRACE_CHECKSUM(main_loop_v2,  v2);
        TRACE_CHECKSUM(main_loop_x0,  x0);
    }

    sys_reboot(SYS_REBOOT_COLD);
    return 0;
}

} // extern "C"
