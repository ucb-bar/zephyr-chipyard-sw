// CURRENT VERSION

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <cstring>
#include <cstdint>

#include "attitude_estimator.h"

#include <admm.hpp>
#include <glob_opts.hpp>
#include <quadrotor_50hz_params_constrained.hpp>
#include "types_eigen.hpp"

LOG_MODULE_REGISTER(control_task, CONFIG_LOG_DEFAULT_LEVEL);

#define CONTROL_HZ 100
#define CONTROL_MS (1000 / CONTROL_HZ)
#define CONTROL_STACK_SIZE 4096
#define CONTROL_PRIORITY 6

K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);
static struct k_thread control_tid;


static TinySolver solver;
static TinyCache cache;
static TinyWorkspace work;
static TinySettings settings;

static void control_thread_fn(void *, void *, void *)
{
    LOG_INF("Control task started at %d Hz", CONTROL_HZ);

    solver.cache    = &cache;
    solver.work     = &work;
    solver.settings = &settings;
    tiny_init(&solver);

    tiny_VectorNx_init(&work.x);
    tiny_VectorNu_init(&work.u);

    cache.rho = rho_value;

    matsetv(cache.Kinf.data, Kinf_data, cache.Kinf.outer, cache.Kinf.inner);
    transpose(cache.Kinf.data, cache.KinfT.data, NINPUTS, NSTATES);
    matsetv(cache.Pinf.data, Pinf_data, cache.Pinf.outer, cache.Pinf.inner);
    transpose(cache.Pinf.data, cache.PinfT.data, NSTATES, NSTATES);
    matsetv(cache.Quu_inv.data, Quu_inv_data, cache.Quu_inv.outer, cache.Quu_inv.inner);
    matsetv(cache.AmBKt.data, AmBKt_data, cache.AmBKt.outer, cache.AmBKt.inner);
    transpose(cache.AmBKt.data, cache.AmBKtT.data, NSTATES, NSTATES);
    matsetv(cache.coeff_d2p.data, coeff_d2p_data, cache.coeff_d2p.outer, cache.coeff_d2p.inner);

    matsetv(work.Adyn.data, Adyn_data, work.Adyn.outer, work.Adyn.inner);
    transpose(work.Adyn.data, work.AdynT.data, NSTATES, NSTATES);
    matsetv(work.Bdyn.data, Bdyn_data, work.Bdyn.outer, work.Bdyn.inner);
    transpose(work.Bdyn.data, work.BdynT.data, NSTATES, NINPUTS);
    matsetv(work.Q.data, Q_data, work.Q.outer, work.Q.inner);
    matsetv(work.R.data, R_data, work.R.outer, work.R.inner);

    matset(work.u_min.data, -0.583f, work.u_min.outer, work.u_min.inner);
    matset(work.u_max.data, 0.417f, work.u_max.outer, work.u_max.inner);
    matset(work.x_min.data, -5.0f, work.x_min.outer, work.x_min.inner);
    matset(work.x_max.data, 5.0f, work.x_max.outer, work.x_max.inner);

    float Xref_origin[NSTATES] = {0};
    for (int j = 0; j < NHORIZON; j++) {
        matsetv(work.Xref.vector[j], Xref_origin, 1, NSTATES);
    }

    struct attitude att, prev_att = {0};
    attitude_estimator_get(&prev_att);

    float dt_s = 1.0f / (float)CONTROL_HZ;
    uint64_t last_time = k_uptime_get();

    while (true) {
        uint64_t t0 = k_uptime_get();

        if (attitude_estimator_get(&att) != 0) att = prev_att;

        float roll_rate  = (att.roll  - prev_att.roll)  / dt_s;
        float pitch_rate = (att.pitch - prev_att.pitch) / dt_s;
        float yaw_rate   = (att.yaw   - prev_att.yaw)   / dt_s;
        prev_att = att;

        float x[NSTATES] = {0};
        x[0] = att.roll;
        x[1] = att.pitch;
        x[2] = att.yaw;
        x[3] = roll_rate;
        x[4] = pitch_rate;
        x[5] = yaw_rate;

        memcpy(work.x.vector[0], x, NSTATES * sizeof(float));
        matset(work.y.data, 0.0f, work.y.outer, work.y.inner);
        matset(work.g.data, 0.0f, work.g.outer, work.g.inner);

        tiny_solve(&solver);

        float* u = work.u.vector[0];

        LOG_INF("att r=%.3f p=%.3f y=%.3f | u0=%.3f u1=%.3f u2=%.3f u3=%.3f",
                att.roll, att.pitch, att.yaw,
                u[0], u[1], u[2], u[3]);

        uint64_t elapsed = k_uptime_get() - t0;
        int64_t sleep_ms = (int64_t)CONTROL_MS - (int64_t)elapsed;
        if (sleep_ms > 0) k_msleep((uint32_t)sleep_ms);
        else k_yield();
    }
}


void control_task_start()
{
    k_thread_create(&control_tid, control_stack, K_THREAD_STACK_SIZEOF(control_stack),
                    control_thread_fn, nullptr, nullptr, nullptr,
                    CONTROL_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&control_tid, "control_task");
}

