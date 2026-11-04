/* SPDX-License-Identifier: Apache-2.0
 * DroNet (steer, collision) -> forward-velocity and yaw setpoints. Platform-free, like
 * fc_shared_math.h, so the host tests compile exactly what the firmware runs.
 *
 * The mapping follows DroNet's own controller: forward speed = (1 - p_collision) * v_max, and
 * steering becomes a yaw rate. Both are first-order smoothed in time (not per DroNet frame) because
 * the control loop runs at a few hundred Hz against a zero-order-held command of a few Hz. The yaw
 * rate is integrated into a yaw reference that the PID's yaw-angle loop tracks. The reference is
 * carried as tan(psi/2), singular at 180 deg, so it is bounded to +-yaw_max.
 *
 * Conservative collision decoding: a sigmoid cannot be negative, and an output at the int8 ceiling
 * only says "at least 127 * coll_scale" -- 0.403 with the committed gray graph, which could never
 * command a stop. Both read as collision probability 1.
 */
#ifndef ROSE_FC_DRONET_NAV_H
#define ROSE_FC_DRONET_NAV_H
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

struct fc_dronet_nav_params {
    float steer_scale, coll_scale;  /* int8 output dequantization (zero point 0) */
    float v_max;                    /* forward speed at collision probability 0 (m/s) */
    float yaw_rate_max;             /* yaw rate at |steer| = 1 (rad/s) */
    float yaw_sign;                 /* +1: positive steer yaws nose-left (+z) */
    float tau_v, tau_w;             /* smoothing time constants (s); 0 = none */
    float yaw_max;                  /* bound on |yaw reference| (rad) */
    int64_t timeout_ms;             /* a command older than this counts as no command */
};

struct fc_dronet_nav {
    float v, w;      /* smoothed forward speed (m/s) and yaw rate (rad/s) */
    float psi_ref;   /* yaw reference (rad) */
};

struct fc_dronet_nav_out {
    float vx;         /* heading-frame forward velocity setpoint (m/s) */
    float yaw_gibbs;  /* yaw setpoint as the Gibbs parameter the controller compares against */
    float steer, collision;
    bool fresh;       /* a command inside the timeout was available */
};

static inline void fc_dronet_nav_reset(struct fc_dronet_nav *n)
{
    n->v = 0.0f; n->w = 0.0f; n->psi_ref = 0.0f;
}

static inline float fc_dronet_clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* One control tick. engaged = DroNet may move the vehicle now. Leaving the engaged state (end of the
 * hover window, abort landing, estop, lost flow) zeroes both commands at once: the velocity request
 * must not outlive the phase that allows it. A stale command inside the engaged state decays to zero
 * through the smoothing instead. The yaw reference is held in both cases. */
static inline struct fc_dronet_nav_out fc_dronet_nav_step(struct fc_dronet_nav *n,
        const struct fc_dronet_nav_params *p, bool engaged, bool have_cmd, int64_t age_ms,
        int8_t steer_q, int8_t coll_q, float dt)
{
    struct fc_dronet_nav_out out;
    out.steer = (float)steer_q * p->steer_scale;
    out.collision = (coll_q < 0 || coll_q == INT8_MAX) ? 1.0f
                  : fc_dronet_clampf((float)coll_q * p->coll_scale, 0.0f, 1.0f);
    out.fresh = have_cmd && age_ms >= 0 && age_ms <= p->timeout_ms;
    if (!engaged) {
        n->v = 0.0f;
        n->w = 0.0f;
    } else if (dt > 0.0f) {
        float v_tgt = 0.0f, w_tgt = 0.0f;
        if (out.fresh) {
            v_tgt = (1.0f - out.collision) * p->v_max;
            w_tgt = p->yaw_sign * fc_dronet_clampf(out.steer, -1.0f, 1.0f) * p->yaw_rate_max;
        }
        const float kv = p->tau_v > 0.0f ? 1.0f - expf(-dt / p->tau_v) : 1.0f;
        const float kw = p->tau_w > 0.0f ? 1.0f - expf(-dt / p->tau_w) : 1.0f;
        n->v += kv * (v_tgt - n->v);
        n->w += kw * (w_tgt - n->w);
        n->psi_ref = fc_dronet_clampf(n->psi_ref + n->w * dt, -p->yaw_max, p->yaw_max);
    }
    out.vx = n->v;
    out.yaw_gibbs = tanf(0.5f * n->psi_ref);
    return out;
}

/* The hover segment of the autoflight profile, matching fc_profile_z()'s segment boundaries. */
static inline bool fc_profile_in_hover(int64_t tf, int climb_ms, int hover_ms)
{
    const int64_t tc = climb_ms > 0 ? climb_ms : 1;
    const int64_t th = hover_ms > 0 ? hover_ms : 0;
    return tf >= tc && tf < tc + th;
}
#endif
