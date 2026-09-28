/* SPDX-License-Identifier: Apache-2.0
 * Platform-free arithmetic shared by Zephyr and native FC replay. Parameters
 * remain caller-owned so firmware defaults and compilation paths are preserved.
 */
#ifndef ROSE_FC_SHARED_MATH_H
#define ROSE_FC_SHARED_MATH_H
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
static inline float fc_tilt_rad(const float *state)
{
    return 2.0f * atanf(sqrtf(state[3]*state[3] + state[4]*state[4]));
}
static inline void fc_actuator_duty(const float *u, float duty[4], float battery_scale,
                                   float ceiling, bool autoflight, float bench_scale)
{
    float peak = 0.0f;
    for (int i=0; i<4; ++i) {
        duty[i] = (u[i] + 0.583f) * battery_scale;
        if (duty[i] < 0.0f) duty[i] = 0.0f;
        if (duty[i] > peak) peak = duty[i];
    }
    if (peak > ceiling) {
        const float cut = peak - ceiling;
        for (int i=0; i<4; ++i) {
            duty[i] -= cut;
            if (duty[i] < 0.0f) duty[i] = 0.0f;
        }
    }
    if (!autoflight) for (int i=0; i<4; ++i) {
        duty[i] *= bench_scale;
        if (duty[i] > bench_scale) duty[i] = bench_scale;
    }
}
static inline float fc_profile_z(int64_t t_ms, int climb_ms, int hover_ms,
                                 int descend_ms, float height, float land_push)
{
    const int tc = climb_ms > 0 ? climb_ms : 1;
    const int th = hover_ms > 0 ? hover_ms : 0;
    const int td = descend_ms > 0 ? descend_ms : 1;
    if (t_ms < tc) return height * ((float)t_ms / (float)tc);
    t_ms -= tc;
    if (t_ms < th) return height;
    t_ms -= th;
    float rate = height / (float)td;
    float sp = height - rate * (float)t_ms;
    return sp < land_push ? land_push : sp;
}
struct fc_flow_filter { float x, y; bool seeded; };
static inline void fc_flow_counts(struct fc_flow_filter *f, int dx, int dy, float dt,
                                  float rad_per_count, float tau_x, float tau_y)
{
    float ax = -(float)dx * rad_per_count / dt;
    float ay =  (float)dy * rad_per_count / dt;
    if (!f->seeded) { f->x=ax; f->y=ay; f->seeded=true; }
    float kx = tau_x > 0.0f ? 1.0f-expf(-dt/tau_x) : 1.0f;
    float ky = tau_y > 0.0f ? 1.0f-expf(-dt/tau_y) : 1.0f;
    f->x += kx*(ax-f->x); f->y += ky*(ay-f->y);
}
static inline void fc_flow_velocity(float ax, float ay, float height,
                                    float ga, float gb, float gc, float velocity[2])
{
    float ct = (1.0f-ga*ga-gb*gb+gc*gc)/(1.0f+ga*ga+gb*gb+gc*gc);
    float h = height * (ct > 0.0f ? ct : 0.0f);
    float vx=ax*h, vy=ay*h;
    velocity[0] = vx > 3.0f ? 3.0f : (vx < -3.0f ? -3.0f : vx);
    velocity[1] = vy > 3.0f ? 3.0f : (vy < -3.0f ? -3.0f : vy);
}
/* These helpers preserve main.cpp's timestamp sentinel and dwell semantics. */
static inline bool fc_arm_dwell(bool ready, int64_t now_ms, int64_t hold_ms, int64_t *since)
{
    if (ready) {
        if (*since == 0) *since = now_ms;
        else if (now_ms - *since >= hold_ms) return true;
    } else *since = 0;
    return false;
}
static inline bool fc_guard_dwell(bool violation, int64_t now_ms, int need_ms,
                                  int minimum_samples, int64_t *since, int *count)
{
    if (!violation) { *since=0; *count=0; }
    else {
        if (*since==0) { *since=now_ms; *count=0; }
        ++*count;
    }
    return violation && *count>=minimum_samples && now_ms-*since >= (int64_t)need_ms;
}
struct fc_profile_result { float z; bool landing; int64_t cap_ms; };
static inline struct fc_profile_result fc_flight_profile(int64_t now_ms, int64_t start_ms,
        int tc, int th, int td, float height, float land_push, int64_t flight_max_ms,
        int64_t abort_ms, float abort_z, float abort_speed)
{
    int64_t tf=now_ms-start_ms;
    struct fc_profile_result result;
    result.z=fc_profile_z(tf,tc,th,td,height,land_push);
    result.landing=tf>=(int64_t)(tc+th+td);
    result.cap_ms=flight_max_ms;
    if (abort_ms!=0) {
        const float ta=(float)(now_ms-abort_ms)*0.001f;
        float za=abort_z-abort_speed*ta;
        if (za<land_push) za=land_push;
        if (za<result.z) result.z=za;
        result.landing=result.landing || za<=0.0f;
        const int64_t need=(abort_ms-start_ms)+2000+
            (int64_t)((abort_z-land_push)/abort_speed*1000.0f);
        if (need>result.cap_ms) result.cap_ms=need;
    }
    return result;
}
#endif
