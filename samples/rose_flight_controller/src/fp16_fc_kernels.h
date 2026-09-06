/*
 * Production fp16 flight-controller kernels (RVV Zvfh).
 *
 * fp16 (Zvfh vector) for the instantaneous algebra; fp32 (native vector, vfLen=64 on this Saturn
 * core — confirmed) for the drift-sensitive accumulators. All floating math is VECTOR (fp16 or
 * fp32) or integer — NO scalar FP instruction is emitted, so it is safe on the misa.F=0 core
 * (scalar F/D trap; scalar float in C would only be soft-float libcalls). Saturation clamps guard
 * the fp16 mixer/force path against overflow->NaN.
 *
 * Same interface as the harness fc.h AND wrapped by the FC's IStateEstimator/IController classes,
 * so the identical kernels are validated off-board and deployed on-target.
 */
#ifndef FP16_FC_KERNELS_H
#define FP16_FC_KERNELS_H
#include <stdint.h>

#define KEST_NSTATES 12
#define KCTRL_NACTIONS 4

typedef struct {
    /* fp32 accumulators (drift-sensitive; kept out of fp16) */
    float q[4];          /* Mahony quaternion (body->world) */
    float vel[3];        /* world velocity */
    float pos[3];        /* world position */
    float alpha_f[3];    /* gyro-derivative LP (lever-arm) */
    float w_prev[3];
    float alt_int, vel_int_1, vel_int_2;
    float desRoll_prev, desPitch_prev;
    /* carried between estimate() and get_state within a tick */
    float g_cur[3];      /* last gyro (body rates) */
    float aw[3];         /* last world accel */
    float dt_last;
    int   have_wprev;
} kfc_state;

void kfc_init(kfc_state *s, float x0, float y0, float z0);
/* estimator update + get_state -> fills state_out[12] */
void kfc_estimate(kfc_state *s, const float accel[3], const float gyro[3],
                  const float flow[2], int flow_valid, float height, int tof_valid,
                  float dt, float state_out[KEST_NSTATES]);
/* hierarchical PID -> u_out[4] (normalized-thrust convention) */
void kfc_control(kfc_state *s, const float state[KEST_NSTATES],
                 const float setpoint[KEST_NSTATES], float u_out[KCTRL_NACTIONS], float dt);

#endif
