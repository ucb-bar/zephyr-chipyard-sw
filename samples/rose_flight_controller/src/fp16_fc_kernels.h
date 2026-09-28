/*
 * Production fp16 flight-controller kernels for the fp16-vector-ONLY Saturn (At35 bitstream).
 *
 * Instantaneous ALGEBRA in fp16 vector (Zvfh); drift-sensitive ACCUMULATORS in int fixed-point
 * (int64) so they run on a core that has NO fp32-vector and NO fp16<->fp32 converts. The gravity
 * chain (R*a - g -> vel -> pos) is entirely integer. fp16<->int crossings use only the SEW=16
 * vfcvt.x.f / vfcvt.f.x converts. Precision validated in fc_i32.c (~fp32 quality). See fp16_fc_kernels.c.
 *
 * Same interface as the harness fc.h AND wrapped by the FC's IStateEstimator/IController classes.
 */
#ifndef FP16_FC_KERNELS_H
#define FP16_FC_KERNELS_H
#include <stdint.h>

#define KEST_NSTATES 12
#define KCTRL_NACTIONS 4

/* int fixed-point accumulators (Q-scales in the .c): quat Q30, vel Q28, pos Q27, alpha Q25,
 * alt_int Q29, vel_int Q30, tilt slew Q31, gyro-prev Q25. Transient carries (g_cur/aw/dt) are
 * plain copies/loads (no fp arithmetic on them at the target). */
typedef struct {
    int64_t q[4];          /* quaternion, Q30 */
    int64_t vel[3];        /* world velocity, Q28 */
    int64_t pos[3];        /* world position, Q27 */
    int64_t alpha[3];      /* gyro-derivative LP, Q25 */
    int64_t wprev[3];      /* previous gyro, Q25 */
    int64_t alt_int;       /* altitude integrator, Q29 */
    int64_t vel_int[2];    /* velocity integrators, Q30 */
    int64_t desprev[2];    /* tilt slew memory, Q31 */
    float   g_cur[3];      /* last gyro (body rates) — copied to state_out */
    float   aw[3];         /* last world accel (for the lead) */
    float   dt_last;
    int     have_wprev;
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
