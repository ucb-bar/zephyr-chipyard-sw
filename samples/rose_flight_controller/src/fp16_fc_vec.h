/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * INTERNAL interface between the soft-float ORCHESTRATOR unit (fp16_fc_glue.c, built
 * -march=rv64imac -mabi=lp64 -> all scalar float is a soft-float libcall, no scalar-FP
 * trap on the misa.F=0 core) and the Zvfh VECTOR unit (fp16_fc_kernels.c, built
 * -march=rv64imafc_zve64d_zvfh -mabi=lp64 -> pure vector fp16/fp32, ZERO scalar FP).
 *
 * The two units MUST NOT be merged: any zve/zvfh -march makes gcc emit scalar-FP for a
 * plain `float` op (fadd.s/flw/fcvt.s.h ...), which traps on this core. So all scalar
 * float lives in the glue unit; the vector unit only touches vectors, and scalars cross
 * the boundary as MEMORY arrays (float[]/_Float16[]) or integer BIT-PATTERNS (uint16/32)
 * -- never as a scalar float in a register.
 *
 * The vector helpers are verbatim relocations of the contiguous vector-intrinsic blocks
 * of the original fp16_fc_kernels.c; the few places that originally extracted/rebuilt an
 * fp16 lane through a scalar `_Float16` (which would emit flh/fsh) are re-expressed with
 * vector ops (vse16 vl=1, vslideup, indexed load) that yield the identical value.
 */
#ifndef FP16_FC_VEC_H
#define FP16_FC_VEC_H
#include <stdint.h>

/* --- estimator vector kernels (fp16_fc_kernels.c) --- */
/* gyro-derivative low-pass (fp32 vector FMA): alpha_f += k*((g_cur-w_prev)/dt - alpha_f) */
void ve_af_lp(const float g_cur[3], const float w_prev[3], float alpha_f[3],
              uint32_t k_bits, uint32_t invdt_bits);
/* |accel| (fp16) -> returned as an fp16 bit-pattern (widen in glue for the gate test) */
uint16_t ve_amag(const float acc_corr[3]);
/* Mahony body-rate: w3 = g_cur (+ KP*gate * (accel_norm x predicted_up) when do_trim) */
void ve_mahony_w3(const float acc_corr[3], const float q[4], const float g_cur[3],
                  uint32_t kpgate_bits, int do_trim, float w3_out[3]);
/* quaternion integrate + normalize (fp32 vector): q += 0.5*step*Omega(w3)*q; q/=|q| */
void ve_quat_integrate(float q[4], const float w3_arr[3], uint32_t step_bits);
/* rotation matrix (fp16) + world accel (R*a - g, fp32) + body->world flow velocity (fp16).
 * Returns R[8] (the tilt cosine) as an fp16 bit-pattern; vf01 is valid iff the caller set
 * flow_f16 to a real sample (glue only consumes it when flow_valid). */
void ve_rot_wa_flow(const float q[4], const float acc_corr[3], const _Float16 flow_f16[2],
                    float aw_out[3], float vf01_out[2], uint16_t *r8_bits_out);
/* velocity predict (fp32 vector FMA): vvel = vel + aw*dt */
void ve_vel_predict(const float vel[3], const float aw[3], uint32_t dt_bits, float vvel_out[3]);
/* horizontal-velocity backstop clamp (fp32 vector, +-VMAXV) */
void ve_clampV(float v[3]);
/* position integrate (fp32 vector FMA): pos += vvel*dt */
void ve_pos_integrate(float pos[3], const float vvel[3], uint32_t dt_bits);
/* get_state lead (fp32 vector FMA): so_pos = pos + vel*hl ; so_vel = vel + aw*hl */
void ve_lead_out(const float pos[3], const float vel[3], const float aw[3],
                 uint32_t hl_bits, float so_pos[3], float so_vel[3]);

/* --- controller vector kernels (fp16_fc_kernels.c) --- */
/* cos polynomial (fp16 vector) for the two angles at once -> cr_cp[0..1] (fp32) */
void vc_cos2(const _Float16 rp_f16[2], float cr_cp_out[2]);
/* horizontal velocity->attitude loop (fp32 vector FMA + clamp + slew) -> desRP[0..1] */
void vc_velloop(const float desV[2], const float vv[2], float vel_int[2],
                uint32_t kivdt_bits, int grounded, const float prev[2],
                uint32_t dmax_bits, float desRP_out[2]);
/* attitude+rate loops, 4x4 mixer, force->duty (all fp16 vector) -> u_out[0..3] (fp32).
 * tgt3_f32 = [desRoll, desPitch, yawTgt]; u0_f16bits = fp16 bits of desNorm*MASS. */
void vc_attitude(const float state[12], const float tgt3_f32[3], uint16_t u0_f16bits,
                 float u_out[4]);

#endif /* FP16_FC_VEC_H */
