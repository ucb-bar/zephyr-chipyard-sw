/*
 * Internal interface between the fp16 flight-controller GLUE unit (fp16_fc_glue.c,
 * soft-float rv64imac) and the VECTOR unit (fp16_fc_kernels.c, Zvfh). The compile-
 * unit split exists because the fp16-only At35 Saturn (misa.F=0) traps on scalar
 * float: the glue unit does the float interface + scalar-float cascade as soft-float
 * libcalls, the vec unit does the fp16-vector algebra + int<->fp16 vfcvt with ZERO
 * scalar FP. Data crosses via memory (fp16 as _Float16[], ints as int16/int32[]);
 * runtime fp16 constants cross as uint16_t bit patterns (glue computes them soft-
 * float; the vec unit splats them via vmv.v.x -- never a scalar float->fp16 convert).
 *
 * Every vec helper is a VERBATIM relocation of a contiguous vector block from the
 * original single-file kernel (a1198297 @ ddcd5d3); the algebra/Q-scales are unchanged.
 */
#ifndef FP16_FC_VEC_H
#define FP16_FC_VEC_H
#include <stdint.h>

/* fp16-bit-pattern of a float. In the GLUE unit (soft-float) this lowers to a
 * __truncsfhf2 libcall (safe); in the VEC unit it is only ever called with
 * compile-time constants (h_splat), which fold -- never a runtime scalar convert. */
static inline uint16_t hb(float c){ union { _Float16 h; uint16_t u; } x; x.h = (_Float16)c; return x.u; }

/* ---- estimate ---- */
/* lever-arm alpha LP increment: out int16 = h_to_i16(hscale((( gv-wp )*invdt - af)*k, M_ALPHA)),
 * with wp/af = hscale(i16_to_h(wprev_i16/alpha_i16), -10). Glue does <<(Q_RATE-M_ALPHA)+accumulate. */
void ve_alpha_inc(int16_t *inc_i16, const _Float16 *gv, const int16_t *wprev_i16,
                  const int16_t *alpha_i16, uint16_t invdt_bits, uint16_t k_bits, int n);
/* gyro products pr=[gxgy,gygz,gz2,gx2] (fp16) and al=fixed_to_h(alpha,Q_RATE,10) (fp16). */
void ve_lever_pr(_Float16 *pr_out, _Float16 *al_out, const _Float16 *gv, const int16_t *alpha_i16);
/* av -= cc  (fp16, 3-lane) */
void ve_av_sub(_Float16 *av_io, const _Float16 *cc, int n);
/* Mahony gravity-trim: w3_out = gv + KP*max(0,1-|(|a|-g)|/(0.5g)) * (an x vu). qh=fixed_to_h(q,Q_QUAT,14). */
void ve_mahony(_Float16 *w3_out, const _Float16 *av, const int16_t *q_i16, const _Float16 *gv, int n);
/* quaternion integrate: dq_i16 = h_to_i16(hscale(0.5*dt * Omega(w3)*qh, M_QUAT)); qh=fixed_to_h(q). */
void ve_quat_dq_i16(int16_t *dq_i16, const int16_t *q_i16, const _Float16 *w3,
                    uint16_t halfdt_bits, int n);
/* normalize inverse: invi = h_to_i16(hscale(1/sqrt(i16_to_h(n16)*2^-14), 14)). */
void ve_quat_inv_i16(int16_t *invi, const int16_t *n16);
/* rotation matrix R[9] (fp16) from qh=fixed_to_h(q,Q_QUAT,14). */
void ve_rotmat(_Float16 *R9, const int16_t *q_i16);
/* h[n] -> int16[n] with hscale(.,m) then vfcvt.x.f (used for R*2^14, a*2^10). */
void ve_h_scale_to_i16(int16_t *out, const _Float16 *in, int m, int n);
/* one flow row dot: out_i16 = h_to_i16(hscale(dot(rrow,fv), Q_VEL-16)). */
void ve_rflow_i16(int16_t *out_i16, const _Float16 *rrow, const _Float16 *fv);

/* ---- control ---- */
/* cos poly cr,cp = 1 - x^2/2 + x^4/24 for x=[roll,pitch] (fp16 bits in). */
void vc_cos2(_Float16 *cr_cp, uint16_t roll_bits, uint16_t pitch_bits);
/* integrator increment: inc_i16 = h_to_i16(hscale((a*b)[n], m)); a,b fp16 bit arrays. */
void vc_inc(int16_t *inc_i16, const uint16_t *a_bits, const uint16_t *b_bits, int m, int n);
/* attitude+rate: cm = ((g3 - tauA*(est3-tgt3)))*tauB   (fp16 3-lane). */
void vc_attitude(_Float16 *cm_out, const _Float16 *est3, const _Float16 *tgt3, const _Float16 *g3);
/* mixer (4x4) + force->duty (incl. vfsqrt) -> u_out[4] (fp16). u=[desNorm*MASS, cm*J]. */
void vc_mixer_force(_Float16 *u_out, const _Float16 *cm, uint16_t desnormmass_bits);

#endif /* FP16_FC_VEC_H */
