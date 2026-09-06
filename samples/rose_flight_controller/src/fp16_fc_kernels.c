/*
 * fp16 flight-controller kernels -- VECTOR unit (Zvfh) of the compile-unit split.
 *
 * Built with -march=rv64imac_zve64x_zvfh -mabi=lp64 -fno-tree-vectorize. Contains
 * ONLY the fp16-vector algebra + int<->fp16 vfcvt (SEW=16) + integer-vector ops --
 * ZERO scalar FP and ZERO blocked ops (no vfncvt.f.f.w / vfwcvt.f.f.v / fp32-vector),
 * so it runs on the fp16-only At35 Saturn (misa.F=0). The scalar-float interface,
 * scalar cascade, and int64 fixed-point accumulator bookkeeping live in the soft-float
 * GLUE unit (fp16_fc_glue.c). Each ve_/vc_ helper is a VERBATIM relocation of a
 * contiguous vector block from a1198297's single-file kernel @ ddcd5d3 (algebra and
 * Q-scales unchanged); data crosses via memory arrays + uint16 fp16-bit constants.
 *
 * Two documented deviations, both bit-exact:
 *   (1) hscale() multiplies by 2^m built as an INTEGER fp16 bit pattern (pow2h) rather
 *       than (_Float16)(float)(1<<m); 2^m is exactly representable so the fp16 value --
 *       hence the vfmul result -- is identical, but no runtime scalar float->fp16 cast.
 *   (2) rotation-matrix elements are extracted with hlane0() (a vse16 of lane 0 to
 *       _Float16) instead of (_Float16)h_get() whose (float)(_Float16) round-trip is an
 *       identity for an fp16 value -- same bits, no scalar float.
 */
#include "fp16_fc_kernels.h"
#include "fp16_fc_vec.h"
#include <stdint.h>
#include <riscv_vector.h>

typedef vfloat16m1_t vh;

/* ---- fp16 primitives (integer-bit splat; no scalar fp16; no .vf) ---- */
static inline vh h_splat(float c, size_t vl){ return __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vmv_v_x_u16m1(hb(c), vl)); } /* c COMPILE-TIME only */
static inline vh h_splat_bits(uint16_t b, size_t vl){ return __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vmv_v_x_u16m1(b, vl)); }
static inline vh h_load(const _Float16 *p, size_t vl){ return __riscv_vle16_v_f16m1(p, vl); }
static inline void h_store(_Float16 *p, vh v, size_t vl){ __riscv_vse16_v_f16m1(p, v, vl); }
static inline void hstore1(_Float16 *p, vh v){ __riscv_vse16_v_f16m1(p, v, 1); }  /* store lane0, vector vse16 (no scalar flh/fsh) */
static inline vh hmul(vh a, vh b, size_t vl){ return __riscv_vfmul_vv_f16m1(a, b, vl); }
static inline vh hadd(vh a, vh b, size_t vl){ return __riscv_vfadd_vv_f16m1(a, b, vl); }
static inline vh hsub(vh a, vh b, size_t vl){ return __riscv_vfsub_vv_f16m1(a, b, vl); }
static inline vh hmulc(vh v, float c, size_t vl){ return __riscv_vfmul_vv_f16m1(v, h_splat(c, vl), vl); } /* c COMPILE-TIME */
static inline vh haddc(vh v, float c, size_t vl){ return __riscv_vfadd_vv_f16m1(v, h_splat(c, vl), vl); }
static inline vh hsubc(vh v, float c, size_t vl){ return __riscv_vfsub_vv_f16m1(v, h_splat(c, vl), vl); }
static inline vh hmaxc(vh v, float c, size_t vl){ return __riscv_vfmax_vv_f16m1(v, h_splat(c, vl), vl); }
static inline vh hminc(vh v, float c, size_t vl){ return __riscv_vfmin_vv_f16m1(v, h_splat(c, vl), vl); }
static inline vh habs(vh v, size_t vl){ return __riscv_vfabs_v_f16m1(v, vl); }
static inline vh hsqrt(vh v, size_t vl){ return __riscv_vfsqrt_v_f16m1(v, vl); }
static inline vh hshuf(vh v, const uint16_t idx[], size_t vl){ return __riscv_vrgather_vv_f16m1(v, __riscv_vle16_v_u16m1(idx, vl), vl); }
static inline vh h_bcast(vh v, int k, size_t vl){ return __riscv_vrgather_vx_f16m1(v, (size_t)k, vl); }
static inline vh hrecip(vh v, size_t vl){ return __riscv_vfdiv_vv_f16m1(h_splat(1.0f, vl), v, vl); }
static inline vh hfmacc(vh acc, vh a, vh b, size_t vl){ return __riscv_vfmacc_vv_f16m1(acc, a, b, vl); }
static inline vh h_dot_bcast(vh a, vh b, size_t n){
    vh r = __riscv_vfredusum_vs_f16m1_f16m1(__riscv_vfmul_vv_f16m1(a, b, n), h_splat(0.0f, n), n);
    return __riscv_vrgather_vx_f16m1(r, 0, n);
}
/* int16<->fp16 via SEW=16 vfcvt (supported on At35 Saturn) */
static inline void h_to_i16(int16_t *o, vh v, size_t vl){ __riscv_vse16_v_i16m1(o, __riscv_vfcvt_x_f_v_i16m1(v, vl), vl); }
static inline vh i16_to_h(const int16_t *p, size_t vl){ return __riscv_vfcvt_f_x_v_f16m1(__riscv_vle16_v_i16m1(p, vl), vl); }

/* 2^m as an fp16 bit pattern (integer; exact for -14<=m<=15). */
static inline uint16_t pow2h(int m){ return (uint16_t)((unsigned)(15 + m) << 10); }
/* scale by 2^m (exact power-of-2, split into 2^15 chunks when |m|>15). Works with a
 * RUNTIME m without any scalar float (deviation (1)). Only m in [-14,+22] is used. */
static vh hscale(vh v, int m, size_t vl){
    while (m > 15){ v = hmul(v, h_splat_bits(pow2h(15), vl), vl); m -= 15; }   /* *2^15 */
    while (m < -14){ v = hmul(v, h_splat_bits(pow2h(-14), vl), vl); m += 14; } /* *2^-14 (dead: min m=-14) */
    if (m != 0) v = hmul(v, h_splat_bits(pow2h(m), vl), vl);
    return v;
}
/* int64-fixed -> fp16 helper mirrors fixed_to_h's vector part: hscale(i16_to_h(hi), -Qm). */
static inline vh fixed_h(const int16_t *hi, int negQm, size_t vl){ return hscale(i16_to_h(hi, vl), negQm, vl); }

/* Q-scales + increment scales (validated in fc_i32.c) -- verbatim from the source. */
#define Q_QUAT 30
#define Q_VEL  28
#define Q_RATE 25
#define M_QUAT 18
#define M_ALPHA 7
#define KP 0.5f
#define J0 16e-6f
#define J1 16e-6f
#define J2 29e-6f
#define LARM 33e-3f
#define KDRAG 0.01f
#define TA 26.919633f
#define TB 35.754861f
#define PROP 1.33f
#define GPN 101.9368f
#define MOT 4.0f
#ifndef FCLAMP
#define FCLAMP 600.0f
#endif
#define TR 0.10f
#define TY 0.25f
#define TRR 0.025f
#define TYR 0.05f

/* ================= estimate vector blocks ================= */

void ve_alpha_inc(int16_t *inc_i16, const _Float16 *gv, const int16_t *wprev_i16,
                  const int16_t *alpha_i16, uint16_t invdt_bits, uint16_t k_bits, int n){
    size_t vl = (size_t)n;
    vh g = h_load(gv, vl);
    vh wp = fixed_h(wprev_i16, -10, vl);
    vh af = fixed_h(alpha_i16, -10, vl);
    vh deriv = hmul(hsub(g, wp, vl), h_splat_bits(invdt_bits, vl), vl);
    vh inc = hmul(hsub(deriv, af, vl), h_splat_bits(k_bits, vl), vl);
    h_to_i16(inc_i16, hscale(inc, M_ALPHA, vl), vl);
}

void ve_lever_pr(_Float16 *pr_out, _Float16 *al_out, const _Float16 *gv, const int16_t *alpha_i16){
    size_t v3 = 3, v4 = 4;
    static const uint16_t gi[4] = {0,1,2,2}, hi[4] = {1,2,0,2};   /* [gx,gy,gz,gz]*[gy,gz,gx,gz] */
    vh g = h_load(gv, v4);                                        /* gv[3]==0 (glue pads) */
    vh al = fixed_h(alpha_i16, -10, v3);
    vh pr = hmul(hshuf(g, gi, v4), hshuf(g, hi, v4), v4);
    h_store(pr_out, pr, v4);
    h_store(al_out, al, v3);
}

void ve_av_sub(_Float16 *av_io, const _Float16 *cc, int n){
    size_t vl = (size_t)n;
    h_store(av_io, hsub(h_load(av_io, vl), h_load(cc, vl), vl), vl);
}

void ve_mahony(_Float16 *w3_out, const _Float16 *av_p, const int16_t *q_i16, const _Float16 *gv, int n){
    size_t v3 = (size_t)n, v4 = 4;
    vh av = h_load(av_p, v3);
    vh qh = fixed_h(q_i16, -14, v4);
    vh w3 = h_load(gv, v3);
    vh amag = hsqrt(h_dot_bcast(av, av, v3), v3);
    vh d = habs(hsubc(amag, 9.81f, v3), v3);
    vh gate = hmaxc(haddc(hmulc(d, -1.0f/(0.5f*9.81f), v3), 1.0f, v3), 0.0f, v3);
    vh inv = hrecip(hmaxc(amag, 1e-3f, v3), v3);
    vh an = hmul(av, inv, v3);
    static const uint16_t iA[4] = {1,2,1,0}, iB[4] = {3,3,1,0}, iC[4] = {0,0,2,0}, iD[4] = {2,1,2,0};
    vh pA = hmul(hshuf(qh, iA, v3), hshuf(qh, iB, v3), v3);
    vh pC = hmul(hshuf(qh, iC, v3), hshuf(qh, iD, v3), v3);
    static const _Float16 sg[4] = {(_Float16)-1,(_Float16)1,(_Float16)1,0};
    static const _Float16 ml[4] = {(_Float16)2,(_Float16)2,(_Float16)-2,0};
    static const _Float16 ad[4] = {(_Float16)0,(_Float16)0,(_Float16)1,0};
    vh t = hfmacc(pA, pC, h_load(sg, v3), v3);
    vh vu = hfmacc(h_load(ad, v3), t, h_load(ml, v3), v3);
    static const uint16_t c1[4] = {1,2,0,0}, c2[4] = {2,0,1,0};
    vh e = hsub(hmul(hshuf(an, c1, v3), hshuf(vu, c2, v3), v3), hmul(hshuf(an, c2, v3), hshuf(vu, c1, v3), v3), v3);
    w3 = hfmacc(w3, hmulc(e, KP, v3), gate, v3);
    h_store(w3_out, w3, v3);
}

void ve_quat_dq_i16(int16_t *dq_i16, const int16_t *q_i16, const _Float16 *w3p,
                    uint16_t halfdt_bits, int n){
    size_t v4 = (size_t)n;
    vh qh = fixed_h(q_i16, -14, v4);
    vh w3 = h_load(w3p, v4);                                      /* w3p[3]==0 (glue pads) */
    static const uint16_t p0[4] = {1,0,3,2}, p1[4] = {2,3,0,1}, p2[4] = {3,2,1,0};
    static const _Float16 s0[4] = {(_Float16)-1,(_Float16)1,(_Float16)1,(_Float16)-1};
    static const _Float16 s1[4] = {(_Float16)-1,(_Float16)-1,(_Float16)1,(_Float16)1};
    static const _Float16 s2[4] = {(_Float16)-1,(_Float16)1,(_Float16)-1,(_Float16)1};
    vh a0 = hmul(hshuf(qh, p0, v4), h_load(s0, v4), v4);
    vh a1 = hmul(hshuf(qh, p1, v4), h_load(s1, v4), v4);
    vh a2 = hmul(hshuf(qh, p2, v4), h_load(s2, v4), v4);
    vh dq = hmul(a0, h_bcast(w3, 0, v4), v4);
    dq = hfmacc(dq, a1, h_bcast(w3, 1, v4), v4);
    dq = hfmacc(dq, a2, h_bcast(w3, 2, v4), v4);
    dq = hmul(dq, h_splat_bits(halfdt_bits, v4), v4);
    h_to_i16(dq_i16, hscale(dq, M_QUAT, v4), v4);
}

void ve_quat_inv_i16(int16_t *invi, const int16_t *n16){
    vh inv = hrecip(hsqrt(hscale(i16_to_h(n16, 1), -14, 1), 1), 1);
    h_to_i16(invi, hscale(inv, 14, 1), 1);
}

void ve_rotmat(_Float16 *R, const int16_t *q_i16){
    size_t v4 = 4;
    vh q = fixed_h(q_i16, -14, v4);
    vh qx_ = h_bcast(q, 1, v4), qy_ = h_bcast(q, 2, v4), qz_ = h_bcast(q, 3, v4), qw_ = h_bcast(q, 0, v4);
    vh two = h_splat(2.0f, v4), one = h_splat(1.0f, v4);
    vh qxx = hmul(qx_, qx_, v4), qyy = hmul(qy_, qy_, v4), qzz = hmul(qz_, qz_, v4);
    hstore1(&R[0], __riscv_vfnmsac_vv_f16m1(one, two, hadd(qyy, qzz, v4), v4));
    hstore1(&R[4], __riscv_vfnmsac_vv_f16m1(one, two, hadd(qxx, qzz, v4), v4));
    hstore1(&R[8], __riscv_vfnmsac_vv_f16m1(one, two, hadd(qxx, qyy, v4), v4));
    { vh xy = hmul(qx_, qy_, v4), wz = hmul(qw_, qz_, v4), xz = hmul(qx_, qz_, v4),
         wy = hmul(qw_, qy_, v4), yz = hmul(qy_, qz_, v4), wx = hmul(qw_, qx_, v4);
      hstore1(&R[1], hmul(two, hsub(xy, wz, v4), v4)); hstore1(&R[2], hmul(two, hadd(xz, wy, v4), v4));
      hstore1(&R[3], hmul(two, hadd(xy, wz, v4), v4)); hstore1(&R[5], hmul(two, hsub(yz, wx, v4), v4));
      hstore1(&R[6], hmul(two, hsub(xz, wy, v4), v4)); hstore1(&R[7], hmul(two, hadd(yz, wx, v4), v4)); }
}

void ve_h_scale_to_i16(int16_t *out, const _Float16 *in, int m, int n){
    size_t i = 0, rem = (size_t)n;
    while (rem > 0){ size_t vl = __riscv_vsetvl_e16m1(rem); h_to_i16(out + i, hscale(h_load(in + i, vl), m, vl), vl); i += vl; rem -= vl; }
}

void ve_rflow_i16(int16_t *out_i16, const _Float16 *rrow, const _Float16 *fv){
    size_t v2 = 2;
    h_to_i16(out_i16, hscale(h_dot_bcast(h_load(rrow, v2), h_load(fv, v2), v2), Q_VEL - 16, 1), 1);
}

/* ================= control vector blocks ================= */

void vc_cos2(_Float16 *cr_cp, uint16_t roll_bits, uint16_t pitch_bits){
    size_t v2 = 2;
    uint16_t xbits[2] = { roll_bits, pitch_bits };   /* integer moves (no scalar flh/fsh) */
    vh x = __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vle16_v_u16m1(xbits, v2));
    vh x2 = hmul(x, x, v2), x4 = hmul(x2, x2, v2);
    vh cc = hadd(hsub(h_splat(1.0f, v2), hmulc(x2, 0.5f, v2), v2), hmulc(x4, 1.0f/24.0f, v2), v2);
    h_store(cr_cp, cc, v2);
}

void vc_inc(int16_t *inc_i16, const uint16_t *a_bits, const uint16_t *b_bits, int m, int n){
    /* a is a fp16 vector (n lanes) passed as bit patterns; b is a single splat constant. */
    size_t vl = (size_t)n;
    vh a = __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vle16_v_u16m1(a_bits, vl));
    vh inc = hmul(a, h_splat_bits(b_bits[0], vl), vl);
    h_to_i16(inc_i16, hscale(inc, m, vl), vl);
}

void vc_attitude(_Float16 *cm_out, const _Float16 *est3, const _Float16 *tgt3, const _Float16 *g3){
    size_t v3 = 3;
    static const _Float16 tauA[4] = {(_Float16)(-1.0f/TR),(_Float16)(-1.0f/TR),(_Float16)(-1.0f/TY),0};
    static const _Float16 tauB[4] = {(_Float16)(-1.0f/TRR),(_Float16)(-1.0f/TRR),(_Float16)(-1.0f/TYR),0};
    vh ratetgt = hmul(hsub(h_load(est3, v3), h_load(tgt3, v3), v3), h_load(tauA, v3), v3);
    vh cmd = hmul(hsub(h_load(g3, v3), ratetgt, v3), h_load(tauB, v3), v3);
    h_store(cm_out, cmd, v3);
}

void vc_mixer_force(_Float16 *u_out, const _Float16 *cm_p, uint16_t desnormmass_bits){
    size_t v3 = 3, v4 = 4;
    vh cmd = h_load(cm_p, v3);
    static const _Float16 Jv[4] = {(_Float16)J0,(_Float16)J1,(_Float16)J2,0};
    /* ut = cm*J as uint16 bit patterns (vector store); ub built by integer moves -> u vector. */
    uint16_t ut_bits[4], ub_bits[4];
    __riscv_vse16_v_u16m1(ut_bits, __riscv_vreinterpret_v_f16m1_u16m1(hmul(cmd, h_load(Jv, v3), v3)), v3);
    ub_bits[0] = desnormmass_bits; ub_bits[1] = ut_bits[0]; ub_bits[2] = ut_bits[1]; ub_bits[3] = ut_bits[2];
    vh u = __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vle16_v_u16m1(ub_bits, v4));
    static const _Float16 Mc0[4] = {(_Float16)0.25f,(_Float16)0.25f,(_Float16)0.25f,(_Float16)0.25f};
    static const _Float16 Mc1[4] = {(_Float16)(0.25f/LARM),(_Float16)(-0.25f/LARM),(_Float16)(-0.25f/LARM),(_Float16)(0.25f/LARM)};
    static const _Float16 Mc2[4] = {(_Float16)(-0.25f/LARM),(_Float16)(-0.25f/LARM),(_Float16)(0.25f/LARM),(_Float16)(0.25f/LARM)};
    static const _Float16 Mc3[4] = {(_Float16)(0.25f/KDRAG),(_Float16)(-0.25f/KDRAG),(_Float16)(0.25f/KDRAG),(_Float16)(-0.25f/KDRAG)};
    vh ctrl = hmul(h_load(Mc0, v4), h_bcast(u, 0, v4), v4);
    ctrl = hfmacc(ctrl, h_load(Mc1, v4), h_bcast(u, 1, v4), v4);
    ctrl = hfmacc(ctrl, h_load(Mc2, v4), h_bcast(u, 2, v4), v4);
    ctrl = hfmacc(ctrl, h_load(Mc3, v4), h_bcast(u, 3, v4), v4);
    static const uint16_t perm[4] = {1,2,3,0};
    static const _Float16 scl[4] = {(_Float16)0.9f,(_Float16)0.9f,(_Float16)(0.9f*0.87f),(_Float16)(0.9f*0.87f)};
    vh in = hmul(hshuf(ctrl, perm, v4), h_load(scl, v4), v4);
    in = __riscv_vfmax_vv_f16m1(in, h_splat(0.0f, v4), v4);
    vh tt = hminc(hmulc(in, GPN*MOT/PROP, v4), FCLAMP, v4);
    vh disc = hminc(haddc(hmulc(tt, 4.0f*TA, v4), TB*TB, v4), 60000.0f, v4);
    vh dd = hmulc(hsubc(hsqrt(disc, v4), TB, v4), 1.0f/(2.0f*TA), v4);
    dd = __riscv_vfmax_vv_f16m1(__riscv_vfmin_vv_f16m1(dd, h_splat(1.0f, v4), v4), h_splat(0.0f, v4), v4);
    dd = hsubc(dd, 0.583f, v4);
    h_store(u_out, dd, v4);
}
