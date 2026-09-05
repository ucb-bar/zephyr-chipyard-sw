//
// Created by widyadewi on 4/27/24.
//

#ifndef TINYMPC_MATLIB_LMUL_H
#define TINYMPC_MATLIB_LMUL_H


#ifndef USE_LMUL
#define USE_LMUL 1
#endif

#if USE_LMUL == 1
#define __riscv_vfabs_v_f32 __riscv_vfabs_v_f32m1
#define __riscv_vfadd_vv_f32 __riscv_vfadd_vv_f32m1
#define __riscv_vfmacc_vv_f32 __riscv_vfmacc_vv_f32m1
#define __riscv_vfmax_vv_f32 __riscv_vfmax_vv_f32m1
#define __riscv_vfmin_vv_f32 __riscv_vfmin_vv_f32m1
#define __riscv_vfmul_vf_f32 __riscv_vfmul_vf_f32m1
#define __riscv_vfmul_vv_f32 __riscv_vfmul_vv_f32m1
#define __riscv_vfmv_f_s_f32_f32 __riscv_vfmv_f_s_f32m1_f32
#define __riscv_vfmv_s_f_f32 __riscv_vfmv_s_f_f32m1
#define __riscv_vfmv_v_f_f32 __riscv_vfmv_v_f_f32m1
#define __riscv_vfneg_v_f32 __riscv_vfneg_v_f32m1
#define __riscv_vfredmax_vs_f32_f32 __riscv_vfredmax_vs_f32m1_f32m1
#define __riscv_vfredmin_vs_f32_f32 __riscv_vfredmin_vs_f32m1_f32m1
#define __riscv_vfredusum_vs_f32_f32 __riscv_vfredusum_vs_f32m1_f32m1
#define __riscv_vfsub_vv_f32 __riscv_vfsub_vv_f32m1
#define __riscv_vle32_v_f32 __riscv_vle32_v_f32m1
#define __riscv_vlse32_v_f32 __riscv_vlse32_v_f32m1
#define __riscv_vse32_v_f32 __riscv_vse32_v_f32m1
#define __riscv_vsetvl_e32 __riscv_vsetvl_e32m1
#define __riscv_vsetvlmax_e32 __riscv_vsetvlmax_e32m1
#define vfloat32_t vfloat32m1_t
#elif USE_LMUL == 2
#define __riscv_vfabs_v_f32 __riscv_vfabs_v_f32m2
#define __riscv_vfadd_vv_f32 __riscv_vfadd_vv_f32m2
#define __riscv_vfmacc_vv_f32 __riscv_vfmacc_vv_f32m2
#define __riscv_vfmax_vv_f32 __riscv_vfmax_vv_f32m2
#define __riscv_vfmin_vv_f32 __riscv_vfmin_vv_f32m2
#define __riscv_vfmul_vf_f32 __riscv_vfmul_vf_f32m2
#define __riscv_vfmul_vv_f32 __riscv_vfmul_vv_f32m2
#define __riscv_vfmv_f_s_f32_f32 __riscv_vfmv_f_s_f32m2_f32
#define __riscv_vfmv_s_f_f32 __riscv_vfmv_s_f_f32m2
#define __riscv_vfmv_v_f_f32 __riscv_vfmv_v_f_f32m2
#define __riscv_vfneg_v_f32 __riscv_vfneg_v_f32m2
#define __riscv_vfredmax_vs_f32_f32 __riscv_vfredmax_vs_f32m2_f32m1
#define __riscv_vfredmin_vs_f32_f32 __riscv_vfredmin_vs_f32m2_f32m1
#define __riscv_vfredusum_vs_f32_f32 __riscv_vfredusum_vs_f32m2_f32m1
#define __riscv_vfsub_vv_f32 __riscv_vfsub_vv_f32m2
#define __riscv_vle32_v_f32 __riscv_vle32_v_f32m2
#define __riscv_vlse32_v_f32 __riscv_vlse32_v_f32m2
#define __riscv_vse32_v_f32 __riscv_vse32_v_f32m2
#define __riscv_vsetvl_e32 __riscv_vsetvl_e32m2
#define __riscv_vsetvlmax_e32 __riscv_vsetvlmax_e32m2
#define vfloat32_t vfloat32m2_t
#elif USE_LMUL == 4
#define __riscv_vfabs_v_f32 __riscv_vfabs_v_f32m4
#define __riscv_vfadd_vv_f32 __riscv_vfadd_vv_f32m4
#define __riscv_vfmacc_vv_f32 __riscv_vfmacc_vv_f32m4
#define __riscv_vfmax_vv_f32 __riscv_vfmax_vv_f32m4
#define __riscv_vfmin_vv_f32 __riscv_vfmin_vv_f32m4
#define __riscv_vfmul_vf_f32 __riscv_vfmul_vf_f32m4
#define __riscv_vfmul_vv_f32 __riscv_vfmul_vv_f32m4
#define __riscv_vfmv_f_s_f32_f32 __riscv_vfmv_f_s_f32m4_f32
#define __riscv_vfmv_s_f_f32 __riscv_vfmv_s_f_f32m4
#define __riscv_vfmv_v_f_f32 __riscv_vfmv_v_f_f32m4
#define __riscv_vfneg_v_f32 __riscv_vfneg_v_f32m4
#define __riscv_vfredmax_vs_f32_f32 __riscv_vfredmax_vs_f32m4_f32m1
#define __riscv_vfredmin_vs_f32_f32 __riscv_vfredmin_vs_f32m4_f32m1
#define __riscv_vfredusum_vs_f32_f32 __riscv_vfredusum_vs_f32m4_f32m1
#define __riscv_vfsub_vv_f32 __riscv_vfsub_vv_f32m4
#define __riscv_vle32_v_f32 __riscv_vle32_v_f32m4
#define __riscv_vlse32_v_f32 __riscv_vlse32_v_f32m4
#define __riscv_vse32_v_f32 __riscv_vse32_v_f32m4
#define __riscv_vsetvl_e32 __riscv_vsetvl_e32m4
#define __riscv_vsetvlmax_e32 __riscv_vsetvlmax_e32m4
#define vfloat32_t vfloat32m4_t
#elif USE_LMUL == 8
#define __riscv_vfabs_v_f32 __riscv_vfabs_v_f32m8
#define __riscv_vfadd_vv_f32 __riscv_vfadd_vv_f32m8
#define __riscv_vfmacc_vv_f32 __riscv_vfmacc_vv_f32m8
#define __riscv_vfmax_vv_f32 __riscv_vfmax_vv_f32m8
#define __riscv_vfmin_vv_f32 __riscv_vfmin_vv_f32m8
#define __riscv_vfmul_vf_f32 __riscv_vfmul_vf_f32m8
#define __riscv_vfmul_vv_f32 __riscv_vfmul_vv_f32m8
#define __riscv_vfmv_f_s_f32_f32 __riscv_vfmv_f_s_f32m8_f32
#define __riscv_vfmv_s_f_f32 __riscv_vfmv_s_f_f32m8
#define __riscv_vfmv_v_f_f32 __riscv_vfmv_v_f_f32m8
#define __riscv_vfneg_v_f32 __riscv_vfneg_v_f32m8
#define __riscv_vfredmax_vs_f32_f32 __riscv_vfredmax_vs_f32m8_f32m1
#define __riscv_vfredmin_vs_f32_f32 __riscv_vfredmin_vs_f32m8_f32m1
#define __riscv_vfredusum_vs_f32_f32 __riscv_vfredusum_vs_f32m8_f32m1
#define __riscv_vfsub_vv_f32 __riscv_vfsub_vv_f32m8
#define __riscv_vle32_v_f32 __riscv_vle32_v_f32m8
#define __riscv_vlse32_v_f32 __riscv_vlse32_v_f32m8
#define __riscv_vse32_v_f32 __riscv_vse32_v_f32m8
#define __riscv_vsetvl_e32 __riscv_vsetvl_e32m8
#define __riscv_vsetvlmax_e32 __riscv_vsetvlmax_e32m8
#define vfloat32_t vfloat32m8_t
#endif

// fp16 intrinsic aliases mirroring the f32 family above. Selected with the
// same USE_LMUL value so the LMUL choice is consistent across precisions.
#if USE_LMUL == 1
#define __riscv_vfabs_v_f16 __riscv_vfabs_v_f16m1
#define __riscv_vfadd_vv_f16 __riscv_vfadd_vv_f16m1
#define __riscv_vfmacc_vv_f16 __riscv_vfmacc_vv_f16m1
#define __riscv_vfmax_vv_f16 __riscv_vfmax_vv_f16m1
#define __riscv_vfmin_vv_f16 __riscv_vfmin_vv_f16m1
#define __riscv_vfmul_vf_f16 __riscv_vfmul_vf_f16m1
#define __riscv_vfmul_vv_f16 __riscv_vfmul_vv_f16m1
#define __riscv_vfmv_f_s_f16_f16 __riscv_vfmv_f_s_f16m1_f16
#define __riscv_vfmv_s_f_f16 __riscv_vfmv_s_f_f16m1
#define __riscv_vfmv_v_f_f16 __riscv_vfmv_v_f_f16m1
#define __riscv_vfneg_v_f16 __riscv_vfneg_v_f16m1
#define __riscv_vfredmax_vs_f16_f16 __riscv_vfredmax_vs_f16m1_f16m1
#define __riscv_vfredmin_vs_f16_f16 __riscv_vfredmin_vs_f16m1_f16m1
#define __riscv_vfredusum_vs_f16_f16 __riscv_vfredusum_vs_f16m1_f16m1
#define __riscv_vfsub_vv_f16 __riscv_vfsub_vv_f16m1
#define __riscv_vle16_v_f16 __riscv_vle16_v_f16m1
#define __riscv_vlse16_v_f16 __riscv_vlse16_v_f16m1
#define __riscv_vse16_v_f16 __riscv_vse16_v_f16m1
#define __riscv_vsetvl_e16 __riscv_vsetvl_e16m1
#define __riscv_vsetvlmax_e16 __riscv_vsetvlmax_e16m1
#define vfloat16_t vfloat16m1_t
#elif USE_LMUL == 2
#define __riscv_vfabs_v_f16 __riscv_vfabs_v_f16m2
#define __riscv_vfadd_vv_f16 __riscv_vfadd_vv_f16m2
#define __riscv_vfmacc_vv_f16 __riscv_vfmacc_vv_f16m2
#define __riscv_vfmax_vv_f16 __riscv_vfmax_vv_f16m2
#define __riscv_vfmin_vv_f16 __riscv_vfmin_vv_f16m2
#define __riscv_vfmul_vf_f16 __riscv_vfmul_vf_f16m2
#define __riscv_vfmul_vv_f16 __riscv_vfmul_vv_f16m2
#define __riscv_vfmv_f_s_f16_f16 __riscv_vfmv_f_s_f16m2_f16
#define __riscv_vfmv_s_f_f16 __riscv_vfmv_s_f_f16m2
#define __riscv_vfmv_v_f_f16 __riscv_vfmv_v_f_f16m2
#define __riscv_vfneg_v_f16 __riscv_vfneg_v_f16m2
#define __riscv_vfredmax_vs_f16_f16 __riscv_vfredmax_vs_f16m2_f16m1
#define __riscv_vfredmin_vs_f16_f16 __riscv_vfredmin_vs_f16m2_f16m1
#define __riscv_vfredusum_vs_f16_f16 __riscv_vfredusum_vs_f16m2_f16m1
#define __riscv_vfsub_vv_f16 __riscv_vfsub_vv_f16m2
#define __riscv_vle16_v_f16 __riscv_vle16_v_f16m2
#define __riscv_vlse16_v_f16 __riscv_vlse16_v_f16m2
#define __riscv_vse16_v_f16 __riscv_vse16_v_f16m2
#define __riscv_vsetvl_e16 __riscv_vsetvl_e16m2
#define __riscv_vsetvlmax_e16 __riscv_vsetvlmax_e16m2
#define vfloat16_t vfloat16m2_t
#elif USE_LMUL == 4
#define __riscv_vfabs_v_f16 __riscv_vfabs_v_f16m4
#define __riscv_vfadd_vv_f16 __riscv_vfadd_vv_f16m4
#define __riscv_vfmacc_vv_f16 __riscv_vfmacc_vv_f16m4
#define __riscv_vfmax_vv_f16 __riscv_vfmax_vv_f16m4
#define __riscv_vfmin_vv_f16 __riscv_vfmin_vv_f16m4
#define __riscv_vfmul_vf_f16 __riscv_vfmul_vf_f16m4
#define __riscv_vfmul_vv_f16 __riscv_vfmul_vv_f16m4
#define __riscv_vfmv_f_s_f16_f16 __riscv_vfmv_f_s_f16m4_f16
#define __riscv_vfmv_s_f_f16 __riscv_vfmv_s_f_f16m4
#define __riscv_vfmv_v_f_f16 __riscv_vfmv_v_f_f16m4
#define __riscv_vfneg_v_f16 __riscv_vfneg_v_f16m4
#define __riscv_vfredmax_vs_f16_f16 __riscv_vfredmax_vs_f16m4_f16m1
#define __riscv_vfredmin_vs_f16_f16 __riscv_vfredmin_vs_f16m4_f16m1
#define __riscv_vfredusum_vs_f16_f16 __riscv_vfredusum_vs_f16m4_f16m1
#define __riscv_vfsub_vv_f16 __riscv_vfsub_vv_f16m4
#define __riscv_vle16_v_f16 __riscv_vle16_v_f16m4
#define __riscv_vlse16_v_f16 __riscv_vlse16_v_f16m4
#define __riscv_vse16_v_f16 __riscv_vse16_v_f16m4
#define __riscv_vsetvl_e16 __riscv_vsetvl_e16m4
#define __riscv_vsetvlmax_e16 __riscv_vsetvlmax_e16m4
#define vfloat16_t vfloat16m4_t
#elif USE_LMUL == 8
#define __riscv_vfabs_v_f16 __riscv_vfabs_v_f16m8
#define __riscv_vfadd_vv_f16 __riscv_vfadd_vv_f16m8
#define __riscv_vfmacc_vv_f16 __riscv_vfmacc_vv_f16m8
#define __riscv_vfmax_vv_f16 __riscv_vfmax_vv_f16m8
#define __riscv_vfmin_vv_f16 __riscv_vfmin_vv_f16m8
#define __riscv_vfmul_vf_f16 __riscv_vfmul_vf_f16m8
#define __riscv_vfmul_vv_f16 __riscv_vfmul_vv_f16m8
#define __riscv_vfmv_f_s_f16_f16 __riscv_vfmv_f_s_f16m8_f16
#define __riscv_vfmv_s_f_f16 __riscv_vfmv_s_f_f16m8
#define __riscv_vfmv_v_f_f16 __riscv_vfmv_v_f_f16m8
#define __riscv_vfneg_v_f16 __riscv_vfneg_v_f16m8
#define __riscv_vfredmax_vs_f16_f16 __riscv_vfredmax_vs_f16m8_f16m1
#define __riscv_vfredmin_vs_f16_f16 __riscv_vfredmin_vs_f16m8_f16m1
#define __riscv_vfredusum_vs_f16_f16 __riscv_vfredusum_vs_f16m8_f16m1
#define __riscv_vfsub_vv_f16 __riscv_vfsub_vv_f16m8
#define __riscv_vle16_v_f16 __riscv_vle16_v_f16m8
#define __riscv_vlse16_v_f16 __riscv_vlse16_v_f16m8
#define __riscv_vse16_v_f16 __riscv_vse16_v_f16m8
#define __riscv_vsetvl_e16 __riscv_vsetvl_e16m8
#define __riscv_vsetvlmax_e16 __riscv_vsetvlmax_e16m8
#define vfloat16_t vfloat16m8_t
#endif

// Top-level matlib-float macro family. Resolves to the f32 or f16 intrinsic
// family depending on USE_TYPE_FP16, so the kernel bodies in matlib_rvv.h
// can be written precision-agnostic.
#ifdef USE_TYPE_FP16
/*
 * This core provides Zvfh (vector fp16) but traps on scalar Zfh (flh/fsh/
 * fmv.h/vfmv.[fs].f). So scalar-fp16 splat/extract are re-expressed as pure
 * vector-integer ops: splat via vmv.v.x on the fp16 bit pattern, and the
 * reduction result is stored straight out of a vector (see mfl_vfmv_f_s_m1).
 * mfl__f16bits() folds to an immediate for the compile-time constants the
 * kernels splat (0), so no scalar fp16 (fsh/fmv.x.h) is emitted.
 */
static inline uint16_t mfl__f16bits(_Float16 f){ uint16_t u; __builtin_memcpy(&u, &f, sizeof(u)); return u; }
#define mfl_vfloat_t              vfloat16_t
#define mfl_vfloat_m1_t           vfloat16m1_t
#define mfl_vsetvl(k)             __riscv_vsetvl_e16(k)
#define mfl_vsetvlmax()           __riscv_vsetvlmax_e16()
#define mfl_vle(p, vl)            __riscv_vle16_v_f16((p), (vl))
#define mfl_vlse(p, stride, vl)   __riscv_vlse16_v_f16((p), (stride), (vl))
#define mfl_vse(p, v, vl)         __riscv_vse16_v_f16((p), (v), (vl))
#define mfl_vfmv_s_f_m1(f, vl)    __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vmv_s_x_u16m1(mfl__f16bits(f), (vl)))
#define mfl_vfmv_v_f_m1(f, vl)    __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vmv_v_x_u16m1(mfl__f16bits(f), (vl)))
#define mfl_vfmv_v_f(f, vl)       __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vmv_v_x_u16m1(mfl__f16bits(f), (vl)))
/* Reached kernels (matvec/matmul/matvec_transpose) use mfl_store1 to write the
 * reduction result straight from the vector (no scalar fp16). vfmv_f_s is kept
 * for the residual-only helpers (maxcoeff/mincoeff/matnorm) which are not on the
 * timed path; its scalar flh sits unreached here. */
#define mfl_vfmv_f_s_m1(v)        __riscv_vfmv_f_s_f16m1_f16((v))
#define mfl_store1(p, v)          __riscv_vse16_v_f16((p), (v), 1)
#define mfl_vfneg(v, vl)          __riscv_vfneg_v_f16((v), (vl))
#define mfl_vfabs(v, vl)          __riscv_vfabs_v_f16((v), (vl))
#define mfl_vfadd_vv(a, b, vl)    __riscv_vfadd_vv_f16((a), (b), (vl))
#define mfl_vfsub_vv(a, b, vl)    __riscv_vfsub_vv_f16((a), (b), (vl))
#define mfl_vfmul_vv(a, b, vl)    __riscv_vfmul_vv_f16((a), (b), (vl))
#define mfl_vfmul_vf(a, f, vl)    __riscv_vfmul_vf_f16((a), (f), (vl))
#define mfl_vfmacc_vv(d, a, b, vl) __riscv_vfmacc_vv_f16((d), (a), (b), (vl))
#define mfl_vfmin_vv(a, b, vl)    __riscv_vfmin_vv_f16((a), (b), (vl))
#define mfl_vfmax_vv(a, b, vl)    __riscv_vfmax_vv_f16((a), (b), (vl))
#define mfl_vfredmax(a, s, vl)    __riscv_vfredmax_vs_f16_f16((a), (s), (vl))
#define mfl_vfredmin(a, s, vl)    __riscv_vfredmin_vs_f16_f16((a), (s), (vl))
#define mfl_vfredusum(a, s, vl)   __riscv_vfredusum_vs_f16_f16((a), (s), (vl))
#else
#define mfl_vfloat_t              vfloat32_t
#define mfl_vfloat_m1_t           vfloat32m1_t
#define mfl_vsetvl(k)             __riscv_vsetvl_e32(k)
#define mfl_vsetvlmax()           __riscv_vsetvlmax_e32()
#define mfl_vle(p, vl)            __riscv_vle32_v_f32((p), (vl))
#define mfl_vlse(p, stride, vl)   __riscv_vlse32_v_f32((p), (stride), (vl))
#define mfl_vse(p, v, vl)         __riscv_vse32_v_f32((p), (v), (vl))
#define mfl_vfmv_s_f_m1(f, vl)    __riscv_vfmv_s_f_f32m1((f), (vl))
#define mfl_vfmv_v_f_m1(f, vl)    __riscv_vfmv_v_f_f32m1((f), (vl))
#define mfl_vfmv_v_f(f, vl)       __riscv_vfmv_v_f_f32((f), (vl))
#define mfl_vfmv_f_s_m1(v)        __riscv_vfmv_f_s_f32m1_f32((v))
#define mfl_store1(p, v)          __riscv_vse32_v_f32((p), (v), 1)
#define mfl_vfneg(v, vl)          __riscv_vfneg_v_f32((v), (vl))
#define mfl_vfabs(v, vl)          __riscv_vfabs_v_f32((v), (vl))
#define mfl_vfadd_vv(a, b, vl)    __riscv_vfadd_vv_f32((a), (b), (vl))
#define mfl_vfsub_vv(a, b, vl)    __riscv_vfsub_vv_f32((a), (b), (vl))
#define mfl_vfmul_vv(a, b, vl)    __riscv_vfmul_vv_f32((a), (b), (vl))
#define mfl_vfmul_vf(a, f, vl)    __riscv_vfmul_vf_f32((a), (f), (vl))
#define mfl_vfmacc_vv(d, a, b, vl) __riscv_vfmacc_vv_f32((d), (a), (b), (vl))
#define mfl_vfmin_vv(a, b, vl)    __riscv_vfmin_vv_f32((a), (b), (vl))
#define mfl_vfmax_vv(a, b, vl)    __riscv_vfmax_vv_f32((a), (b), (vl))
#define mfl_vfredmax(a, s, vl)    __riscv_vfredmax_vs_f32_f32((a), (s), (vl))
#define mfl_vfredmin(a, s, vl)    __riscv_vfredmin_vs_f32_f32((a), (s), (vl))
#define mfl_vfredusum(a, s, vl)   __riscv_vfredusum_vs_f32_f32((a), (s), (vl))
#endif

#endif //TINYMPC_MATLIB_LMUL_H
