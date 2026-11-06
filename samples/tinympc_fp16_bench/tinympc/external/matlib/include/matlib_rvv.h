#pragma once
#ifndef TINYMPC_MATLIB_RVV_H
#define TINYMPC_MATLIB_RVV_H

#include <float.h>

#include "riscv_vector.h"
#include "matlib_types.h"

extern "C"
{

// RVV-specific function declarations
tinytype maxcoeff_rvv(tinytype *a, int n, int m);
tinytype mincoeff_rvv(tinytype *a, int n, int m);
tinytype matnorm_rvv(tinytype *a, int n, int m);
void matneg_rvv(tinytype *a, tinytype *b, int n, int m);
void cwiseabs_rvv(tinytype *a, tinytype *b, int n, int m);
void cwisemin_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void cwisemax_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void cwisemul_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void matmul_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m, int o);
void matvec_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void matvec_transpose_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void matmulf_rvv(tinytype *a, tinytype *b, tinytype f, int n, int m);
void matsub_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void matadd_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void transpose_rvv(tinytype *a, tinytype *b, int n, int m);
void matcopy_rvv(const tinytype *a, tinytype *b, int n, int m);
void matset_rvv(tinytype *a, tinytype f, int n, int m);
void matsetv_rvv(tinytype *a, tinytype *f, int n, int m);

#define maxcoeff maxcoeff_rvv
#define mincoeff mincoeff_rvv
#define matnorm matnorm_rvv
#define matneg matneg_rvv
#define cwiseabs cwiseabs_rvv
#define cwisemin cwisemin_rvv
#define cwisemax cwisemax_rvv
#define cwisemul cwisemul_rvv
#define matmul matmul_rvv
#define matvec matvec_rvv
#define matvec_transpose matvec_transpose_rvv
#define matmulf matmulf_rvv
#define matsub matsub_rvv
#define matadd matadd_rvv
#define transpose transpose_rvv
#define matcopy matcopy_rvv
#define matset matset_rvv
#define matsetv matsetv_rvv


// matrix maximum coefficient
inline tinytype maxcoeff_rvv(tinytype *ptr_a, int n, int m) {
    tinytype max = MATLIB_FLOAT_MIN;
    mfl_vfloat_m1_t vec_max = mfl_vfmv_s_f_m1(max, 1);
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        vec_max = mfl_vfredmax(vec_a, vec_max, vl);
    }
    max = mfl_vfmv_f_s_m1(vec_max);
    return max;
}

// matrix min coefficient
inline tinytype mincoeff_rvv(tinytype *ptr_a, int n, int m) {
    tinytype min = MATLIB_FLOAT_MAX;
    mfl_vfloat_m1_t vec_min = mfl_vfmv_s_f_m1(min, 1);
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        vec_min = mfl_vfredmin(vec_a, vec_min, vl);
    }
    min = mfl_vfmv_f_s_m1(vec_min);
    return min;
}

// matrix unary negative
inline void matneg_rvv(tinytype *ptr_a, tinytype *ptr_b, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vfloat_t vec_b = mfl_vfneg(vec_a, vl);
        mfl_vse(ptr_b + l, vec_b, vl);
    }
}

// matrix l2 norm. Accumulates in tinytype precision; for fp16 inputs whose
// squared sum exceeds ~65k the caller should rescale first.
inline tinytype matnorm_rvv(tinytype *ptr_a, int n, int m) {
    int k = m * n, l = 0;
    size_t vlmax = mfl_vsetvlmax();
    mfl_vfloat_m1_t vec_zero = mfl_vfmv_v_f_m1((tinytype)0, vlmax);
    mfl_vfloat_t vec_s = mfl_vfmv_v_f((tinytype)0, vlmax);
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        vec_s = mfl_vfmacc_vv(vec_s, vec_a, vec_a, vl);
    }
    mfl_vfloat_m1_t vec_sum = mfl_vfredusum(vec_s, vec_zero, vlmax);
    tinytype sum = mfl_vfmv_f_s_m1(vec_sum);
    return (tinytype)sqrtf((float)sum);
}

// matrix coefficient-wise abs
inline void cwiseabs_rvv(tinytype *ptr_a, tinytype *ptr_b, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vfloat_t vec_b = mfl_vfabs(vec_a, vl);
        mfl_vse(ptr_b + l, vec_b, vl);
    }
}

// matrix coefficient-wise min
inline void cwisemin_rvv(tinytype *ptr_a, tinytype *ptr_b, tinytype *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vfloat_t vec_b = mfl_vle(ptr_b + l, vl);
        mfl_vfloat_t vec_c = mfl_vfmin_vv(vec_a, vec_b, vl);
        mfl_vse(ptr_c + l, vec_c, vl);
    }
}

// matrix coefficient-wise multiplication
inline void cwisemul_rvv(tinytype *ptr_a, tinytype *ptr_b, tinytype *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vfloat_t vec_b = mfl_vle(ptr_b + l, vl);
        mfl_vfloat_t vec_c = mfl_vfmul_vv(vec_a, vec_b, vl);
        mfl_vse(ptr_c + l, vec_c, vl);
    }
}

// matrix coefficient-wise max
inline void cwisemax_rvv(tinytype *ptr_a, tinytype *ptr_b, tinytype *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vfloat_t vec_b = mfl_vle(ptr_b + l, vl);
        mfl_vfloat_t vec_c = mfl_vfmax_vv(vec_a, vec_b, vl);
        mfl_vse(ptr_c + l, vec_c, vl);
    }
}

// matrix multiplication, note B is not [o][m]
// A[n][o], B[m][o] --> C[n][m];
inline void matmul_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m, int o) {
    size_t vlmax = mfl_vsetvlmax();
    mfl_vfloat_m1_t vec_zero = mfl_vfmv_v_f_m1((tinytype)0, vlmax);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            tinytype *ptr_a = a + i * o; // row major
            tinytype *ptr_b = b + j * o; // column major
            int k = o;
            mfl_vfloat_t vec_s = mfl_vfmv_v_f((tinytype)0, vlmax);
            for (size_t vl; k > 0; k -= vl, ptr_a += vl, ptr_b += vl) {
                vl = mfl_vsetvl(k);
                mfl_vfloat_t vec_a = mfl_vle(ptr_a, vl);
                mfl_vfloat_t vec_b = mfl_vle(ptr_b, vl);
                vec_s = mfl_vfmacc_vv(vec_s, vec_a, vec_b, vl);
            }
            mfl_vfloat_m1_t vec_sum = mfl_vfredusum(vec_s, vec_zero, vlmax);
            mfl_store1(c + i * m + j, vec_sum);
        }
    }
}

/*  a is row major
 *        j
 *    1 2 3 4     9
 *  i 5 6 7 8  *  6
 *    9 8 7 6     5 j
 *                4
 */
inline void matvec_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    size_t vlmax = mfl_vsetvlmax();
    mfl_vfloat_m1_t vec_zero = mfl_vfmv_v_f_m1((tinytype)0, vlmax);
    for (int i = 0; i < n; ++i) {
        int k = m;
        tinytype *ptr_a = a + i * m; // row major
        tinytype *ptr_b = b;
        mfl_vfloat_t vec_s = mfl_vfmv_v_f((tinytype)0, vlmax);
        for (size_t vl; k > 0; k -= vl, ptr_a += vl, ptr_b += vl) {
            vl = mfl_vsetvl(k);
            mfl_vfloat_t vec_a = mfl_vle(ptr_a, vl);
            mfl_vfloat_t vec_b = mfl_vle(ptr_b, vl);
            vec_s = mfl_vfmacc_vv(vec_s, vec_a, vec_b, vl);
        }
        mfl_vfloat_m1_t vec_sum = mfl_vfredusum(vec_s, vec_zero, vlmax);
        mfl_store1(c + i, vec_sum);
    }
}

/*  a is col major
 *      j         i
 *  9 6 5 4  *  1 5 9
 *              2 6 8
 *              3 7 7 j
 *              4 8 6
 */
inline void matvec_transpose_rvv(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    size_t vlmax = mfl_vsetvlmax();
    mfl_vfloat_m1_t vec_zero = mfl_vfmv_v_f_m1((tinytype)0, vlmax);
    for (int i = 0; i < m; ++i) {
        int k = n;
        tinytype *ptr_a = a + i; // column major
        tinytype *ptr_b = b;
        mfl_vfloat_t vec_s = mfl_vfmv_v_f((tinytype)0, vlmax);
        for (size_t vl; k > 0; k -= vl, ptr_a += m * vl, ptr_b += vl) {
            vl = mfl_vsetvl(k);
            mfl_vfloat_t vec_a = mfl_vlse(ptr_a, m * sizeof(tinytype), vl);
            mfl_vfloat_t vec_b = mfl_vle(ptr_b, vl);
            vec_s = mfl_vfmacc_vv(vec_s, vec_a, vec_b, vl);
        }
        mfl_vfloat_m1_t vec_sum = mfl_vfredusum(vec_s, vec_zero, vlmax);
        mfl_store1(c + i, vec_sum);
    }
}

// matrix scalar multiplication
inline void matmulf_rvv(tinytype *ptr_a, tinytype *ptr_b, tinytype f, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vfloat_t vec_b = mfl_vfmul_vf(vec_a, f, vl);
        mfl_vse(ptr_b + l, vec_b, vl);
    }
}

#ifdef USE_TYPE_FP16
/* Multiply a matrix by a scalar supplied as an fp16 BIT PATTERN. The scalar is
 * splatted with vmv.v.x (integer) and applied with vfmul.vv, so NO scalar fp16
 * (flh/fsh/vfmul.vf) is emitted -- required on the vector-fp16-only bench core. */
inline void matmulf_bits_rvv(tinytype *ptr_a, tinytype *ptr_b, unsigned short fbits, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vsc = __riscv_vreinterpret_v_u16m1_f16m1(__riscv_vmv_v_x_u16m1(fbits, vl));
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vse(ptr_b + l, mfl_vfmul_vv(vec_a, vsc, vl), vl);
    }
}
#define matmulf_bits matmulf_bits_rvv
#endif

// matrix subtraction
inline void matsub_rvv(tinytype *ptr_a, tinytype *ptr_b, tinytype *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vfloat_t vec_b = mfl_vle(ptr_b + l, vl);
        mfl_vfloat_t vec_c = mfl_vfsub_vv(vec_a, vec_b, vl);
        mfl_vse(ptr_c + l, vec_c, vl);
    }
}

// matrix addition
inline void matadd_rvv(tinytype *ptr_a, tinytype *ptr_b, tinytype *ptr_c, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vfloat_t vec_b = mfl_vle(ptr_b + l, vl);
        mfl_vfloat_t vec_c = mfl_vfadd_vv(vec_a, vec_b, vl);
        mfl_vse(ptr_c + l, vec_c, vl);
    }
}

// matrix transpose
inline void transpose_rvv(tinytype *a, tinytype *b, int n, int m) {
    for (int j = 0; j < m; ++j) {
        tinytype *ptr_a = a + j;
        tinytype *ptr_b = b + j * n;
        int k = n;
        int l = 0;
        for (size_t vl; k > 0; k -= vl, l += vl, ptr_a = a + l * m + j, ptr_b += vl) {
            vl = mfl_vsetvl(k);
            mfl_vfloat_t vec_a = mfl_vlse(ptr_a, sizeof(tinytype) * m, vl);
            mfl_vse(ptr_b, vec_a, vl);
        }
    }
};

// matrix copy
inline void matcopy_rvv(const tinytype *ptr_a, tinytype *ptr_b, int n, int m) {
    int k = n * m, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vle(ptr_a + l, vl);
        mfl_vse(ptr_b + l, vec_a, vl);
    }
}

inline void matset_rvv(tinytype *ptr_a, tinytype f, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_a = mfl_vfmv_v_f(f, vl);
        mfl_vse(ptr_a + l, vec_a, vl);
    }
}

inline void matsetv_rvv(tinytype *ptr_a, tinytype *f, int n, int m) {
    int k = m * n, l = 0;
    for (size_t vl; k > 0; k -= vl, l += vl) {
        vl = mfl_vsetvl(k);
        mfl_vfloat_t vec_f = mfl_vle(f + l, vl);
        mfl_vse(ptr_a + l, vec_f, vl);
    }
}

}
#endif //TINYMPC_MATLIB_RVV_H
