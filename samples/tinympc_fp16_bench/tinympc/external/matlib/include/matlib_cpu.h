#pragma once
#ifndef TINYMPC_MATLIB_CPU_H
#define TINYMPC_MATLIB_CPU_H

#include <float.h>

#include "matlib_types.h"

extern "C"
{

// Golden (reference) function declarations
tinytype maxcoeff_cpu(tinytype *a, int n, int m);
tinytype mincoeff_cpu(tinytype *a, int n, int m);
tinytype matnorm_cpu(tinytype *a, int n, int m);
void matneg_cpu(tinytype *a, tinytype *b, int n, int m);
void cwiseabs_cpu(tinytype *a, tinytype *b, int n, int m);
void cwisemin_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int mr);
void cwisemax_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void cwisemul_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void matmul_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m, int o);
void matvec_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void matvec_transpose_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void matmulf_cpu(tinytype *a, tinytype *b, tinytype f, int n, int m);
void matsub_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void matadd_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m);
void transpose_cpu(tinytype *a, tinytype *b, int n, int m);
void matcopy_cpu(const tinytype *a, tinytype *b, int n, int m);
void matset_cpu(tinytype *a, tinytype f, int n, int m);
void matsetv_cpu(tinytype *a, tinytype *f, int n, int m);

#ifdef USE_CPU
#define maxcoeff maxcoeff_cpu
#define mincoeff mincoeff_cpu
#define matnorm matnorm_cpu
#define matneg matneg_cpu
#define cwiseabs cwiseabs_cpu
#define cwisemin cwisemin_cpu
#define cwisemax cwisemax_cpu
#define cwisemul cwisemul_cpu
#define matmul matmul_cpu
#define matvec matvec_cpu
#define matvec_transpose matvec_transpose_cpu
#define matmulf matmulf_cpu
#define matsub matsub_cpu
#define matadd matadd_cpu
#define transpose transpose_cpu
#define matcopy matcopy_cpu
#define matset matset_cpu
#define matsetv matsetv_cpu
#endif

}

// matrix maximum coefficient
inline tinytype maxcoeff_cpu(tinytype *a, int n, int m) {
    tinytype max = MATLIB_FLOAT_MIN;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            max = a[i * m + j] > max ? a[i * m + j] : max;
        }
    }
    return max;
}

// matrix min coefficient
inline tinytype mincoeff_cpu(tinytype *a, int n, int m) {
    tinytype min = MATLIB_FLOAT_MAX;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            min = a[i * m + j] < min ? a[i * m + j] : min;
        }
    }
    return min;
}

// matrix unary negative
inline void matneg_cpu(tinytype *a, tinytype *b, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            b[i * m + j] = -a[i * m + j];
        }
    }
}

// matrix coefficient-wise abs
inline void cwiseabs_cpu(tinytype *a, tinytype *b, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            tinytype v = a[i * m + j];
            b[i * m + j] = v < (tinytype)0 ? -v : v;
        }
    }
}

// matrix coefficient-wise min
inline void cwisemin_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            c[i * m + j] = a[i * m + j] < b[i * m + j] ? a[i * m + j] : b[i * m + j];
        }
    }
}

// matrix coefficient-wise multiplication
inline void cwisemul_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            c[i * m + j] = a[i * m + j] * b[i * m + j];
        }
    }
}

// matrix coefficient-wise max
inline void cwisemax_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            c[i * m + j] = a[i * m + j] > b[i * m + j] ? a[i * m + j] : b[i * m + j];
        }
    }
}

// matrix multiplication, note B is not [o][m]
// A[n][o], B[m][o] --> C[n][m];
inline void matmul_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m, int o) {
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j) {
            tinytype_acc acc = 0;
            for (int k = 0; k < o; ++k)
                acc += (tinytype_acc)a[i * o + k] * (tinytype_acc)b[j * o + k];
            c[i * m + j] = (tinytype)acc;
        }
}

inline void matvec_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    for (int i = 0; i < n; ++i) {
        tinytype_acc acc = 0;
        for (int j = 0; j < m; ++j) {
            acc += (tinytype_acc)a[i * m + j] * (tinytype_acc)b[j];
        }
        c[i] = (tinytype)acc;
    }
}

inline void matvec_transpose_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    for (int i = 0; i < m; ++i) {
        tinytype_acc acc = 0;
        for (int j = 0; j < n; ++j) {
            acc += (tinytype_acc)a[j * m + i] * (tinytype_acc)b[j];
        }
        c[i] = (tinytype)acc;
    }
}

// matrix scalar multiplication
inline void matmulf_cpu(tinytype *a, tinytype *b, tinytype f, int n, int m) {
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j) {
            b[i * m + j] = f * a[i * m + j];
        }
}

// matrix subtraction
inline void matsub_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            c[i * m + j] = a[i * m + j] - b[i * m + j];
        }
    }
}

// matrix addition
inline void matadd_cpu(tinytype *a, tinytype *b, tinytype *c, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            c[i * m + j] = a[i * m + j] + b[i * m + j];
        }
    }
}

// matrix transpose
inline void transpose_cpu(tinytype *a, tinytype *b, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            b[j * n + i] = a[i * m + j];
        }
    }
}

// matrix copy
inline void matcopy_cpu(const tinytype *a, tinytype *b, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            b[i * m + j] = a[i * m + j];
        }
    }
}

inline void matset_cpu(tinytype *a, tinytype f, int n, int m) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            a[i * m + j] = f;
        }
    }
}

inline void matsetv_cpu(tinytype *a, tinytype *f, int n, int m) {
    for (int i = 0, k = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j, ++k) {
            a[i * m + j] = f[k];
        }
    }
}

// matrix l2 norm. Accumulates in tinytype_acc (fp32 for fp16 storage by default).
inline tinytype matnorm_cpu(tinytype *a, int n, int m) {
    tinytype_acc sum = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            tinytype_acc v = (tinytype_acc)a[i * m + j];
            sum += v * v;
        }
    }
    return (tinytype)sqrtf((float)sum);
}

#endif // TINYMPC_MATLIB_CPU_H
