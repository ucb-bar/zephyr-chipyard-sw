#pragma once
#ifndef TINYMPC_MATLIB_H
#define TINYMPC_MATLIB_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MSTATUS_VS          0x00000600
#define MSTATUS_FS          0x00006000
#define MSTATUS_XS          0x00018000

#include "matlib_types.h"
#include "matlib_lmul.h"
#ifdef USE_CPU
#include "matlib_cpu.h"
#endif
#ifdef USE_RVA
#include "matlib_rva.h"
#endif
#ifdef USE_RVV
#include "matlib_rvv.h"
#endif
#ifdef USE_RVVU
#include "matlib_rvvu.h"
#endif

extern "C" {

inline void gen_rand_1d(tinytype *a, int n);
inline void gen_string(char *s, int n);
inline void print_string(const char *a, const char *name);
inline void print_array_1d(tinytype *a, int n, const char *type, const char *name);
inline bool float_eq(tinytype golden, tinytype actual, tinytype relErr);
inline bool compare_1d(tinytype *golden, tinytype *actual, int n);
inline bool compare_string(const char *golden, const char *actual, int n);
inline tinytype *alloc_array_1d(int n);
inline void free_array_1d(tinytype *ar);
inline void init_array_zero_1d(tinytype *ar, int n);
inline void init_array_one_1d(tinytype *ar, int n);

#ifdef USE_RVA
inline void gen_rand_2d(tinytype **ar, int n, int m);
inline void print_array_2d(tinytype **a, int n, int m, const char *type, const char *name);
inline bool compare_2d(tinytype **golden, tinytype **actual, int n, int m);
inline tinytype **alloc_array_2d(int n, int m);
inline tinytype **alloc_array_2d_col(int n, int m);
inline tinytype checksum(tinytype **ar, int n, int m);
inline void free_array_2d(tinytype **ar);
inline void init_array_one_2d(tinytype **ar, int n, int m);
inline void printx(tinytype **a, int n, int m, const char *name);
#elif defined(USE_RVV) || defined(USE_RVVU)
inline void gen_rand_2d(tinytype *ar, int n, int m);
inline void print_array_2d(tinytype *a, int n, int m, const char *type, const char *name);
inline bool compare_2d(tinytype *golden, tinytype *actual, int n, int m);
inline tinytype *alloc_array_2d(int n, int m);
inline tinytype *alloc_array_2d_col(int n, int m);
inline tinytype checksum(tinytype *ar, int n, int m);
inline void free_array_2d(tinytype *ar);
inline void init_array_one_2d(tinytype *ar, int n, int m);
inline void printx(tinytype *a, int n, int m, const char *name);
#endif

#ifdef TRACE_CHECKSUMS
#define TRACE_CHECKSUM(func, matrix) \
    printf( "%s checksum %f\n", #func, (double)(matrix).checksum() ); \
    print_array_2d((matrix).data, (matrix).outer, (matrix).inner, "float",  #func);
#else
#define TRACE_CHECKSUM(func, matrix)
#endif

#ifdef USE_PK
static inline void enable_vector_operations() {
    printf("Using PK\n");
}
#else
static inline void enable_vector_operations() {
    unsigned long mstatus;

    // Read current mstatus
    asm volatile("csrr %0, mstatus" : "=r"(mstatus));

    // Set VS field to Dirty (11)
    mstatus |= MSTATUS_VS | MSTATUS_FS | MSTATUS_XS;

    // Write back updated mstatus
    asm volatile("csrw mstatus, %0"::"r"(mstatus));
}
#endif // USE_PK

static inline uint64_t read_cycles() {
    uint64_t cycles;
    asm volatile ("csrr %0, cycle" : "=r" (cycles));
    return cycles;
}

inline void gen_rand_1d(tinytype *a, int n) {
    for (int i = 0; i < n; ++i)
        a[i] = (tinytype)((float)rand() / (float)RAND_MAX + (float)(rand() % 1000));
}

inline void gen_string(char *s, int n) {
    // char value range: -128 ~ 127
    for (int i = 0; i < n - 1; ++i)
        s[i] = (char)(rand() % 127) + 1;
    s[n - 1] = '\0';
}

inline void print_string(const char *a, const char *name) {
    printf("const char *%s = \"", name);
    int i = 0;
    while (a[i] != 0)
        putchar(a[i++]);
    printf("\"\n");
    puts("");
}

inline void print_array_1d(tinytype *a, int n, const char *type, const char *name) {
    printf("%s %s[%d] = {\n", type, name, n);
    for (int i = 0; i < n; ++i) {
        printf("% 8.4f%s", (double)a[i], i != n - 1 ? "," : "};\n");
        if (i % 10 == 9)
            puts("");
    }
    puts("");
}

inline bool float_eq(tinytype golden, tinytype actual, tinytype relErr) {
    float g = (float)golden;
    float a = (float)actual;
    float e = (float)relErr;
    float diff = a - g;
    if (diff < 0) diff = -diff;
    float scale = (a < 0) ? -a : a;
    return (diff < e) || ((scale > 0) && ((diff / scale) < e));
}

inline bool compare_1d(tinytype *golden, tinytype *actual, int n) {
    for (int i = 0; i < n; ++i)
        if (!float_eq(golden[i], actual[i], (tinytype)MATLIB_COMPARE_TOL))
            return false;
    return true;
}

inline bool compare_string(const char *golden, const char *actual, int n) {
    for (int i = 0; i < n; ++i)
        if (golden[i] != actual[i])
            return false;
    return true;
}

inline tinytype *alloc_array_1d(int n) {
    return (tinytype *)malloc(sizeof(tinytype) * n);
}

inline void free_array_1d(tinytype *ar) {
    free(ar);
}

inline void init_array_zero_1d(tinytype *ar, int n) {
    for (int i = 0; i < n; ++i)
        ar[i] = 0;
}

inline void init_array_one_1d(tinytype *ar, int n) {
    for (int i = 0; i < n; ++i)
        ar[i] = 1;
}

#if defined(USE_RVV) || defined(USE_RVVU) || defined(USE_CPU)

inline void gen_rand_2d(tinytype *ar, int n, int m) {
    for (int i = 0; i < m * n; ++i)
        ar[i] = (tinytype)((float)rand() / (float)RAND_MAX + (float)(rand() % 1000));
}

inline void print_array_2d(tinytype *a, int n, int m, const char *type, const char *name) {
    printf("%s %s[%d][%d] = {\n", type, name, n, m);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            printf("%8.4f", (double)a[i * m + j]);
            if (j == m - 1)
                printf(i == n - 1 ? "};\n" : ",\n");
            else
                printf(",");
        }
    }
    puts("");
}

inline bool compare_2d(tinytype *golden, tinytype *actual, int n, int m) {
    for (int i = 0; i < m * n; ++i)
        if (!float_eq(golden[i], actual[i], (tinytype)MATLIB_COMPARE_TOL))
            return false;
    return true;
}

// Row major allocation
inline tinytype *alloc_array_2d(int n, int m) {
    tinytype *data = (tinytype *)malloc(sizeof(tinytype) * n * m);
    for (int i = 0; i < m * n; i++) {
        data[i] = 0;
    }
    return data;
}

// Column major allocation
inline tinytype *alloc_array_2d_col(int n, int m) {
    tinytype *data = (tinytype *)malloc(sizeof(tinytype) * n * m);
    for (int i = 0; i < m * n; i++) {
        data[i] = 0;
    }
    return data;
}

inline tinytype checksum(tinytype *ar, int n, int m) {
    tinytype sum = 0;
    for (int i = 0; i < m * n; ++i)
        sum += ar[i];
    return sum;
}

inline void free_array_2d(tinytype *ar) {
    free(ar);
}

inline void init_array_one_2d(tinytype *ar, int n, int m) {
    for (int i = 0; i < m * n; ++i)
        ar[i] = 1;
}

inline void printx(tinytype *a, int n, int m, const char *name) {
    printf("%s ", name);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            printf("%8.4f", (double)a[i * m + j]);
            if (j == m - 1)
                puts(i == n - 1 ? "" : ",");
            else
                putchar(',');
        }
    }
}

#endif
#ifdef USE_RVA

inline void gen_rand_2d(tinytype **ar, int n, int m) {
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            ar[i][j] = (tinytype)((float)rand() / (float)RAND_MAX + (float)(rand() % 1000));
}

inline void print_array_2d(tinytype **a, int n, int m, const char *type, const char *name) {
    printf("%s %s[%d][%d] = {\n", type, name, n, m);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            printf("%8.4f", (double)a[i][j]);
            if (j == m - 1)
                printf(i == n - 1 ? "};\n" : ",\n");
            else
                printf(",");
        }
    }
    puts("");
}

inline bool compare_2d(tinytype **golden, tinytype **actual, int n, int m) {
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            if (!float_eq(golden[i][j], actual[i][j], (tinytype)MATLIB_COMPARE_TOL))
                return false;
    return true;
}

// Row major allocation
inline tinytype **alloc_array_2d(int n, int m) {
    tinytype **ret = (tinytype **)malloc(sizeof(tinytype *) * n);
    tinytype *data = (tinytype *)malloc(sizeof(tinytype) * n * m);
    for (int i = 0; i < n; ++i)
        ret[i] = (tinytype *)(&data[i * m]);
    for (int i = 0; i < m * n; i++) {
        data[i] = 0;
    }
    return ret;
}

// Column major allocation
inline tinytype **alloc_array_2d_col(int n, int m) {
    tinytype **ret = (tinytype **)malloc(sizeof(tinytype *) * m);
    tinytype *data = (tinytype *)malloc(sizeof(tinytype) * n * m);
    for (int i = 0; i < m; ++i)
        ret[i] = (tinytype *)(&data[i * n]);
    for (int i = 0; i < m * n; i++) {
        data[i] = 0;
    }
    return ret;
}

inline tinytype checksum(tinytype *ar, int n, int m) {
    tinytype sum = 0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            sum += ar[i][j];
    return sum;
}

inline void free_array_2d(tinytype **ar) {
    free(ar[0]);
    free((tinytype *)ar);
}

inline void init_array_one_2d(tinytype **ar, int n, int m) {
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            ar[i][j] = 1;
}

inline void printx(tinytype **a, int n, int m, const char *name) {
    printf("%s ", name);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            printf("%8.4f", (double)a[i][j]);
            if (j == m - 1)
                puts(i == n - 1 ? "" : ",");
            else
                putchar(',');
        }
    }
}
#endif

};

#endif //TINYMPC_MATLIB_H
