/*
 * Copyright (c) 2025 Dima Nikiforov
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <zephyr/sys/reboot.h>

/* ---- helpers to view raw 16-bit encodings ---- */
static inline uint16_t f16_bits(_Float16 x) {
    union { _Float16 f; uint16_t u; } v;
    v.f = x;
    return v.u;
}

#if defined(__BFLT16_MANT_DIG__) || defined(__FLT16_BF16__)
static inline uint16_t bf16_bits(__bf16 x) {
    union { __bf16 f; uint16_t u; } v;
    v.f = x;
    return v.u;
}
#endif

static void print_f16(const char *label, _Float16 x) {
    printf("%-24s = % .8f  (bits 0x%04" PRIx16 ")\n", label, (double)x, f16_bits(x));
}

#if defined(__BFLT16_MANT_DIG__) || defined(__FLT16_BF16__)
static void print_bf16(const char *label, __bf16 x) {
    float as_f = (float)x;
    printf("%-24s = % .8f  (bits 0x%04" PRIx16 ")\n", label, (double)as_f, bf16_bits(x));
}
#endif

/* ---- create a small runtime-only seed (not constant-foldable) ---- */
static __attribute__((noinline)) uint32_t runtime_seed(void) {
    /* Walk the board string via a volatile pointer so loads can't be elided/folded */
    volatile const char *vp = CONFIG_BOARD_TARGET;
    uint32_t h = 2166136261u;          /* FNV-ish */
    for (size_t i = 0; vp[i] != '\0'; ++i) {
        h ^= (uint8_t)vp[i];
        h *= 16777619u;
    }
    /* Mix in a stack address to avoid the compiler treating it as a pure function */
    uintptr_t spish = (uintptr_t)&vp;
    h ^= (uint32_t)spish;
    h ^= (uint32_t)(spish >> 11);
    return h ? h : 1u;
}

/* ---- FP16 dynamic arithmetic that must execute at runtime ---- */
static __attribute__((noinline)) _Float16 f16_dyn_mix(_Float16 a, _Float16 b) {
    volatile _Float16 buf[8];
    /* Fill with data derived from inputs so it can't be precomputed */
    for (int i = 0; i < 8; ++i) {
        /* (i+1) in denom prevents the compiler from recognizing trivial patterns */
        buf[i] = (a + (_Float16)(float)i) * (_Float16)0.5f - b / (_Float16)(float)(i + 1);
    }
    _Float16 s = (_Float16)0.0f;
    for (int i = 0; i < 8; ++i) {
        /* A couple of ops to exercise add/mul/div/FMA lowering paths */
        _Float16 t = (buf[i] * a) + (b / (_Float16)(float)(i + 2));
        buf[i] = t - (_Float16)0.0625f;
        s += buf[i];
    }
    return s;
}

#if defined(__BFLT16_MANT_DIG__) || defined(__FLT16_BF16__)
static __attribute__((noinline)) __bf16 bf16_dyn_mix(__bf16 a, __bf16 b) {
    volatile __bf16 buf[8];
    for (int i = 0; i < 8; ++i) {
        buf[i] = (__bf16)((float)a + (float)i) * (__bf16)0.5f - b / (__bf16)(float)(i + 1);
    }
    __bf16 s = (__bf16)0.0f;
    for (int i = 0; i < 8; ++i) {
        __bf16 t = (buf[i] * a) + (b / (__bf16)(float)(i + 2));
        buf[i] = t - (__bf16)0.0625f;
        s += buf[i];
    }
    return s;
}
#endif

int main(void)
{
    printf("Zephyr FP16/BF16 test on: %s\n", CONFIG_BOARD_TARGET);

#if defined(__FLT16_MANT_DIG__)
    _Static_assert(sizeof(_Float16) == 2, "_Float16 should be 2 bytes");

    /* --- simple, possibly foldable sanity prints --- */
    _Float16 a = (_Float16)1.5f, b = (_Float16)0.25f, c = (_Float16)(-2.0f);
    print_f16("a", a);
    print_f16("b", b);
    print_f16("c", c);
    print_f16("a + b", a + b);
    print_f16("a * b", a * b);
    print_f16("c / a", c / a);

    /* --- dynamic path that cannot be constant-folded --- */
    uint32_t seed = runtime_seed();
    /* Map seed to benign FP16 inputs in (0, 2) */
    _Float16 dx = (_Float16)((seed % 97u) / 97.0f + 0.125f);
    _Float16 dy = (_Float16)(((seed >> 7) % 53u) / 53.0f + 0.1875f);

    print_f16("dx (dyn)", dx);
    print_f16("dy (dyn)", dy);

    _Float16 s16 = f16_dyn_mix(dx, dy);
    print_f16("f16_dyn_mix sum", s16);

    /* epsilon probe with dynamic offset to force runtime evaluation */
    _Float16 one = (_Float16)1.0f;
    _Float16 eps = (_Float16)0x1p-10f; /* exact FP16 ulp at 1.0 */
    _Float16 jitter = (_Float16)((seed & 7u) * 0.0625f); /* {0, 1/16, 2/16, ...} */
    print_f16("1 + eps + jitter", one + eps + jitter);
#else
    printf("This toolchain does not support _Float16 (no __FLT16_MANT_DIG__).\n");
#endif

#if defined(__BFLT16_MANT_DIG__) || defined(__FLT16_BF16__)
    _Static_assert(sizeof(__bf16) == 2, "__bf16 should be 2 bytes");

    /* Map the same seed to BF16 inputs */
    uint32_t seed2 = runtime_seed() ^ 0x9E3779B9u;
    __bf16 bx = (__bf16)((seed2 % 97u) / 97.0f + 0.25f);
    __bf16 by = (__bf16)(((seed2 >> 5) % 61u) / 61.0f + 0.0625f);

    print_bf16("__bf16 bx (dyn)", bx);
    print_bf16("__bf16 by (dyn)", by);

    __bf16 s_bf16 = bf16_dyn_mix(bx, by);
    print_bf16("bf16_dyn_mix sum", s_bf16);

    /* Compare encodings for ~same numeric range */
    print_f16("as _Float16 ~bx", (_Float16)(float)bx);
    print_bf16("as __bf16  ~bx", bx);
#else
    printf("__bf16 not supported by this GCC/target.\n");
#endif

    /* End the sim */
    sys_reboot(SYS_REBOOT_COLD);
    return 0;
}
