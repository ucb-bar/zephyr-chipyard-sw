/*
 * SPDX-License-Identifier: Apache-2.0
 * Minimal Zvfh test: _Float16 + RVV intrinsics
 */

#include <stdio.h>
#include <stdint.h>
#include <zephyr/sys/reboot.h>
#include <riscv_vector.h>

typedef _Float16 f16;

/* --- tiny runtime seed so the optimizer can't precompute everything --- */
static inline uint32_t runtime_seed(void) {
    volatile const char *vp = CONFIG_BOARD_TARGET;
    uint32_t h = 2166136261u;
    for (size_t i = 0; vp[i] != '\0'; ++i) { h ^= (uint8_t)vp[i]; h *= 16777619u; }
    h ^= (uint32_t)(uintptr_t)&vp;  /* mix stack addr to foil CSE */
    return h ? h : 1u;
}

/* read misa (to check 'V') */
static inline unsigned long read_csr_misa(void) {
    unsigned long x; __asm__ volatile("csrr %0, misa" : "=r"(x));
    return x;
}
#define MISA_EXT_BIT(ch) (1UL << ((ch) - 'A'))

/* simple driver: y = a*b + c, then y += alpha */
static void zvfh_fma_and_axpy(f16 *y, const f16 *a, const f16 *b, const f16 *c,
                              f16 alpha, size_t n)
{
    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e16m1(n - i);

        /* loads */
        vfloat16m1_t va = __riscv_vle16_v_f16m1(&a[i], vl);
        vfloat16m1_t vb = __riscv_vle16_v_f16m1(&b[i], vl);
        vfloat16m1_t vc = __riscv_vle16_v_f16m1(&c[i], vl);

        /* vy = vc + va * vb  (fused) */
        vfloat16m1_t vy = __riscv_vfmacc_vv_f16m1(vc, va, vb, vl);

        /* vy = vy + alpha (vector-scalar add) */
        vy = __riscv_vfadd_vf_f16m1(vy, alpha, vl);

        /* store */
        __riscv_vse16_v_f16m1(&y[i], vy, vl);

        i += vl;
    }
}

int main(void)
{
    printf("Zephyr Zvfh intrinsics smoke test on: %s\n", CONFIG_BOARD_TARGET);

    /* Runtime check: require V (Zvfh rides on V). */
    unsigned long misa = read_csr_misa();
    if ((misa & MISA_EXT_BIT('V')) == 0) {
        printf("This hart does not advertise V in misa; skipping RVV test.\n");
        sys_reboot(SYS_REBOOT_COLD);
        return 0;
    }

    enum { N = 128 };
    static f16 a[N], b[N], c[N], y[N];

    /* Fill inputs with non-constant-foldable data */
    uint32_t seed = runtime_seed();
    for (int i = 0; i < N; ++i) {
        float s = 0.001f * (float)((seed ^ (0x9E3779B9u + i*2654435761u)) & 0x3FF);
        a[i] = (f16)(0.25f + s + 0.01f * (float)(i % 7));
        b[i] = (f16)(0.50f - s + 0.02f * (float)(i % 5));
        c[i] = (f16)(-0.10f + 0.03f * (float)(i % 3));
        y[i] = (f16)0.0f;
    }

    f16 alpha = (f16)0.125f;

    /* Do a couple of passes to exercise the intrinsics a bit */
    zvfh_fma_and_axpy(y, a, b, c, alpha, N);
    zvfh_fma_and_axpy(y, y, a, c, alpha, N);  /* re-use y as input to mix it up */

    /* Print a few sample results */
    for (int i = 0; i < 8; ++i) {
        printf("y[%d] = % .6f\n", i, (double)y[i]);
    }

    /* Done */
    sys_reboot(SYS_REBOOT_COLD);
    return 0;
}
