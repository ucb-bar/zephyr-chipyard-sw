/*
 * Symmetric scalar parallel kernel on the 2-Shuttle SoC.
 *
 * Both harts execute the SAME worker function (`scalar_matmul_slab`) over
 * disjoint slabs of the output of an int8 GEMM:
 *
 *      C[i, j] = sum_{k=0..K} A[i, k] * B[j, k]            (B is K-contiguous on j)
 *
 *   - A is M x K int8
 *   - B is N x K int8
 *   - C is M x N int32
 *
 * Hart 0 computes rows [0, M/2);  hart 1 computes rows [M/2, M).
 * Same source code on both harts; the only per-hart input is the slab range.
 * No vector instructions -- pure RV64GC scalar, so this works on both tiles
 * regardless of whether they have the OPU.
 *
 * After joining, hart 0 verifies the output against a single-hart scalar
 * reference computed in the main thread, and prints per-hart cycle counts
 * + parallelism factor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

/* ======================== Cycle counter ======================== */
static inline uint64_t rdmcycle(void)
{
	uint64_t c;
	__asm__ __volatile__("csrr %0, mcycle\n" : "=r"(c));
	return c;
}

/* ======================== Workload sizes ======================== */
#define M 64
#define N 64
#define K 64
#define HARTS 2

/* Per-hart slab of M rows. */
_Static_assert(M % HARTS == 0, "M must divide evenly across HARTS");
#define ROWS_PER_HART (M / HARTS)

/* ======================== Buffers ======================== */
static int8_t  A[M][K] __attribute__((aligned(16)));
static int8_t  B[N][K] __attribute__((aligned(16)));
static int32_t C[M][N] __attribute__((aligned(16)));     /* parallel result   */
static int32_t Cref[M][N] __attribute__((aligned(16)));  /* serial reference  */

/* ======================== Shared parallel state ======================== */
static volatile int      hart0_alive     __attribute__((aligned(64))) = 0;
static volatile int      hart1_alive     __attribute__((aligned(64))) = 0;
static volatile uint64_t hart0_busy      __attribute__((aligned(64))) = 0;
static volatile uint64_t hart1_busy      __attribute__((aligned(64))) = 0;
static volatile uint64_t serial_baseline __attribute__((aligned(64))) = 0;

/* ======================== The kernel (same code on both harts) ========================
 *
 * Compute the slab of C with row indices in [row_lo, row_hi).
 * Loop order is i-j-k (the unrolled inner loop is K, which we know at compile time).
 */
static __attribute__((noinline))
void scalar_matmul_slab(int row_lo, int row_hi,
                        const int8_t (*restrict a)[K],
                        const int8_t (*restrict b)[K],
                        int32_t (*restrict c)[N])
{
	for (int i = row_lo; i < row_hi; i++) {
		for (int j = 0; j < N; j++) {
			int32_t acc = 0;
			const int8_t *arow = a[i];
			const int8_t *brow = b[j];
			for (int k = 0; k < K; k++) {
				acc += (int32_t)arow[k] * (int32_t)brow[k];
			}
			c[i][j] = acc;
		}
	}
}

/* ======================== Threading boilerplate ======================== */
#define STACK_SIZE 4096
K_THREAD_STACK_DEFINE(hart0_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(hart1_stack, STACK_SIZE);
static struct k_thread hart0_data;
static struct k_thread hart1_data;

/* ======================== Per-hart worker entry ======================== */
static void hart_worker(void *expected_hart_p, void *row_lo_p, void *row_hi_p)
{
	int expected = (int)(uintptr_t)expected_hart_p;
	int row_lo   = (int)(uintptr_t)row_lo_p;
	int row_hi   = (int)(uintptr_t)row_hi_p;

	int hart = arch_curr_cpu()->id;
	if (hart == 0) hart0_alive = (expected == 0);
	if (hart == 1) hart1_alive = (expected == 1);

	if (hart != expected) {
		/* refuse to run if we landed on the wrong hart */
		return;
	}

	uint64_t t0 = rdmcycle();
	scalar_matmul_slab(row_lo, row_hi, A, B, C);
	uint64_t t1 = rdmcycle();

	if (hart == 0) hart0_busy = t1 - t0;
	if (hart == 1) hart1_busy = t1 - t0;
}

/* ======================== Pretty print ======================== */
static void put_u64(uint64_t v)
{
	char buf[24];
	int  n = 0;
	if (v == 0) { buf[n++] = '0'; }
	while (v) { buf[n++] = '0' + (int)(v % 10); v /= 10; }
	for (int i = n - 1; i >= 0; i--) putchar(buf[i]);
}

int main(void)
{
	printf("=== symmetric scalar parallel matmul on %s ===\n", CONFIG_BOARD_TARGET);
	printf("zephyr cpus = %d\n", CONFIG_MP_MAX_NUM_CPUS);
	printf("Workload: int8 M=%d, N=%d, K=%d  ->  int32 C = A * B^T\n", M, N, K);
	printf("Both harts run the SAME function (scalar_matmul_slab), each owning %d rows.\n",
	       ROWS_PER_HART);

	/* Initialize A, B with a deterministic pattern so the result is verifiable. */
	for (int i = 0; i < M; i++) {
		for (int k = 0; k < K; k++) A[i][k] = (int8_t)((i + k) & 0x3F);
	}
	for (int j = 0; j < N; j++) {
		for (int k = 0; k < K; k++) B[j][k] = (int8_t)((j ^ k) & 0x3F);
	}
	memset(C,    0xAA, sizeof(C));
	memset(Cref, 0,    sizeof(Cref));

	/* ----- 1. Serial baseline on hart 0 only (the main thread runs on hart 0). ----- */
	uint64_t s0 = rdmcycle();
	scalar_matmul_slab(0, M, A, B, Cref);
	uint64_t s1 = rdmcycle();
	serial_baseline = s1 - s0;
	printf("[serial] one hart computed full %dx%d matmul in ", M, N);
	put_u64(serial_baseline); printf(" cycles\n");

	/* ----- 2. Parallel run: spawn one worker per hart, each does half the rows. ----- */
	uint64_t pw_start = rdmcycle();

	k_tid_t t0 = k_thread_create(&hart0_data, hart0_stack, STACK_SIZE,
				     hart_worker,
				     (void*)(uintptr_t)0, (void*)(uintptr_t)0,
				     (void*)(uintptr_t)(M / 2),
				     K_PRIO_PREEMPT(1), 0, K_FOREVER);
	k_thread_cpu_pin(t0, 0);

	k_tid_t t1 = k_thread_create(&hart1_data, hart1_stack, STACK_SIZE,
				     hart_worker,
				     (void*)(uintptr_t)1, (void*)(uintptr_t)(M / 2),
				     (void*)(uintptr_t)M,
				     K_PRIO_PREEMPT(1), 0, K_FOREVER);
	k_thread_cpu_pin(t1, 1);

	k_thread_start(t0);
	k_thread_start(t1);
	k_thread_join(t0, K_FOREVER);
	k_thread_join(t1, K_FOREVER);

	uint64_t pw_end   = rdmcycle();
	uint64_t parallel_wall = pw_end - pw_start;

	/* ----- 3. Verify the parallel result against the serial reference. ----- */
	int errors = 0;
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			if (C[i][j] != Cref[i][j]) errors++;
		}
	}

	/* ----- 4. Report. ----- */
	printf("=== summary ===\n");
	printf("hart0 alive=%d, busy cycles: ", hart0_alive); put_u64(hart0_busy); printf("\n");
	printf("hart1 alive=%d, busy cycles: ", hart1_alive); put_u64(hart1_busy); printf("\n");
	printf("parallel wallclock (incl. thread spawn/join): "); put_u64(parallel_wall); printf("\n");
	printf("serial baseline (one hart, full matmul):     "); put_u64(serial_baseline); printf("\n");

	uint64_t sum_busy = hart0_busy + hart1_busy;
	uint64_t max_busy = (hart0_busy > hart1_busy) ? hart0_busy : hart1_busy;
	if (max_busy == 0) max_busy = 1;
	uint64_t parallel_x100 = (sum_busy * 100ULL) / max_busy;
	printf("hart parallelism x100 (sum_busy / max_busy): "); put_u64(parallel_x100); printf("\n");
	printf("  200 = both harts busy 100%% of the time\n");
	printf("  100 = one hart idle\n");

	uint64_t speedup_x100 = (serial_baseline * 100ULL) / max_busy;
	printf("speedup x100 (serial_baseline / max_busy):   "); put_u64(speedup_x100); printf("\n");
	printf("  200 = ideal 2x speedup from SMP\n");

	printf("output verify: %d / %d cells correct, errors=%d\n",
	       M * N - errors, M * N, errors);

	int ok = (errors == 0) && hart0_alive && hart1_alive &&
	         parallel_x100 >= 150 && speedup_x100 >= 150;
	if (ok) {
		printf(">>> SMP SCALAR-PARALLEL TEST PASSED <<<\n");
	} else if (errors == 0 && hart0_alive && hart1_alive) {
		printf(">>> CORRECTNESS PASS, parallelism below 1.5x threshold <<<\n");
	} else {
		printf(">>> TEST FAILED <<<\n");
	}
	return 0;
}
