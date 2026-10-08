/*
 * Multithreaded multicore RVV compute demo (Merlin).
 *
 * NUM_THREADS worker threads, pinned round-robin across the harts, each repeatedly
 * accumulate  y[i] += a * x[i]  over their own slice using an RVV (vfmacc) kernel
 * written with `.option arch, +v` inline asm (so it builds with the SDK 0.17.0
 * GCC 12.2.0, which lacks riscv_vector.h and RVV asm-clobber support). Between
 * rounds each thread k_yield()s, so several V-using threads time-share every hart
 * and the scheduler interleaves them.
 *
 * Correctness is the point: arch/riscv/core/v.c must save/restore each thread's
 * v0..v31 + V CSRs on every context switch (eager path, CONFIG_RISCV_ISA_EXT_V_LAZY=n).
 * If it did not, a thread preempted mid-kernel would resume with another thread's
 * vector registers and its slice would be wrong. Each thread validates its slice
 * against a scalar reference and prints PASS/FAIL; main tallies them.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/arch/cpu.h>
#include <stdio.h>
#include <math.h>

#define NUM_HARTS   CONFIG_MP_MAX_NUM_CPUS
#define NUM_THREADS (NUM_HARTS * 3)     /* several V-using threads per hart */
#define SLICE       2048                /* floats per thread (strip-mined) */
#define ITERS       8                   /* accumulation rounds, yielding between */
#define STACK_SIZE  8192

static float g_x[SLICE];                /* shared read-only input */
static float g_y[NUM_THREADS][SLICE];   /* per-thread accumulator */

K_THREAD_STACK_ARRAY_DEFINE(stacks, NUM_THREADS, STACK_SIZE);
static struct k_thread threads[NUM_THREADS];
static k_tid_t tids[NUM_THREADS];
static struct k_mutex print_mutex;
static atomic_t pass_count;

/* y[i] += a * x[i], i in [0,n).  RVV vfmacc, e32/m8, strip-mined.
 * No vector-register clobbers (GCC 12.2.0 can't name them); .option arch,+v
 * assembles the V ops and the "memory" clobber + volatile order the accesses. */
static void rvv_saxpy(float a, const float *x, float *y, int n)
{
	const float *xp = x;
	float *yp = y;
	int nn = n;

	__asm volatile(
		".option push\n\t"
		".option arch, +v\n\t"
		"1:\n\t"
		"vsetvli t0, %[n], e32, m8, ta, ma\n\t"
		"vle32.v v8, (%[x])\n\t"
		"vle32.v v16, (%[y])\n\t"
		"vfmacc.vf v16, %[a], v8\n\t"     /* v16 = a*v8 + v16 */
		"vse32.v v16, (%[y])\n\t"
		"slli t1, t0, 2\n\t"              /* bytes = vl * 4 */
		"add %[x], %[x], t1\n\t"
		"add %[y], %[y], t1\n\t"
		"sub %[n], %[n], t0\n\t"
		"bnez %[n], 1b\n\t"
		".option pop\n\t"
		: [x] "+r"(xp), [y] "+r"(yp), [n] "+r"(nn)
		: [a] "f"(a)
		: "t0", "t1", "memory");
}

static void worker(void *arg1, void *arg2, void *arg3)
{
	int id = (int)(uintptr_t)arg1;
	float a = (float)(id + 1);
	float *y = g_y[id];

	for (int i = 0; i < SLICE; i++) {
		y[i] = (float)id;             /* y_init = id */
	}

	for (int r = 0; r < ITERS; r++) {
		rvv_saxpy(a, g_x, y, SLICE);  /* y += a*x */
		k_yield();                    /* let the other workers on this hart run */
	}

	/* reference: y_init + ITERS * a * x[i] */
	int errors = 0;
	for (int i = 0; i < SLICE; i++) {
		float ref = (float)id + (float)ITERS * a * g_x[i];
		if (fabsf(y[i] - ref) > 1e-2f * (1.0f + fabsf(ref))) {
			if (errors < 3) {
				k_mutex_lock(&print_mutex, K_FOREVER);
				printf("thread %d FAIL at %d: got %f want %f\n",
				       id, i, (double)y[i], (double)ref);
				k_mutex_unlock(&print_mutex);
			}
			errors++;
		}
	}

	k_mutex_lock(&print_mutex, K_FOREVER);
	if (errors == 0) {
		printf("thread %2d on hart %d: PASS (%d elems, a=%d)\n",
		       id, arch_curr_cpu()->id, SLICE, (int)a);
		atomic_inc(&pass_count);
	} else {
		printf("thread %2d on hart %d: FAIL (%d errors)\n", id, arch_curr_cpu()->id,
		       errors);
	}
	k_mutex_unlock(&print_mutex);
}

int main(void)
{
	k_mutex_init(&print_mutex);
	for (int i = 0; i < SLICE; i++) {
		g_x[i] = 0.5f * (float)(i % 17);
	}

	printf("Merlin multicore RVV SAXPY: %d threads over %d harts (%s)\n",
	       NUM_THREADS, NUM_HARTS, CONFIG_BOARD_TARGET);

	for (int i = 0; i < NUM_THREADS; i++) {
		tids[i] = k_thread_create(&threads[i], stacks[i], STACK_SIZE, worker,
					  (void *)(uintptr_t)i, NULL, NULL,
					  K_PRIO_PREEMPT(1), 0, K_FOREVER);
		k_thread_cpu_pin(tids[i], i % NUM_HARTS);
		k_thread_start(tids[i]);
	}
	for (int i = 0; i < NUM_THREADS; i++) {
		k_thread_join(tids[i], K_FOREVER);
	}

	printf("RESULT: %d/%d threads PASS\n", (int)atomic_get(&pass_count), NUM_THREADS);
	printf("All threads finished execution.\n");
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
