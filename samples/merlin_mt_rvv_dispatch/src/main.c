/*
 * Merlin multicore RVV dispatch executor (Zephyr SMP).
 *
 * Runs a Merlin-style level-synchronous dispatch schedule across harts: a fixed
 * thread pool (one worker pinned per hart) executes the dispatches of one
 * dependency level concurrently, barriers, then advances to the next level —
 * exactly the structure that merlin's `lowering/schedule_dispatch.py` emits
 * (levels + per-node hart assignment) for an outlined dispatch program.
 *
 * The schedule here is the canonical "diamond": two independent matmuls
 *      P = A * X   (level 0, hart 0)
 *      Q = B * X   (level 0, hart 1)   <- runs concurrently with P
 *      R = P * Q   (level 1, hart 0)   <- waits on both via the level barrier
 * Each matmul dispatch is an RVV kernel (rank-1 vfmacc updates, `.option arch,+v`
 * inline asm), so several V-using worker threads run on distinct harts and rely on
 * arch/riscv/core/v.c to keep their vector state separate. R is validated against a
 * scalar reference.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/arch/cpu.h>
#include <stdio.h>
#include <math.h>

#define NUM_HARTS  CONFIG_MP_MAX_NUM_CPUS
#define DIM        32
#define STACK_SIZE 16384

static float A[DIM * DIM], B[DIM * DIM], X[DIM * DIM];
static float P[DIM * DIM], Q[DIM * DIM], R[DIM * DIM];

/* row[i] += a * src[i], i in [0,n).  RVV vfmacc, e32/m8, strip-mined.
 * No vector-register clobbers (GCC 12.2.0 can't name them). */
static void rvv_axpy(float a, const float *src, float *row, int n)
{
	const float *sp = src;
	float *rp = row;
	int nn = n;

	__asm volatile(
		".option push\n\t.option arch, +v\n\t"
		"1:\n\t"
		"vsetvli t0, %[n], e32, m8, ta, ma\n\t"
		"vle32.v v8, (%[s])\n\t"
		"vle32.v v16, (%[r])\n\t"
		"vfmacc.vf v16, %[a], v8\n\t"
		"vse32.v v16, (%[r])\n\t"
		"slli t1, t0, 2\n\t"
		"add %[s], %[s], t1\n\t"
		"add %[r], %[r], t1\n\t"
		"sub %[n], %[n], t0\n\t"
		"bnez %[n], 1b\n\t"
		".option pop\n\t"
		: [s] "+r"(sp), [r] "+r"(rp), [n] "+r"(nn)
		: [a] "f"(a)
		: "t0", "t1", "memory");
}

/* C[MxN] = Av[MxK] * Bv[KxN], via M*K RVV rank-1 updates into the zeroed rows of C. */
static void matmul_rvv(const float *Av, const float *Bv, float *C, int M, int K, int N)
{
	for (int i = 0; i < M * N; i++) {
		C[i] = 0.0f;
	}
	for (int i = 0; i < M; i++) {
		for (int k = 0; k < K; k++) {
			rvv_axpy(Av[i * K + k], &Bv[k * N], &C[i * N], N);
		}
	}
}

typedef struct {
	int level, hart;
	const float *Av, *Bv;
	float *C;
	int M, K, N;
} dispatch_t;

/* The schedule a Merlin partition would hand us: {level, hart, op operands}. */
static dispatch_t SCHED[] = {
	{0, 0, A, X, P, DIM, DIM, DIM},   /* P = A*X */
	{0, 1, B, X, Q, DIM, DIM, DIM},   /* Q = B*X (concurrent with P) */
	{1, 0, P, Q, R, DIM, DIM, DIM},   /* R = P*Q (after the level-0 barrier) */
};
#define N_DISPATCH ((int)(sizeof(SCHED) / sizeof(SCHED[0])))

typedef struct {
	dispatch_t *task;
	struct k_sem start, done;
} worker_t;

static worker_t workers[NUM_HARTS];
K_THREAD_STACK_ARRAY_DEFINE(stacks, NUM_HARTS, STACK_SIZE);
static struct k_thread wthreads[NUM_HARTS];

static void worker_fn(void *a1, void *a2, void *a3)
{
	worker_t *w = &workers[(int)(uintptr_t)a1];

	while (1) {
		k_sem_take(&w->start, K_FOREVER);
		dispatch_t *d = w->task;
		matmul_rvv(d->Av, d->Bv, d->C, d->M, d->K, d->N);
		k_sem_give(&w->done);
	}
}

int main(void)
{
	for (int i = 0; i < DIM * DIM; i++) {
		int r = i / DIM, c = i % DIM;
		A[i] = (float)((r + c) % 3);
		B[i] = (float)((r * 2 + c) % 4 - 1);
		X[i] = (float)((r + 2 * c) % 5 - 2);
	}

	printf("Merlin multicore RVV dispatch executor: %d dispatches over %d harts (%s)\n",
	       N_DISPATCH, NUM_HARTS, CONFIG_BOARD_TARGET);

	for (int h = 0; h < NUM_HARTS; h++) {
		k_sem_init(&workers[h].start, 0, 1);
		k_sem_init(&workers[h].done, 0, 1);
		k_tid_t tid = k_thread_create(&wthreads[h], stacks[h], STACK_SIZE, worker_fn,
					      (void *)(uintptr_t)h, NULL, NULL,
					      K_PRIO_PREEMPT(1), 0, K_FOREVER);
		k_thread_cpu_pin(tid, h);
		k_thread_start(tid);
	}

	int max_level = 0;
	for (int i = 0; i < N_DISPATCH; i++) {
		if (SCHED[i].level > max_level) {
			max_level = SCHED[i].level;
		}
	}

	for (int level = 0; level <= max_level; level++) {
		bool used[NUM_HARTS] = {false};
		int count = 0;
		for (int i = 0; i < N_DISPATCH; i++) {
			if (SCHED[i].level != level) {
				continue;
			}
			int h = SCHED[i].hart;
			workers[h].task = &SCHED[i];
			used[h] = true;
			k_sem_give(&workers[h].start);
			count++;
		}
		for (int h = 0; h < NUM_HARTS; h++) {
			if (used[h]) {
				k_sem_take(&workers[h].done, K_FOREVER);   /* level barrier */
			}
		}
		printf("level %d: %d dispatch(es) done\n", level, count);
	}

	/* scalar reference: R = (A*X) * (B*X) */
	static float Pr[DIM * DIM], Qr[DIM * DIM], Rr[DIM * DIM];
	for (int i = 0; i < DIM; i++) {
		for (int j = 0; j < DIM; j++) {
			float p = 0, q = 0;
			for (int k = 0; k < DIM; k++) {
				p += A[i * DIM + k] * X[k * DIM + j];
				q += B[i * DIM + k] * X[k * DIM + j];
			}
			Pr[i * DIM + j] = p;
			Qr[i * DIM + j] = q;
		}
	}
	for (int i = 0; i < DIM; i++) {
		for (int j = 0; j < DIM; j++) {
			float r = 0;
			for (int k = 0; k < DIM; k++) {
				r += Pr[i * DIM + k] * Qr[k * DIM + j];
			}
			Rr[i * DIM + j] = r;
		}
	}

	int errors = 0;
	for (int i = 0; i < DIM * DIM; i++) {
		if (fabsf(R[i] - Rr[i]) > 1e-2f * (1.0f + fabsf(Rr[i]))) {
			errors++;
		}
	}
	printf("RESULT: %s (%d/%d elems correct)\n", errors ? "FAIL" : "PASS",
	       DIM * DIM - errors, DIM * DIM);
	printf("All dispatches finished execution.\n");
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
