/*
 * Standalone fp16-vectorized TinyMPC benchmark for the FP16-only Saturn Rocket
 * (misa.F=0) on the Arty-200T bench. Single-thread, no HTIF, no worker server.
 * Times tiny_solve() with rdcycle and prints raw cycle deltas over the SiFive UART.
 *
 * Boot/FP notes (see reset.S fscsr trap): built CONFIG_FPU=n so Zephyr's boot
 * does NOT execute `fscsr zero` (illegal on misa.F=0). We enable FS+VS ourselves
 * via enable_vector_operations() (csrs mstatus, no fcsr access). MATLIB_NARROW_ACC=1
 * keeps the dot-product accumulator in fp16 so no scalar fp32 is emitted, and
 * check_termination > max_iter keeps matnorm's sqrtf() fp32 path out of the loop.
 */
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <stdint.h>

#include "admm.hpp"
#include "problem_data/quadrotor_50hz_params_constrained.hpp"
#include "glob_opts.hpp"

#ifndef NRUNS
#define NRUNS 32
#endif
#ifndef BENCH_MAX_ITER
#define BENCH_MAX_ITER 10
#endif

static TinyCache     cache;
static TinyWorkspace work;
static TinySettings  settings;
static TinySolver    solver;

static inline uint64_t rdcycle64(void) {
	uint64_t c;
	__asm__ volatile("rdcycle %0" : "=r"(c));
	return c;
}

static void load_problem(void)
{
	solver.cache = &cache;
	solver.work = &work;
	solver.settings = &settings;
	tiny_init(&solver);

	init_VectorNx(&work.x1);
	init_VectorNx(&work.x2);
	init_VectorNx(&work.x3);
	init_VectorNu(&work.u1);
	init_VectorNu(&work.u2);

	/* rho is the compile-time constant rho_value (63.0). Store its fp16 bits via
	 * integer writes (reading the rho_value global or storing _Float16 would emit
	 * flh/fsh, which trap on the vector-fp16-only core). matmulf_bits uses rho_bits. */
	cache.rho_bits = __builtin_bit_cast(unsigned short, (tinytype)63.0f);
	*(unsigned short *)&cache.rho = cache.rho_bits;
	matsetv(cache.Kinf.data, Kinf_data, cache.Kinf.outer, cache.Kinf.inner);
	transpose(cache.Kinf.data, cache.KinfT.data, NINPUTS, NSTATES);
	matsetv(cache.Pinf.data, Pinf_data, cache.Pinf.outer, cache.Pinf.inner);
	transpose(cache.Pinf.data, cache.PinfT.data, NSTATES, NSTATES);
	matsetv(cache.Quu_inv.data, Quu_inv_data, cache.Quu_inv.outer, cache.Quu_inv.inner);
	matsetv(cache.AmBKt.data, AmBKt_data, cache.AmBKt.outer, cache.AmBKt.inner);
	transpose(cache.AmBKt.data, cache.AmBKtT.data, NSTATES, NSTATES);
	matsetv(cache.coeff_d2p.data, coeff_d2p_data, cache.coeff_d2p.outer, cache.coeff_d2p.inner);

	matsetv(work.Adyn.data, Adyn_data, work.Adyn.outer, work.Adyn.inner);
	transpose(work.Adyn.data, work.AdynT.data, NSTATES, NSTATES);
	matsetv(work.Bdyn.data, Bdyn_data, work.Bdyn.outer, work.Bdyn.inner);
	transpose(work.Bdyn.data, work.BdynT.data, NSTATES, NINPUTS);
	matsetv(work.Q.data, Q_data, work.Q.outer, work.Q.inner);
	matsetv(work.R.data, R_data, work.R.outer, work.R.inner);

	matset(work.u_min.data, (tinytype)-0.583f, work.u_min.outer, work.u_min.inner);
	matset(work.u_max.data, (tinytype)(1.0f - 0.583f), work.u_max.outer, work.u_max.inner);
	matset(work.x_min.data, (tinytype)-5.0f, work.x_min.outer, work.x_min.inner);
	matset(work.x_max.data, (tinytype)5.0f, work.x_max.outer, work.x_max.inner);

	tinytype Xref_origin[NSTATES] = {0};
	for (int j = 0; j < NHORIZON; j++) {
		matsetv(work.Xref.vector[j], Xref_origin, 1, NSTATES);
	}

	/* Pinf > 65504 overflows fp16 -> +inf; zero it (valid only for Xref=0,
	 * since PinfT*Xref[N-1] is then mathematically 0). Same asterisk as the
	 * validated SIL build. Does not change the solve's op/cycle count. */
	matset(cache.Pinf.data, (tinytype)0.0f, cache.Pinf.outer, cache.Pinf.inner);
	matset(cache.PinfT.data, (tinytype)0.0f, cache.PinfT.outer, cache.PinfT.inner);

	/* Fixed ADMM iteration count for a clean benchmark: force exactly
	 * max_iter iterations and keep the fp32 sqrtf residual path out of the loop. */
	settings.max_iter = BENCH_MAX_ITER;
	settings.check_termination = BENCH_MAX_ITER + 1;
	settings.en_state_bound = 1;
	settings.en_input_bound = 1;
}

int main(void)
{
	enable_vector_operations();  /* set mstatus FS|VS|XS; no fcsr write */
	load_problem();

	/* representative displaced-hover initial state (Xref = origin), given as fp16
	 * bit patterns so no scalar _Float16 store (fsh) is emitted: 0.5=0x3800,
	 * 1.0=0x3C00, 0.0=0x0000. matsetv copies it into the solver via vle16/vse16. */
	static const unsigned short state_bits[NSTATES] = {
		0x3800, 0x3800, 0x3C00, 0, 0, 0, 0, 0, 0, 0, 0, 0
	};
	const tinytype *state = (const tinytype *)state_bits;

	printk("TINYMPC_FP16_BENCH start: NSTATES=%d NINPUTS=%d NHORIZON=%d max_iter=%d runs=%d\n",
	       NSTATES, NINPUTS, NHORIZON, (int)settings.max_iter, NRUNS);

	uint64_t best = ~0ULL, worst = 0, sum = 0;
	int last_iter = 0, last_status = 0;
	unsigned short u0bits = 0;

	for (int r = 0; r < NRUNS; r++) {
		matsetv(work.x.vector[0], (tinytype *)state, 1, NSTATES);
		matset(work.y.data, (tinytype)0.0f, work.y.outer, work.y.inner);
		matset(work.g.data, (tinytype)0.0f, work.g.outer, work.g.inner);

		unsigned int key = irq_lock();
		uint64_t c0 = rdcycle64();
		tiny_solve(&solver);
		uint64_t c1 = rdcycle64();
		irq_unlock(key);

		uint64_t d = c1 - c0;
		if (d < best) best = d;
		if (d > worst) worst = d;
		sum += d;
		last_iter = work.iter;
		last_status = work.status;
		/* read u[0] as raw fp16 bits via an integer load (no scalar flh) */
		u0bits = *(const unsigned short *)&work.u.vector[0][0];
		printk("run %2d: %lu cycles (iters=%d status=%d)\n",
		       r, (unsigned long)d, work.iter, work.status);
	}

	/* u[0] as raw fp16 bits (0x7c00=+inf, 0x7e00=nan) to sanity-check the solution */
	printk("TINYMPC_FP16_BENCH done: best=%lu avg=%lu worst=%lu cycles/solve @%d iters | u[0]bits=0x%04x status=%d\n",
	       (unsigned long)best, (unsigned long)(sum / NRUNS), (unsigned long)worst,
	       last_iter, (unsigned int)u0bits, last_status);

	while (1) {
		k_msleep(1000);
	}
	return 0;
}
