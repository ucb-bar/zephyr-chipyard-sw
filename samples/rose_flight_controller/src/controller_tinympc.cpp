/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * TinyMPC controller implementation. Solver caches/workspace + init and one-step solve, moved
 * verbatim from main.cpp behind the IController interface (behaviour unchanged).
 */
#include "controller_tinympc.hpp"

#include <zephyr/sys/printk.h>

#include "admm.hpp"
#include "problem_data/quadrotor_50hz_params_constrained.hpp"
#include "glob_opts.hpp"

/* TinyMPC (single drone) -- file-scope so no __cxa_guard_* (minimal libcpp). */
static TinyCache     cache;
static TinyWorkspace work;
static TinySettings  settings;
static TinySolver    solver;

void TinympcController::init()
{
	enable_vector_operations();

	solver.cache    = &cache;
	solver.work     = &work;
	solver.settings = &settings;
	tiny_init(&solver);

	init_VectorNx(&work.x1);
	init_VectorNx(&work.x2);
	init_VectorNx(&work.x3);
	init_VectorNu(&work.u1);
	init_VectorNu(&work.u2);

	cache.rho = rho_value;
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

	matset(work.u_min.data, -0.583, work.u_min.outer, work.u_min.inner);
	matset(work.u_max.data, 1 - 0.583, work.u_max.outer, work.u_max.inner);
	matset(work.x_min.data, -5, work.x_min.outer, work.x_min.inner);
	matset(work.x_max.data, 5, work.x_max.outer, work.x_max.inner);

	float Xref_origin[NSTATES] = {0};
	for (int j = 0; j < NHORIZON; j++) {
		matsetv(work.Xref.vector[j], Xref_origin, 1, NSTATES);
	}
}

void TinympcController::compute(const float state[CTRL_NSTATES], const float setpoint[CTRL_NSTATES],
			       float u_out[CTRL_NACTIONS], float dt)
{
	(void)dt;   /* TinyMPC gains are baked at the 50 Hz design rate */
	float err[NSTATES];
	for (int i = 0; i < NSTATES; i++) {
		err[i] = state[i] - setpoint[i];
	}
	err[0] = 0.0f; err[1] = 0.0f;   /* x/y position unobservable from flow -> regulate velocity */
	matsetv(work.x.vector[0], err, 1, NSTATES);
	matset(work.y.data, 0.0, work.y.outer, work.y.inner);
	matset(work.g.data, 0.0, work.g.outer, work.g.inner);
	tiny_solve(&solver);

	/*
	 * PROJECT THE ANSWER ONTO THE BOX THE SOLVER WAS GIVEN.
	 *
	 * What went wrong. init() tells the solver u in [-0.583, 0.417] (i.e. per-motor duty in
	 * [0, 1]), and ADMM enforces that on the SLACK only -- update_slack_1() in admm_rvv.hpp
	 * clamps znew and nothing ever clamps u. The two agree only at convergence, and this
	 * build gets settings->max_iter = 10. Measured on the host with this exact problem data
	 * and a bench state (drone on the floor, TARGET_Z = 1.0 m): the returned u is
	 * [1.260 1.284 1.247 1.218] against a bound of 0.417 -- a 3.1x violation, matching the
	 * 0.98-1.62 seen on hardware. It is pure non-convergence, not a porting fault: the same
	 * call with max_iter 50 / 200 / 5000 returns 0.520 / 0.427 / 0.418, converging (status 1,
	 * primal_residual_input = abs_pri_tol) onto the bound. 200+ iterations is not available
	 * at 25-35 ms for ten, so the budget cannot be the fix.
	 *
	 * Why not simply return the slack z. z is feasible by construction and it is the obvious
	 * candidate, but z = clamp(u + y) is clamped PER MOTOR, so when the collective demand
	 * exceeds the ceiling every element pins to u_max and the four commands become IDENTICAL
	 * -- zero roll/pitch/yaw authority exactly when the vehicle is working hardest. Host
	 * numbers for the partially-saturated case (15 cm altitude error): u spreads 0.0733 across
	 * the four motors, z spreads 0.0000, a per-motor clamp of u spreads 0.0267. That flattening
	 * is the same failure actuator_duty() in main.cpp was written to avoid. (z is also stale by
	 * one iteration on the early-convergence path, which returns before `z = znew`.)
	 *
	 * What this does instead: subtract the common excess so the PEAK lands on u_max, then clamp
	 * what remains into the box. That is a collective (thrust) reduction with the differential
	 * (attitude) preserved -- spread 0.0733 in the case above, i.e. all of it.
	 *
	 * It is deliberately the same operation main.cpp's anti-saturation cut already performs one
	 * layer down, and composes with it exactly rather than competing: because u_max + 0.583 = 1,
	 * subtracting (peak - u_max) here and subtracting (peak_duty - ceiling) there are the same
	 * map, and the algebra survives the battery scale applied between them. Verified on the
	 * host: for every case tried, the duty[] that main.cpp produces is bit-identical with and
	 * without this projection. So no motor sees a different number than it does today -- what
	 * changes is that the CONTROLLER no longer hands out a command 3x outside the constraint it
	 * was configured with, which is what any other consumer (telemetry, the ESP link encoder,
	 * the flight log) has been reading.
	 *
	 * In the unsaturated regime this is a no-op: nothing exceeds u_max, no shift is taken, and
	 * u_out is bit-identical to what this function returned before. Host check at a realistic
	 * hover (2 cm altitude error, ~2 deg tilt): ADMM converges in under 10 iterations and
	 * u == z == clamp(u) == this == [0.0856 0.0955 0.0399 -0.0046].
	 *
	 * Bounds are read from the workspace, not written as literals, so they cannot drift from
	 * what init() actually told the solver.
	 */
	float excess = 0.0f;
	for (int i = 0; i < CTRL_NACTIONS; i++) {
		const float over = work.u.vector[0][i] - work.u_max.vector[0][i];

		if (over > excess) {
			excess = over;
		}
	}
	for (int i = 0; i < CTRL_NACTIONS; i++) {
		const float lo = work.u_min.vector[0][i];
		const float hi = work.u_max.vector[0][i];
		float v = work.u.vector[0][i] - excess;

		if (v < lo) { v = lo; }
		if (v > hi) { v = hi; }
		u_out[i] = v;
	}
	if (excess > 0.0f) {
		/* Once per boot, not per iteration: this loop busy-waits on printk on the FPGA
		 * carrier. Saturating is not itself a fault -- but a build that saturates on its
		 * FIRST solve is being asked for thrust it cannot deliver (the stock TARGET_Z is
		 * 1.0 m, which is a 1 m step for a drone on the bench), and that is worth one line
		 * in the boot log rather than being invisible. */
		static bool said;

		if (!said) {
			said = true;
			printk("TinyMPC: input bound saturated -- ADMM returned a command %d milli "
			       "above u_max after %d iterations; collective thrust cut, attitude "
			       "differential preserved. Lower TARGET_Z or use -DROSE_USE_PID=1.\n",
			       (int)(excess * 1000.0f), settings.max_iter);
		}
	}
}
