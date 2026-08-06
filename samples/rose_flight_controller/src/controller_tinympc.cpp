/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * TinyMPC controller implementation. Solver caches/workspace + init and one-step solve, moved
 * verbatim from main.cpp behind the IController interface (behaviour unchanged).
 */
#include "controller_tinympc.hpp"

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
	for (int i = 0; i < CTRL_NACTIONS; i++) {
		u_out[i] = work.u.vector[0][i];
	}
}
