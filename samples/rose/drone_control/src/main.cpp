/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE drone control: a RoSE co-simulation port of samples/drone_control's TinyMPC
 * HIL controller. The upstream sample talks to a physics host over a physical UART;
 * this one talks to the RoSE bridge instead, and gets the FULL simulator state
 * directly (12-DoF linearized quadrotor state) rather than a rose-imu abstraction.
 *
 * Per control step (paired with PyBulletDroneMPCEnv-v0):
 *   1. request the full state  ->  reqrsp cmd 0x12  ->  12 float32 on channel 2
 *   2. solve TinyMPC
 *   3. return 4 normalized motor thrusts  ->  TX cmd 0x20  ->  applied as the env action
 *
 * State vector (matches scripts/pybullet_hil.py / the env):
 *   [x, y, z, r1, r2, r3, vx, vy, vz, dphi, dtheta, dpsi]   (Rodrigues attitude)
 *
 * Controller/solver code is reused verbatim from samples/drone_control/main_mt_binary.cpp
 * (single-drone). Requires the tinympc submodule; see README.rst.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>

#include <rose/rose.h>
#include <rose/rose_proto.h>

#include "admm.hpp"
#include "problem_data/quadrotor_50hz_params_constrained.hpp"
#include "glob_opts.hpp"

#define NSTATES   12
#define NACTIONS  4

/* RoSE command / channel map (see deploy/config/config_gym_PyBulletDroneMPCEnv-v0.yaml) */
#define ROSE_CMD_STATE    0x12u   /* request full state (reqrsp) */
#define ROSE_CMD_CONTROL  0x20u   /* submit control (action_latch) */
#define ROSE_STATE_CH     2       /* reqrsp1 (ROSE_RX_DATA_2) */

static const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);

/* TinyMPC (single drone) */
static TinyCache     cache;
static TinyWorkspace work;
static TinySettings  settings;
static TinySolver    solver;

static void mpc_init(void)
{
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

/* Request the full 12-DoF state over the RoSE bridge; returns words read. */
static int recv_state(float *state)
{
	uint32_t raw[NSTATES];

	rose_request(rose, ROSE_CMD_STATE, 0);           /* [cmd][num_bytes=0] */
	int n = rose_recv_reqrsp(rose, ROSE_STATE_CH, raw, NSTATES);
	if (n < NSTATES) {
		return n;
	}
	for (int i = 0; i < NSTATES; i++) {
		memcpy(&state[i], &raw[i], sizeof(float));   /* words are float32 bits */
	}
	return NSTATES;
}

/* Return 4 normalized motor thrusts to the env: TX [cmd][16][u0..u3]. */
static void send_control(const float *u)
{
	rose_tx(rose, ROSE_CMD_CONTROL);
	rose_tx(rose, NACTIONS * sizeof(float));
	for (int i = 0; i < NACTIONS; i++) {
		uint32_t w;
		memcpy(&w, &u[i], sizeof(float));
		rose_tx(rose, w);
	}
}

int main(void)
{
	if (!device_is_ready(rose)) {
		printk("ROSE drone_control: FAIL (device not ready)\n");
		return -1;
	}
	enable_vector_operations();
	mpc_init();
	printk("ROSE drone_control: TinyMPC ready, entering control loop\n");

	float state[NSTATES];
	float u[NACTIONS];

	int iter = 0;
	while (1) {
		int n = recv_state(state);
		if (n < NSTATES) {
			printk("ROSE drone_control: short state read n=%d\n", n);
			continue;
		}

		matsetv(work.x.vector[0], state, 1, NSTATES);
		matset(work.y.data, 0.0, work.y.outer, work.y.inner);
		matset(work.g.data, 0.0, work.g.outer, work.g.inner);

		tiny_solve(&solver);

		for (int i = 0; i < NACTIONS; i++) {
			u[i] = work.u.vector[0][i];
		}
		if ((iter++ % 10) == 0) {
			printk("ROSE drone_control: iter=%d z_err=%d.%03d u0=%d.%03d\n", iter,
			       (int)state[2], (int)(state[2] * 1000) % 1000,
			       (int)u[0], (int)(u[0] * 1000) % 1000);
		}
		send_control(u);
	}
	return 0;
}
