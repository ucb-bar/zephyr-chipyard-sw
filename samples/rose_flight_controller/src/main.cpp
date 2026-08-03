/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE flight controller: a step toward a real onboard flight stack. Unlike
 * samples/rose/drone_control (which is handed the full ground-truth state), this guest
 * receives only what real sensors measure over the RoSE bridge, runs a STATE ESTIMATOR
 * to reconstruct the vehicle state, and only then runs TinyMPC.
 *
 * Per control step (paired with IsaacCrazyflieSensorEnv-v0):
 *   1. request IMU   -> reqrsp cmd 0x12 -> 6 float32 on ch2  [ax,ay,az, gx,gy,gz] (body)
 *   2. request FLOW  -> reqrsp cmd 0x13 -> 3 float32 on ch1  [vx,vy,h] (flow + ToF height)
 *   3. estimator.update(accel, gyro, flow, height, dt) -> 12-DoF state estimate
 *   4. subtract the hover setpoint -> regulation error
 *   5. solve TinyMPC
 *   6. return 4 normalized motor thrusts -> TX cmd 0x20
 *
 * Because the sensor set has no absolute position/heading reference, the estimated pose
 * (and thus the vehicle) drifts laterally and in yaw over time — expected. Altitude holds
 * as well as the accelerometer + known takeoff height allow.
 *
 * TinyMPC controller/solver code is reused from samples/drone_control (single drone).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>

#include <rose/rose.h>
#include <rose/rose_proto.h>

#include "admm.hpp"
#include "problem_data/quadrotor_50hz_params_constrained.hpp"
#include "glob_opts.hpp"

#include "estimator.hpp"

#define NSTATES   12
#define NACTIONS  4

/* RoSE command / channel map (see config_gym_IsaacCrazyflieSensorEnv-v0.yaml) */
#define ROSE_CMD_IMU      0x12u   /* request IMU  (reqrsp) -> 6 words on ch2 */
#define ROSE_CMD_FLOW     0x13u   /* request FLOW (reqrsp) -> 2 words on ch1 */
#define ROSE_CMD_CONTROL  0x20u   /* submit control (action_latch)           */
#define ROSE_IMU_CH       2       /* reqrsp1 (ROSE_RX_DATA_2)                 */
#define ROSE_FLOW_CH      1       /* reqrsp0 (ROSE_RX_DATA_1)                 */

/* Loop timing + setpoint (must match the env: 50 Hz, takeoff z=0.5, hover z=1.0). */
/* Control period. MUST match the co-sim rate (gym_timestep = firesim_step/firesim_freq):
 * 0.02 = 50 Hz, 0.005 = 200 Hz. The TinyMPC LQR gain is rate-tolerant, so running the
 * 50 Hz policy at a higher rate just tightens the loop (better phase margin for the fast
 * attitude dynamics). */
#define CTRL_DT      0.005f
/* Start near the hover setpoint: from the estimated state the controller cannot brake a
 * hard max-thrust takeoff without overshoot (unlike the ground-truth loop), so a gentle
 * initial transient keeps the estimator-in-the-loop stable. */
#define START_Z      0.9f
#define TARGET_Z     1.0f
#define CTRL_ITERS   5000    /* bounded by max_sim_time; ~25 s at 200 Hz */

static const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);

/* TinyMPC (single drone) */
static TinyCache     cache;
static TinyWorkspace work;
static TinySettings  settings;
static TinySolver    solver;

/* State estimator: build-time-selected pluggable filter (EKF by default). */
static IStateEstimator &est = active_estimator();

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

/* Request one reqrsp sensor packet and read @p nwords float32 words from @p channel. */
static int recv_sensor(uint32_t cmd, int channel, float *out, int nwords)
{
	uint32_t raw[NSTATES];

	rose_request(rose, cmd, 0);                      /* [cmd][num_bytes=0] */
	int n = rose_recv_reqrsp(rose, channel, raw, nwords);
	if (n < nwords) {
		return n;
	}
	for (int i = 0; i < nwords; i++) {
		memcpy(&out[i], &raw[i], sizeof(float));     /* words are float32 bits */
	}
	return nwords;
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
		printk("ROSE flight_controller: FAIL (device not ready)\n");
		return -1;
	}
	enable_vector_operations();
	mpc_init();
	est.init(0.0f, 0.0f, START_Z);   /* known takeoff pose */

	/* Hover setpoint the estimated state is regulated to (TinyMPC drives error -> 0). */
	const float setpoint[NSTATES] = {0.0f, 0.0f, TARGET_Z, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	printk("ROSE flight_controller: estimator=%s + TinyMPC ready, entering control loop\n",
	       est.name());

	float imu[6];        /* [ax,ay,az, gx,gy,gz]     */
	float flow[3];       /* [vx,vy, h] (flow + ToF)  */
	float state[NSTATES];
	float err[NSTATES];
	float u[NACTIONS];

	for (int iter = 0; iter < CTRL_ITERS; iter++) {
		if (recv_sensor(ROSE_CMD_IMU, ROSE_IMU_CH, imu, 6) < 6) {
			printk("ROSE flight_controller: short IMU read\n");
			continue;
		}
		if (recv_sensor(ROSE_CMD_FLOW, ROSE_FLOW_CH, flow, 3) < 3) {
			printk("ROSE flight_controller: short FLOW read\n");
			continue;
		}

		/* Estimate the full state from the sensors, then form the regulation error.
		 * flow[0..1] = body horizontal velocity, flow[2] = ToF height above ground. */
		uint64_t c0;
		__asm__ volatile("rdcycle %0" : "=r"(c0));
		est.update(&imu[0], &imu[3], flow, flow[2], CTRL_DT);
		est.get_state(state);
		for (int i = 0; i < NSTATES; i++) {
			err[i] = state[i] - setpoint[i];
		}
		/* Horizontal position (x,y) is NOT observable from this sensor set (flow gives
		 * velocity, not absolute position), so the dead-reckoned x,y drift. Regulating
		 * that drifting estimate makes TinyMPC chase a phantom and destabilize. Instead
		 * regulate horizontal VELOCITY only (leave err[6],err[7]=vx,vy): zero the x,y
		 * position error so the vehicle holds level and damps velocity, drifting slowly
		 * in position — the correct behavior for optical-flow-only sensing. */
		err[0] = 0.0f;
		err[1] = 0.0f;

		matsetv(work.x.vector[0], err, 1, NSTATES);
		matset(work.y.data, 0.0, work.y.outer, work.y.inner);
		matset(work.g.data, 0.0, work.g.outer, work.g.inner);

		tiny_solve(&solver);
		uint64_t c1;
		__asm__ volatile("rdcycle %0" : "=r"(c1));

		for (int i = 0; i < NACTIONS; i++) {
			u[i] = work.u.vector[0][i];
		}
		if ((iter % 200) == 10) {
			printk("ROSE flight_controller: compute cycles (estimator+solve) = %u\n",
			       (unsigned)(c1 - c0));
		}
		if ((iter % 10) == 0) {
			printk("ROSE flight_controller: iter=%d z_est=%d.%03d z_err=%d.%03d "
			       "u0=%d.%03d\n", iter,
			       (int)state[2], (int)(state[2] * 1000) % 1000,
			       (int)err[2], (int)(err[2] * 1000) % 1000,
			       (int)u[0], (int)(u[0] * 1000) % 1000);
		}
		send_control(u);
	}
	printk("ROSE flight_controller: control loop done (%d iters)\n", CTRL_ITERS);
	return 0;
}
