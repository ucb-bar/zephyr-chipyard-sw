/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared drone flight controller — one application, two targets.
 *
 * Sensor input goes through the STANDARD Zephyr sensor API (a named IMU / optical-flow /
 * ToF device via device-tree aliases), so this exact code runs:
 *   - in RoSE co-sim  : aliases bind to the virtual ucbbar,rose-* drivers (data over the
 *                       RoSE bridge from the Isaac Sim virtual sensors);
 *   - on real hardware: aliases bind to the real bosch,bmi08x-* / st,vl53l1x / flow
 *                       drivers over I2C/SPI (ESP32C6 "riskybird" board).
 * Only the board overlay + prj.conf differ; main, the estimator (IStateEstimator), and
 * TinyMPC are byte-for-byte shared. See docs/ROSE_SENSOR_ABSTRACTION.md.
 *
 * The only target-specific code here is the actuator OUTPUT (a RoSE-bridge TX packet in
 * co-sim vs PWM motors on hardware) — actuator parity is future work; the sensor/estimator/
 * control path is fully shared.
 *
 * Per control step (200 Hz):
 *   1. sample_fetch/channel_get IMU (accel+gyro), optical flow, and (low-rate) ToF height
 *   2. estimator.update(...) -> 12-DoF state; ToF fused only on fresh samples (multi-rate)
 *   3. subtract the hover setpoint (regulate velocity, not the unobservable x/y position)
 *   4. TinyMPC -> 4 normalized motor thrusts -> actuator output
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>

#include "admm.hpp"
#include "problem_data/quadrotor_50hz_params_constrained.hpp"
#include "glob_opts.hpp"

#include "estimator.hpp"
#include <rose/rose_sensor.h>   /* private optical-flow channels */

#define NSTATES   12
#define NACTIONS  4

/* Control period — MUST match the co-sim rate (gym_timestep = firesim_step/firesim_freq):
 * 0.005 = 200 Hz. The 50 Hz TinyMPC LQR gain is rate-tolerant; running it faster tightens
 * the loop (phase margin for the fast attitude dynamics with the estimator in the loop). */
#define CTRL_DT      0.005f
#define START_Z      0.9f     /* gentle takeoff from near the setpoint */
#define TARGET_Z     1.0f
#define CTRL_ITERS   5000     /* bounded by max_sim_time / run time */

/* ---- Sensor devices (Zephyr sensor API; bound per board overlay) ---- */
static const struct device *accel_dev = DEVICE_DT_GET(DT_ALIAS(bmi088_accel));
static const struct device *gyro_dev  = DEVICE_DT_GET(DT_ALIAS(bmi088_gyro));

#define HAVE_FLOW DT_NODE_EXISTS(DT_ALIAS(flow))
#define HAVE_TOF  DT_NODE_EXISTS(DT_ALIAS(tof))
#if HAVE_FLOW
static const struct device *flow_dev = DEVICE_DT_GET(DT_ALIAS(flow));
#endif
#if HAVE_TOF
static const struct device *tof_dev  = DEVICE_DT_GET(DT_ALIAS(tof));
#endif

/* ---- Actuator output: RoSE bridge (co-sim) vs PWM motors (real) ---- */
#define HAVE_ROSE DT_HAS_COMPAT_STATUS_OKAY(ucbbar_roseadapter)
#if HAVE_ROSE
#include <rose/rose.h>
#define ROSE_CMD_CONTROL 0x20u
static const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
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
#else /* real target: drive 4 PWM motors (thrust ~ duty). Actuator parity is future work. */
#include <zephyr/drivers/pwm.h>
#define MOTORS_NODE DT_ALIAS(motors)
#if DT_NODE_EXISTS(MOTORS_NODE)
static const struct pwm_dt_spec motors[NACTIONS] = {
	PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 0),
	PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 1),
	PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 2),
	PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 3),
};
static void send_control(const float *u)
{
	for (int i = 0; i < NACTIONS; i++) {
		/* normalized thrust u in ~[-0.583, 0.417] -> [0,1] duty */
		float duty = u[i] + 0.583f;
		if (duty < 0.0f) duty = 0.0f;
		if (duty > 1.0f) duty = 1.0f;
		pwm_set_pulse_dt(&motors[i], (uint32_t)(motors[i].period * duty));
	}
}
#else
static void send_control(const float *u) { (void)u; /* no actuator bound */ }
#endif
#endif

/* TinyMPC (single drone) */
static TinyCache     cache;
static TinyWorkspace work;
static TinySettings  settings;
static TinySolver    solver;

/* State estimator: build-time-selected pluggable filter (default complementary). */
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

/* Read a 3-axis channel into a float[3] via the Zephyr sensor API. */
static int read_xyz(const struct device *dev, enum sensor_channel chan, float out[3])
{
	struct sensor_value v[3];
	int rc = sensor_sample_fetch(dev);
	if (rc < 0) {
		return rc;
	}
	rc = sensor_channel_get(dev, chan, v);
	if (rc < 0) {
		return rc;
	}
	for (int i = 0; i < 3; i++) {
		out[i] = (float)sensor_value_to_double(&v[i]);
	}
	return 0;
}

int main(void)
{
	if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev)) {
		printk("flight_controller: FAIL (IMU not ready)\n");
		return -1;
	}
	enable_vector_operations();
	mpc_init();
	est.init(0.0f, 0.0f, START_Z);

	const float setpoint[NSTATES] = {0.0f, 0.0f, TARGET_Z, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	printk("flight_controller: estimator=%s + TinyMPC ready (%s), entering control loop\n",
	       est.name(), HAVE_ROSE ? "RoSE co-sim" : "real target");

	float accel[3], gyro[3];
	float flow[2] = {0.0f, 0.0f};
	float height = START_Z;
	float state[NSTATES], err[NSTATES], u[NACTIONS];

	for (int iter = 0; iter < CTRL_ITERS; iter++) {
		if (read_xyz(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel) < 0 ||
		    read_xyz(gyro_dev,  SENSOR_CHAN_GYRO_XYZ,  gyro) < 0) {
			printk("flight_controller: IMU read error\n");
			continue;
		}

#if HAVE_FLOW
		if (sensor_sample_fetch(flow_dev) == 0) {
			struct sensor_value vx, vy;
			sensor_channel_get(flow_dev, (enum sensor_channel)ROSE_SENSOR_CHAN_FLOW_VX, &vx);
			sensor_channel_get(flow_dev, (enum sensor_channel)ROSE_SENSOR_CHAN_FLOW_VY, &vy);
			flow[0] = (float)sensor_value_to_double(&vx);
			flow[1] = (float)sensor_value_to_double(&vy);
		}
#endif
		/* ToF is low-rate: fetch returns 0 only when a fresh sample is available, else
		 * -EAGAIN. Fuse altitude only on fresh samples (multi-rate estimation). */
		bool tof_valid = false;
#if HAVE_TOF
		if (sensor_sample_fetch(tof_dev) == 0) {
			struct sensor_value h;
			sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &h);
			height = (float)sensor_value_to_double(&h);
			tof_valid = true;
		}
#endif

		est.update(accel, gyro, flow, height, tof_valid, CTRL_DT);
		est.get_state(state);
		for (int i = 0; i < NSTATES; i++) {
			err[i] = state[i] - setpoint[i];
		}
		/* Horizontal position is unobservable (flow gives velocity) -> regulate velocity
		 * only, let x/y position dead-reckon (drift). Zero the x/y position error. */
		err[0] = 0.0f;
		err[1] = 0.0f;

		matsetv(work.x.vector[0], err, 1, NSTATES);
		matset(work.y.data, 0.0, work.y.outer, work.y.inner);
		matset(work.g.data, 0.0, work.g.outer, work.g.inner);

		tiny_solve(&solver);

		for (int i = 0; i < NACTIONS; i++) {
			u[i] = work.u.vector[0][i];
		}
		if ((iter % 10) == 0) {
			printk("flight_controller: iter=%d z_est=%d.%03d z_err=%d.%03d u0=%d.%03d\n",
			       iter,
			       (int)state[2], (int)(state[2] * 1000) % 1000,
			       (int)err[2], (int)(err[2] * 1000) % 1000,
			       (int)u[0], (int)(u[0] * 1000) % 1000);
		}
		send_control(u);
	}
	printk("flight_controller: control loop done (%d iters)\n", CTRL_ITERS);
	return 0;
}
