/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE navigation controller — waypoint flight in a corridor using obstacle-relative
 * position from the 4x horizontal VL53L5CX multizone ToF (F/R/B/L).
 *
 * This extends the hover flight controller: same IMU + optical-flow + downward-ToF sensing,
 * same modular estimator + TinyMPC. The addition is the FOUR horizontal ToFs (read through the
 * reusable ucbbar,rose-tof-zone driver) fed to estimator.fuse_walls(): in a corridor the
 * difference of opposite wall distances is an EXACT lateral / longitudinal position relative to
 * the corridor center, which the flow-only filter lacks (flow gives velocity; x/y position
 * drifts). With x/y now observable, TinyMPC regulates POSITION -- so the controller tracks a
 * WAYPOINT (hold the corridor center, advance forward), not just hover.
 *
 * Same shared-app sim-to-real story: on real riskybird hardware the aliases bind to
 * st,vl53l5cx (4x, tested on the riskybird branch) instead of the RoSE virtual driver.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>

#include "admm.hpp"
#include "problem_data/quadrotor_50hz_params_constrained.hpp"
#include "glob_opts.hpp"

#include "estimator.hpp"
#include <rose/rose_sensor.h>

#define NSTATES   12
#define NACTIONS  4

#define CTRL_DT      0.005f
#define START_Z      0.9f
#define TARGET_Z     1.0f
#define CTRL_ITERS   6000
#define TOF_MAX_RANGE 4.0f

/* Waypoint schedule (local maneuver): settle a hover holding the corridor center, then ramp
 * the forward (x) setpoint to WAYPOINT_X and hold, while keeping the lateral setpoint at the
 * corridor center (y=0). Gentle ramp so TinyMPC stays within its constraints. */
#define SETTLE_ITERS  600      /* 3 s: acquire walls + steady hover at center */
#define RAMP_ITERS    1000     /* 5 s: ramp x 0 -> WAYPOINT_X */
#define WAYPOINT_X    1.0f      /* advance 1 m down the corridor */

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

/* Four horizontal multizone ToF (nearest wall per direction). */
static const struct device *tof_f = DEVICE_DT_GET(DT_ALIAS(tof_front));
static const struct device *tof_r = DEVICE_DT_GET(DT_ALIAS(tof_right));
static const struct device *tof_b = DEVICE_DT_GET(DT_ALIAS(tof_back));
static const struct device *tof_l = DEVICE_DT_GET(DT_ALIAS(tof_left));

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
#else
static void send_control(const float *u) { (void)u; }
#endif

static TinyCache     cache;
static TinyWorkspace work;
static TinySettings  settings;
static TinySolver    solver;
static IStateEstimator &est = active_estimator();

static void mpc_init(void)
{
	solver.cache = &cache; solver.work = &work; solver.settings = &settings;
	tiny_init(&solver);
	init_VectorNx(&work.x1); init_VectorNx(&work.x2); init_VectorNx(&work.x3);
	init_VectorNu(&work.u1); init_VectorNu(&work.u2);
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

static float read_dist(const struct device *dev)
{
	struct sensor_value v;
	if (sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &v) < 0) {
		return TOF_MAX_RANGE;
	}
	return (float)sensor_value_to_double(&v);
}

int main(void)
{
	if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev)) {
		printk("nav_controller: FAIL (IMU not ready)\n");
		return -1;
	}
	bool have_walls = device_is_ready(tof_f) && device_is_ready(tof_r) &&
			  device_is_ready(tof_b) && device_is_ready(tof_l);
	enable_vector_operations();
	mpc_init();
	est.init(0.0f, 0.0f, START_Z);

	printk("nav_controller: estimator=%s + TinyMPC ready, walls=%d, waypoint x->%d.%02d m\n",
	       est.name(), have_walls, (int)WAYPOINT_X, (int)(WAYPOINT_X * 100) % 100);

	float accel[3], gyro[3];
	float flow[2] = {0.0f, 0.0f};
	float height = START_Z;
	float state[NSTATES], err[NSTATES], u[NACTIONS];

	for (int iter = 0; iter < CTRL_ITERS; iter++) {
		/* Phase 1: issue ALL sensor requests (pipelined; low-rate ones return -EAGAIN). */
		int rc_a = sensor_sample_fetch(accel_dev);
		int rc_g = sensor_sample_fetch(gyro_dev);
#if HAVE_FLOW
		sensor_sample_fetch(flow_dev);
#endif
		bool tof_valid = false;
#if HAVE_TOF
		tof_valid = (sensor_sample_fetch(tof_dev) == 0);
#endif
		if (have_walls) {
			sensor_sample_fetch(tof_f); sensor_sample_fetch(tof_r);
			sensor_sample_fetch(tof_b); sensor_sample_fetch(tof_l);
		}
		if (rc_a < 0 || rc_g < 0) { printk("nav_controller: IMU fetch error\n"); continue; }

		/* Phase 2: collect. */
		struct sensor_value av[3], gv[3];
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, av);
		sensor_channel_get(gyro_dev,  SENSOR_CHAN_GYRO_XYZ,  gv);
		for (int i = 0; i < 3; i++) {
			accel[i] = (float)sensor_value_to_double(&av[i]);
			gyro[i]  = (float)sensor_value_to_double(&gv[i]);
		}
		bool flow_valid = true;
#if HAVE_FLOW
		{
			struct sensor_value vx, vy;
			sensor_channel_get(flow_dev, (enum sensor_channel)ROSE_SENSOR_CHAN_FLOW_VX, &vx);
			sensor_channel_get(flow_dev, (enum sensor_channel)ROSE_SENSOR_CHAN_FLOW_VY, &vy);
			flow[0] = (float)sensor_value_to_double(&vx);
			flow[1] = (float)sensor_value_to_double(&vy);
			if (flow[0] != flow[0] || flow[1] != flow[1]) {
				flow_valid = false; flow[0] = flow[1] = 0.0f;
			}
		}
#endif
#if HAVE_TOF
		if (tof_valid) {
			struct sensor_value h;
			sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &h);
			height = (float)sensor_value_to_double(&h);
		}
#endif
		est.update(accel, gyro, flow, flow_valid, height, tof_valid, CTRL_DT);
		if (have_walls) {
			/* obstacle-relative position: anchors x/y to the corridor center */
			est.fuse_walls(read_dist(tof_f), read_dist(tof_b),
				       read_dist(tof_l), read_dist(tof_r), TOF_MAX_RANGE);
		}
		est.get_state(state);

		/* Waypoint setpoint: hold corridor center (y=0), ramp x -> WAYPOINT_X after settle. */
		float sx = 0.0f;
		if (iter > SETTLE_ITERS) {
			float f = (float)(iter - SETTLE_ITERS) / (float)RAMP_ITERS;
			if (f > 1.0f) f = 1.0f;
			sx = WAYPOINT_X * f;
		}
		const float setpoint[NSTATES] = {sx, 0.0f, TARGET_Z, 0,0,0, 0,0,0, 0,0,0};
		for (int i = 0; i < NSTATES; i++) {
			err[i] = state[i] - setpoint[i];
		}
		/* x/y position error is KEPT (observable via the walls) -> waypoint tracking. Without
		 * walls, fall back to velocity-only regulation (zero the position error) as before. */
		if (!have_walls) { err[0] = 0.0f; err[1] = 0.0f; }

		matsetv(work.x.vector[0], err, 1, NSTATES);
		matset(work.y.data, 0.0, work.y.outer, work.y.inner);
		matset(work.g.data, 0.0, work.g.outer, work.g.inner);
		tiny_solve(&solver);
		for (int i = 0; i < NACTIONS; i++) {
			u[i] = work.u.vector[0][i];
		}
		if ((iter % 20) == 0) {
			printk("nav: iter=%d x=%d.%03d y=%d.%03d z=%d.%03d  sx=%d.%03d\n", iter,
			       (int)state[0], (int)(state[0]*1000)%1000,
			       (int)state[1], (int)(state[1]*1000)%1000,
			       (int)state[2], (int)(state[2]*1000)%1000,
			       (int)sx, (int)(sx*1000)%1000);
		}
		send_control(u);
	}
	printk("nav_controller: done (%d iters)\n", CTRL_ITERS);
	return 0;
}
