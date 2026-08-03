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

#define NSTATES   12
#define NACTIONS  4

/* Optional sensors: present iff the board overlay declares the alias. On the RoSE target
 * both are virtual ucbbar,rose-* devices; on real hardware they bind to PMW3901 / VL53L1x
 * (flow may be absent on a given board -> the app compiles and runs without it). */
#define HAVE_FLOW DT_NODE_EXISTS(DT_ALIAS(flow))
#define HAVE_TOF  DT_NODE_EXISTS(DT_ALIAS(tof))
#if HAVE_FLOW
#include <rose/rose_sensor.h>   /* RoSE private optical-flow channels */
#endif

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

/* =====================================================================================
 * Modular task blocks. The controller is split into three cooperating blocks so IO can be
 * decoupled from compute and the estimator/controller can run at DIFFERENT rates:
 *
 *   [sensor IO] --sem_sample--> [estimator] --sem_state--> [control] --g_control--> [actuator]
 *
 * The blocks are Zephyr threads sharing latest-value buffers (mutex-protected) and handing
 * off via semaphores. The pipeline is IO-paced: the sensor exchange gates one iteration.
 * ROSE_CTRL_DIV runs TinyMPC once every N estimator ticks -> control rate = estimation rate
 * / DIV (e.g. estimate @200 Hz, control @50 Hz to match the TinyMPC design rate). On real
 * hardware the IO block's transport (DMA/IRQ) overlaps with compute; in the single-core
 * lockstep co-sim the blocks serialize within each grant but the structure + rates are real.
 * Set ROSE_THREADED=0 for the original single-loop build (kept for A/B).
 * ===================================================================================== */
/* NOTE: default 0 (single-loop). The threaded blocks are fully implemented and validated to
 * hover (DIV=1 matches the single loop; DIV=4 runs control @50 Hz / estimate @200 Hz), but the
 * RoSE *lockstep* co-sim intermittently deadlocks after ~235 steps: the guest waits for the
 * synchronizer's next grant while the synchronizer waits for the guest -- a subtle timing
 * desync between preemptive threading and the deterministic per-grant protocol (the guest is
 * NOT crashed; the hover is perfect until it stalls). Real hardware (no lockstep) is unaffected.
 * Build -DROSE_THREADED=1 to use/continue-debugging the threaded architecture. */
#ifndef ROSE_THREADED
#define ROSE_THREADED 0
#endif
#ifndef ROSE_CTRL_DIV
#define ROSE_CTRL_DIV 1        /* control runs every Nth estimator tick (1 = same rate) */
#endif

static const float g_setpoint[NSTATES] = {0.0f, 0.0f, TARGET_Z, 0,0,0, 0,0,0, 0,0,0};

struct sensor_frame {
	float accel[3], gyro[3], flow[2], height;
	bool flow_valid, tof_valid;
};

/* Sensor IO block: batched fetch (TX) then collect (blocking RX) of the whole sensor set. */
static bool read_sensor_frame(struct sensor_frame *f)
{
	int rc_a = sensor_sample_fetch(accel_dev);
	int rc_g = sensor_sample_fetch(gyro_dev);
#if HAVE_FLOW
	sensor_sample_fetch(flow_dev);
#endif
	f->tof_valid = false;
#if HAVE_TOF
	f->tof_valid = (sensor_sample_fetch(tof_dev) == 0);
#endif
	if (rc_a < 0 || rc_g < 0) {
		return false;
	}
	struct sensor_value av[3], gv[3];
	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, av);
	sensor_channel_get(gyro_dev,  SENSOR_CHAN_GYRO_XYZ,  gv);
	for (int i = 0; i < 3; i++) {
		f->accel[i] = (float)sensor_value_to_double(&av[i]);
		f->gyro[i]  = (float)sensor_value_to_double(&gv[i]);
	}
	f->flow[0] = f->flow[1] = 0.0f;
	f->flow_valid = true;
#if HAVE_FLOW
	{
		struct sensor_value vx, vy;
		sensor_channel_get(flow_dev, (enum sensor_channel)ROSE_SENSOR_CHAN_FLOW_VX, &vx);
		sensor_channel_get(flow_dev, (enum sensor_channel)ROSE_SENSOR_CHAN_FLOW_VY, &vy);
		f->flow[0] = (float)sensor_value_to_double(&vx);
		f->flow[1] = (float)sensor_value_to_double(&vy);
		if (f->flow[0] != f->flow[0] || f->flow[1] != f->flow[1]) {   /* NaN = dropout sentinel */
			f->flow_valid = false;
			f->flow[0] = f->flow[1] = 0.0f;
		}
	}
#endif
	f->height = START_Z;
#if HAVE_TOF
	if (f->tof_valid) {
		struct sensor_value h;
		sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &h);
		f->height = (float)sensor_value_to_double(&h);
	}
#endif
	return true;
}

/* Control block: TinyMPC solve from a 12-DoF state -> 4 motor thrusts. */
static void solve_control(const float *state, float *u)
{
	float err[NSTATES];
	for (int i = 0; i < NSTATES; i++) {
		err[i] = state[i] - g_setpoint[i];
	}
	err[0] = 0.0f; err[1] = 0.0f;   /* x/y position unobservable from flow -> regulate velocity */
	matsetv(work.x.vector[0], err, 1, NSTATES);
	matset(work.y.data, 0.0, work.y.outer, work.y.inner);
	matset(work.g.data, 0.0, work.g.outer, work.g.inner);
	tiny_solve(&solver);
	for (int i = 0; i < NACTIONS; i++) {
		u[i] = work.u.vector[0][i];
	}
}

#if ROSE_THREADED
/* ---- inter-block state: latest-value buffers + handoff semaphores ---- */
K_MUTEX_DEFINE(mtx_frame);
K_MUTEX_DEFINE(mtx_state);
K_MUTEX_DEFINE(mtx_ctrl);
K_SEM_DEFINE(sem_sample, 0, 1);   /* IO -> estimator: a new sensor frame is ready */
K_SEM_DEFINE(sem_state, 0, 1);    /* estimator -> control: a new state estimate is ready */
K_SEM_DEFINE(sem_done, 0, 1);     /* control -> IO: this grant's estimate+control finished */
static struct sensor_frame g_frame;
static float g_state[NSTATES];
static float g_ctrl[NACTIONS] = {0};

/* Priorities: control (lowest number) preempts estimator preempts IO, so a fresh frame flows
 * frame -> state -> control within one grant, then the IO block sends the fresh command. */
#define PRIO_IO   7
#define PRIO_EST  5
#define PRIO_CTRL 3
#define PRIO_KEEPALIVE 14         /* lowest app priority (just above the idle thread) */
K_THREAD_STACK_DEFINE(io_stack,   16384);
K_THREAD_STACK_DEFINE(est_stack,  65536);
K_THREAD_STACK_DEFINE(ctrl_stack, 327680);   /* TinyMPC solve working set (was the main stack) */
K_THREAD_STACK_DEFINE(keepalive_stack, 2048);
static struct k_thread io_t, est_t, ctrl_t, keepalive_t;

/* Keepalive: under the RoSE lockstep, the guest's virtual clock (mtime) only advances while it
 * executes; if every app thread blocks for even an instant the idle thread runs WFI, which
 * HALTS mtime -> the timer interrupt that would wake it can't fire (the sync gates mtime to the
 * grant budget) -> guest + sync deadlock. This lowest-priority thread never blocks, so the CPU
 * always has something to run instead of idling; any ready IO/estimator/control thread still
 * preempts it. (On real hardware you would drop this and let the core sleep.) */
static volatile uint32_t keepalive_spin;
static void keepalive_block(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	/* True busy-spin (NOT k_yield -- Zephyr still idles the core when the yielding thread is the
	 * only ready one, which WFI-halts mtime). This never yields, so the core never idles; the
	 * higher-priority IO/estimator/control threads still preempt it the instant they are ready. */
	for (;;) {
		keepalive_spin++;
	}
}

static void io_block(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (int iter = 0; iter < CTRL_ITERS; iter++) {
		struct sensor_frame f;
		if (!read_sensor_frame(&f)) {
			continue;
		}
		k_mutex_lock(&mtx_frame, K_FOREVER);
		g_frame = f;
		k_mutex_unlock(&mtx_frame);
		k_sem_give(&sem_sample);        /* trigger estimator -> control for this frame */
		k_sem_take(&sem_done, K_FOREVER);  /* BLOCK (yield) until compute finishes -> the per-
		                                    * grant sequence stays deterministic (no preemption
		                                    * mid-IO), which the lockstep protocol requires. */

		float u[NACTIONS];
		k_mutex_lock(&mtx_ctrl, K_FOREVER);
		for (int i = 0; i < NACTIONS; i++) u[i] = g_ctrl[i];
		k_mutex_unlock(&mtx_ctrl);
		send_control(u);                /* fresh command (from this frame), applied next step */
	}
	printk("flight_controller: IO block done (%d iters)\n", CTRL_ITERS);
}

static void est_block(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sem_take(&sem_sample, K_FOREVER);
		struct sensor_frame f;
		k_mutex_lock(&mtx_frame, K_FOREVER);
		f = g_frame;
		k_mutex_unlock(&mtx_frame);

		est.update(f.accel, f.gyro, f.flow, f.flow_valid, f.height, f.tof_valid, CTRL_DT);
		float st[NSTATES];
		est.get_state(st);

		k_mutex_lock(&mtx_state, K_FOREVER);
		for (int i = 0; i < NSTATES; i++) g_state[i] = st[i];
		k_mutex_unlock(&mtx_state);
		k_sem_give(&sem_state);
	}
}

static void ctrl_block(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	uint32_t tick = 0;
	while (1) {
		k_sem_take(&sem_state, K_FOREVER);
		if ((++tick % ROSE_CTRL_DIV) == 0) {   /* sub-rate: solve only every DIV-th tick */
			float st[NSTATES], u[NACTIONS];
			k_mutex_lock(&mtx_state, K_FOREVER);
			for (int i = 0; i < NSTATES; i++) st[i] = g_state[i];
			k_mutex_unlock(&mtx_state);

			solve_control(st, u);

			k_mutex_lock(&mtx_ctrl, K_FOREVER);
			for (int i = 0; i < NACTIONS; i++) g_ctrl[i] = u[i];
			k_mutex_unlock(&mtx_ctrl);

			if ((tick % (10 * ROSE_CTRL_DIV)) == 0) {
				printk("flight_controller: t=%u z=%d.%03d u0=%d.%03d\n", tick,
				       (int)st[2], (int)(st[2]*1000)%1000, (int)u[0], (int)(u[0]*1000)%1000);
			}
		}
		/* Always signal IO -- on skip grants g_ctrl is held (sub-rate command). */
		k_sem_give(&sem_done);
	}
}
#endif /* ROSE_THREADED */

int main(void)
{
	if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev)) {
		printk("flight_controller: FAIL (IMU not ready)\n");
		return -1;
	}
	enable_vector_operations();
	mpc_init();
	est.init(0.0f, 0.0f, START_Z);

#if ROSE_THREADED
	printk("flight_controller: estimator=%s + TinyMPC (%s), THREADED blocks "
	       "(estimate@grant, control every %d) \n",
	       est.name(), HAVE_ROSE ? "RoSE co-sim" : "real target", ROSE_CTRL_DIV);
	/* Start the blocks after init so nothing runs against an uninitialized estimator/MPC. */
	k_thread_create(&ctrl_t, ctrl_stack, K_THREAD_STACK_SIZEOF(ctrl_stack),
			ctrl_block, NULL, NULL, NULL, PRIO_CTRL, 0, K_NO_WAIT);
	k_thread_create(&est_t, est_stack, K_THREAD_STACK_SIZEOF(est_stack),
			est_block, NULL, NULL, NULL, PRIO_EST, 0, K_NO_WAIT);
	k_thread_create(&keepalive_t, keepalive_stack, K_THREAD_STACK_SIZEOF(keepalive_stack),
			keepalive_block, NULL, NULL, NULL, PRIO_KEEPALIVE, 0, K_NO_WAIT);
	k_thread_create(&io_t, io_stack, K_THREAD_STACK_SIZEOF(io_stack),
			io_block, NULL, NULL, NULL, PRIO_IO, 0, K_NO_WAIT);
	k_thread_join(&io_t, K_FOREVER);   /* run until the IO block finishes its iterations */
	k_thread_abort(&keepalive_t);
	printk("flight_controller: control loop done (%d iters)\n", CTRL_ITERS);
	return 0;
#else
	printk("flight_controller: estimator=%s + TinyMPC ready (%s), single-loop\n",
	       est.name(), HAVE_ROSE ? "RoSE co-sim" : "real target");
	struct sensor_frame f;
	float state[NSTATES], u[NACTIONS];
	for (int iter = 0; iter < CTRL_ITERS; iter++) {
		if (!read_sensor_frame(&f)) {
			printk("flight_controller: IMU fetch error\n");
			continue;
		}
		est.update(f.accel, f.gyro, f.flow, f.flow_valid, f.height, f.tof_valid, CTRL_DT);
		est.get_state(state);
		solve_control(state, u);
		if ((iter % 10) == 0) {
			printk("flight_controller: iter=%d z_est=%d.%03d u0=%d.%03d\n", iter,
			       (int)state[2], (int)(state[2]*1000)%1000, (int)u[0], (int)(u[0]*1000)%1000);
		}
		send_control(u);
	}
	printk("flight_controller: control loop done (%d iters)\n", CTRL_ITERS);
	return 0;
#endif
}
