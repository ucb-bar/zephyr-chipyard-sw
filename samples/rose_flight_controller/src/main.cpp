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
 * Only the board overlay + prj.conf differ; main, the estimator (IStateEstimator), and the
 * controller (IController) are byte-for-byte shared. See docs/ROSE_SENSOR_ABSTRACTION.md.
 *
 * The only target-specific code here is the actuator OUTPUT (a RoSE-bridge TX packet in
 * co-sim vs PWM motors on hardware) — actuator parity is future work; the sensor/estimator/
 * control path is fully shared.
 *
 * Per control step (200 Hz):
 *   1. sample_fetch/channel_get IMU (accel+gyro), optical flow, and (low-rate) ToF height
 *   2. estimator.update(...) -> 12-DoF state; ToF fused only on fresh samples (multi-rate)
 *   3. subtract the hover setpoint (regulate velocity, not the unobservable x/y position)
 *   4. controller.compute(...) -> 4 normalized motor thrusts -> actuator output
 *      (controller = TinyMPC by default, or hierarchical PID via -DROSE_USE_PID=1)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>

#include "estimator.hpp"
#include "controller.hpp"

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
/* Iteration count. Default 5000 suits the RoSE co-sim (~max_sim_time). On real HW the loop now
 * runs ~1 kHz, so 5000 iters is only ~7 s -- override (-DCTRL_ITERS=...) for a longer bench run. */
#ifndef CTRL_ITERS
#define CTRL_ITERS   5000
#endif

/* Sign-correct fixed-point print of a float as "[-]int.frac(3)" without needing %f (portable to
 * builds with printf FP support off). Expands to the sign string + magnitude int + 3-digit frac. */
#include <math.h>
#define FP3(x) ((x) < 0 ? "-" : ""), (int)fabsf(x), ((int)(fabsf(x) * 1000.0f)) % 1000

/* ---- Sensor devices (Zephyr sensor API; bound per board overlay) ---- */
static const struct device *accel_dev = DEVICE_DT_GET(DT_ALIAS(bmi088_accel));
static const struct device *gyro_dev  = DEVICE_DT_GET(DT_ALIAS(bmi088_gyro));

#if HAVE_FLOW
static const struct device *flow_dev = DEVICE_DT_GET(DT_ALIAS(flow));
#endif
#if HAVE_TOF
static const struct device *tof_dev  = DEVICE_DT_GET(DT_ALIAS(tof));
#define TOF_FETCH_PERIOD_MS 33   /* ~30 Hz: matches the VL53L1X ranging budget */
#endif

/* ---- Board sensor-init hook (HW-specific power/enable; no-op on RoSE) --------------------------
 * On the riskybird ESP32 the on-board VL53L1X down-ToF is powered through the ADS7128 I/O
 * expander's GPIO6 (XSHUT). The ADS7128 has no Zephyr gpio-controller driver, so raise the rail
 * here via raw I2C BEFORE the vl53l1x sensor driver inits (SYS_INIT POST_KERNEL/80, ahead of the
 * default sensor init at 90). This block is compiled ONLY when an st,vl53l1x node is present
 * (the real board); on RoSE the ToF is a virtual ucbbar,rose-* device, so this is a no-op. */
#if DT_HAS_COMPAT_STATUS_OKAY(st_vl53l1x)
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor/vl53l1x.h>   /* vl53l1x_reinit(): run the deferred ST init */
#define ADS7128_I2C_ADDR      0x17
#define ADS7128_CMD_REG_WRITE 0x08
#define ADS7128_CMD_REG_READ  0x10
#define ADS7128_PIN_CFG       0x05
#define ADS7128_GPIO_CFG      0x07
#define ADS7128_GPO_DRIVE_CFG 0x09
#define ADS7128_GPO_VALUE     0x0B
#define VL53L1X_XSHUT_CH      6      /* ADS7128 GPIO6 = DOWN VL53L1X XSHUT (riskybird v3) */
/* ADS7128 GPIO1-4 = XSHUT of the 4 SIDE VL53L1X. They power-on at the SAME I2C address (0x29) as
 * the down ToF and still carry their protective film, so if left enabled they contend on 0x29 and
 * dominate it with a 0mm / ~87 Mcps crosstalk return (the down sensor's real reads only occasionally
 * win the bus). The flight controller uses ONLY the down ToF, so hold the sides in reset. See
 * samples/riskybird/sensor_bringup for the full multi-sensor readdressing scheme (down -> 0x30). */
static const uint8_t VL53L1X_SIDE_CH[4] = { 1, 2, 3, 4 };

static int ads7128_rd(const struct device *bus, uint8_t reg, uint8_t *v)
{
	uint8_t tx[2] = { ADS7128_CMD_REG_READ, reg };
	int rc = i2c_write(bus, tx, sizeof(tx), ADS7128_I2C_ADDR);
	return rc ? rc : i2c_read(bus, v, 1, ADS7128_I2C_ADDR);
}
static int ads7128_wr(const struct device *bus, uint8_t reg, uint8_t v)
{
	uint8_t tx[3] = { ADS7128_CMD_REG_WRITE, reg, v };
	return i2c_write(bus, tx, sizeof(tx), ADS7128_I2C_ADDR);
}
static int ads7128_set_bit(const struct device *bus, uint8_t reg, uint8_t ch)
{
	uint8_t v;
	int rc = ads7128_rd(bus, reg, &v);
	if (rc) return rc;
	return ads7128_wr(bus, reg, (uint8_t)(v | (1U << ch)));
}
static int ads7128_clr_bit(const struct device *bus, uint8_t reg, uint8_t ch)
{
	uint8_t v;
	int rc = ads7128_rd(bus, reg, &v);
	if (rc) return rc;
	return ads7128_wr(bus, reg, (uint8_t)(v & ~(1U << ch)));
}

/* Power the VL53L1X via the ADS7128 GPIO6 XSHUT rail. This runs on every boot, but a warm reset
 * (ESP EN pin) does NOT power-cycle the ADS7128, so GPIO6 stays high and the VL53L1X keeps stale
 * state from the previous boot (its address/ranging config), which then makes the ST DataInit fail
 * and the sensor NAK at 0x29. So drive XSHUT LOW first (force the part into reset), then HIGH, to
 * guarantee a clean boot regardless of prior state. */
static int board_sensor_init(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_BUS(DT_INST(0, st_vl53l1x)));
	if (!device_is_ready(bus)) {
		printk("board_sensor_init: I2C bus not ready — ToF stays unpowered\n");
		return 0;   /* non-fatal: the app still runs on the IMU */
	}
	/* First, hold the 4 side ToFs in reset (XSHUT low) so they release the shared 0x29 address and
	 * only the down ToF answers there. Do this BEFORE powering the down sensor. */
	for (int i = 0; i < 4; i++) {
		uint8_t ch = VL53L1X_SIDE_CH[i];
		ads7128_set_bit(bus, ADS7128_PIN_CFG, ch);        /* GPIO mode */
		ads7128_set_bit(bus, ADS7128_GPIO_CFG, ch);       /* output */
		ads7128_set_bit(bus, ADS7128_GPO_DRIVE_CFG, ch);  /* push-pull */
		ads7128_clr_bit(bus, ADS7128_GPO_VALUE, ch);      /* XSHUT LOW = reset (side ToF off) */
	}
	int rc = ads7128_set_bit(bus, ADS7128_PIN_CFG, VL53L1X_XSHUT_CH)        /* GPIO mode */
	       | ads7128_set_bit(bus, ADS7128_GPIO_CFG, VL53L1X_XSHUT_CH)       /* output */
	       | ads7128_set_bit(bus, ADS7128_GPO_DRIVE_CFG, VL53L1X_XSHUT_CH); /* push-pull */
	rc |= ads7128_clr_bit(bus, ADS7128_GPO_VALUE, VL53L1X_XSHUT_CH);        /* XSHUT LOW (reset) */
	k_msleep(5);
	rc |= ads7128_set_bit(bus, ADS7128_GPO_VALUE, VL53L1X_XSHUT_CH);        /* XSHUT HIGH (boot) */
	if (rc) {
		printk("board_sensor_init: ADS7128 VL53L1X power-up failed (rc=%d) — ToF may be absent\n", rc);
		return 0;   /* non-fatal */
	}
	k_msleep(10);   /* let the VL53L1X boot before the sensor driver (prio 90) talks to it */
	printk("board_sensor_init: VL53L1X powered via ADS7128 GPIO%d\n", VL53L1X_XSHUT_CH);
	return 0;
}
SYS_INIT(board_sensor_init, POST_KERNEL, 80);   /* before CONFIG_SENSOR_INIT_PRIORITY (90) */
#endif /* st_vl53l1x present */

/* ---- Emergency cutoff watchdog --------------------------------------------------------------
 * A hard, controller-independent backstop (on top of the MOTOR_MAX_DUTY cap and the actuation
 * timeout): latch motors OFF if the vehicle state exceeds safe limits -- excess tilt, body rate,
 * or velocity -- or a key sensor drops out. Once latched, g_estop stays set until reset, and
 * send_control() forces every motor to 0. Thresholds are build-overridable. */
#include <math.h>
#ifndef SAFE_MAX_TILT_RAD
#define SAFE_MAX_TILT_RAD   1.0f     /* ~57 deg: past any recoverable bench perturbation */
#endif
#ifndef SAFE_MAX_RATE_RADPS
#define SAFE_MAX_RATE_RADPS 10.0f    /* ~573 deg/s: a violent tumble */
#endif
#ifndef SAFE_MAX_VEL_MPS
#define SAFE_MAX_VEL_MPS    2.5f     /* runaway translational velocity */
#endif
#ifndef SAFE_MAX_HEIGHT_M
#define SAFE_MAX_HEIGHT_M   2.0f     /* altitude ceiling: cut before hitting the ceiling */
#endif
#ifndef SAFE_MAX_IMU_MISS
#define SAFE_MAX_IMU_MISS   10       /* consecutive IMU read failures => sensor lost */
#endif
static volatile bool g_estop;        /* latched emergency stop (cleared only by reset) */

/* Returns a short reason if any safety limit is exceeded (state = 12-DoF body state, gyro = body
 * rates rad/s), else NULL. Attitude from the estimate; rate straight from the gyro (no filter lag);
 * velocity from the estimate. */
static const char *safety_violation(const float *state, const float *gyro)
{
	if (fabsf(state[3]) > SAFE_MAX_TILT_RAD || fabsf(state[4]) > SAFE_MAX_TILT_RAD) {
		return "tilt";
	}
	if (fabsf(gyro[0]) > SAFE_MAX_RATE_RADPS || fabsf(gyro[1]) > SAFE_MAX_RATE_RADPS ||
	    fabsf(gyro[2]) > SAFE_MAX_RATE_RADPS) {
		return "rate";
	}
	if (fabsf(state[6]) > SAFE_MAX_VEL_MPS || fabsf(state[7]) > SAFE_MAX_VEL_MPS ||
	    fabsf(state[8]) > SAFE_MAX_VEL_MPS) {
		return "velocity";
	}
	if (state[2] > SAFE_MAX_HEIGHT_M) {   /* z = altitude (up); one-sided ceiling guard */
		return "height";
	}
	return NULL;
}

/* ---- Actuator output: RoSE bridge (co-sim) vs PWM motors (real) ---- */
#define HAVE_ROSE DT_HAS_COMPAT_STATUS_OKAY(ucbbar_roseadapter)
#if HAVE_ROSE
#include <rose/rose.h>
#define ROSE_CMD_CONTROL 0x20u
static const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
static void send_control(const float *u)
{
	/* Emergency cutoff: send minimum thrust (u = -0.583 -> 0 duty) instead of the command. */
	static const float off[NACTIONS] = { -0.583f, -0.583f, -0.583f, -0.583f };
	const float *cmd = g_estop ? off : u;
	rose_tx(rose, ROSE_CMD_CONTROL);
	rose_tx(rose, NACTIONS * sizeof(float));
	for (int i = 0; i < NACTIONS; i++) {
		uint32_t w;
		memcpy(&w, &cmd[i], sizeof(float));
		rose_tx(rose, w);
	}
}
static void motors_startup_pulse(void) { /* no motors on the RoSE target */ }
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
/* SAFETY (early bench bring-up): hard ceiling on motor duty. The controller
 * regulates to a hover setpoint, so its raw command ramps toward hover/takeoff
 * thrust; this scales the full [0,1] duty into [0, MOTOR_MAX_DUTY] so the
 * controller can respond and be observed, but CANNOT produce flight thrust.
 * Raise deliberately only for actual flight testing. */
#ifndef MOTOR_MAX_DUTY
#define MOTOR_MAX_DUTY 0.10f
#endif
static void send_control(const float *u)
{
	/* Emergency cutoff: once latched, force every motor to 0 and ignore the command entirely. */
	if (g_estop) {
		for (int i = 0; i < NACTIONS; i++) {
			pwm_set_pulse_dt(&motors[i], 0);
		}
		return;
	}
#if ROSE_ACTUATE_TIMEOUT_MS > 0
	/* Bench safety: cut all motors ROSE_ACTUATE_TIMEOUT_MS after boot. The
	 * controller/estimator keep running (still logging) — only the actuator stops. */
	if (k_uptime_get() >= (int64_t)ROSE_ACTUATE_TIMEOUT_MS) {
		static bool stopped;
		for (int i = 0; i < NACTIONS; i++) {
			pwm_set_pulse_dt(&motors[i], 0);
		}
		if (!stopped) {
			printk("send_control: actuation timeout (%d ms) reached -- motors OFF\n",
			       (int)ROSE_ACTUATE_TIMEOUT_MS);
			stopped = true;
		}
		return;
	}
#endif
	for (int i = 0; i < NACTIONS; i++) {
		/* normalized thrust u in ~[-0.583, 0.417] -> [0,1] duty */
		float duty = u[i] + 0.583f;
		if (duty < 0.0f) duty = 0.0f;
		if (duty > 1.0f) duty = 1.0f;
		duty *= MOTOR_MAX_DUTY;                            /* scale [0,1] -> [0, cap] */
		if (duty > MOTOR_MAX_DUTY) duty = MOTOR_MAX_DUTY;  /* absolute backstop */
		pwm_set_pulse_dt(&motors[i], (uint32_t)(motors[i].period * duty));
	}
}
/* Optional boot "go" signal: pulse all motors at the safety cap for ROSE_START_PULSE_MS, then
 * stop. Used by the handheld IMU tilt test so the operator knows when the stream has started.
 * Runs once in main() BEFORE the control loop, so it is independent of the actuation timeout. */
static void motors_startup_pulse(void)
{
#if defined(ROSE_START_PULSE_MS) && ROSE_START_PULSE_MS > 0
	for (int i = 0; i < NACTIONS; i++) {
		pwm_set_pulse_dt(&motors[i], (uint32_t)(motors[i].period * MOTOR_MAX_DUTY));
	}
	k_msleep(ROSE_START_PULSE_MS);
	for (int i = 0; i < NACTIONS; i++) {
		pwm_set_pulse_dt(&motors[i], 0);
	}
	k_msleep(400);   /* settle gap so the pulse is distinct from the first tilt motion */
#endif
}
#else
static void send_control(const float *u) { (void)u; /* no actuator bound */ }
static void motors_startup_pulse(void) { /* no motors bound */ }
#endif
#endif

/* State estimator: build-time-selected pluggable filter (default complementary). */
static IStateEstimator &est = active_estimator();

/* Controller: build-time-selected pluggable control law. Default is TinyMPC (constrained MPC);
 * build -DROSE_USE_PID=1 for the hierarchical PID cascade. The solver/gain internals live in the
 * controller_*.cpp behind IController, so main only talks to ctrl.init()/ctrl.compute(). */
static IController &ctrl = active_controller();

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
/* ---- IMU mounting -> drone body frame (forward +x, left +y, up +z) ----------------------------
 * On riskybird v3 the BMI088 (U3) is on the BOARD BOTTOM, rotated 180 deg, so its sensor axes do
 * NOT equal the drone body frame; raw readings must be rotated before the estimator uses them.
 *
 * Derivation (datasheet BST-BMI088-DS001 rev 1.9 + this layout + "pin 1 faces +x,+y"):
 *   - Accel & gyro SHARE one coordinate system (datasheet Fig 12 labels both on one axis triad),
 *     so the SAME rotation applies to both.
 *   - Pin-1 sits at the sensor's (+X,+Y) corner (datasheet Table 15: landscape/pin-top-left reads
 *     +1g on X, so +X and +Y meet at the pin-1 corner).
 *   - Bottom-side mount => the marking/top face points DOWN => sensor +Z = drone -Z. This part is
 *     CERTAIN: the estimator wants body accel_z = +9.81 at rest (az_w = R*a - GRAVITY); with +Z
 *     facing down the part reads -9.81 on +Z, so negating Z yields the required +9.81.
 *   - Pin-1's (+X,+Y) diagonal is aligned to the drone (+x,+y) diagonal; the bottom-side mirror
 *     then resolves the in-plane part to a SWAP (proper rotation, det +1):
 *       body_x = +sensor_y ,  body_y = +sensor_x ,  body_z = -sensor_z
 *
 * VERIFIED ON HARDWARE 2026-08-06 via the ROSE_IMU_DEBUG tilt test (all six channels correct;
 * accel uses the +g-up / specific-force convention, so the axis tilted UP reads POSITIVE):
 *   level at rest        -> accel ~ (0, 0, +9.6)                       [observed +9.6]
 *   tilt NOSE-UP         -> accel_x positive; gyro_y transient NEGATIVE [ax +6.9, gy -0.85]
 *   tilt RIGHT-WING-DOWN -> accel_y positive; gyro_x transient positive [ay +7.0, gx +0.88]
 *   yaw NOSE-LEFT (+y)   -> gyro_z positive                            [gz +1.06]
 * If the board is ever re-spun and a channel changes, fix the SRC index / SIGN below and re-run the
 * tilt test. RoSE's virtual IMU is already body-frame (the HAVE_ROSE branch is identity). */
#if HAVE_ROSE
#define IMU_REMAP(dst, src) do { (dst)[0]=(src)[0]; (dst)[1]=(src)[1]; (dst)[2]=(src)[2]; } while (0)
#else
/* body[k] = SIGN_k * sensor[SRC_k] ; defaults = swap X/Y + negate Z (see derivation above) */
#define IMU_BX_SRC 1
#define IMU_BX_SIGN (+1.0f)
#define IMU_BY_SRC 0
#define IMU_BY_SIGN (+1.0f)
#define IMU_BZ_SRC 2
#define IMU_BZ_SIGN (-1.0f)
#define IMU_REMAP(dst, src) do {                          \
		float _r0 = IMU_BX_SIGN * (src)[IMU_BX_SRC];      \
		float _r1 = IMU_BY_SIGN * (src)[IMU_BY_SRC];      \
		float _r2 = IMU_BZ_SIGN * (src)[IMU_BZ_SRC];      \
		(dst)[0] = _r0; (dst)[1] = _r1; (dst)[2] = _r2;   \
	} while (0)
#endif

/* ---- optional per-phase profiling (build -DROSE_PROFILE=1) -------------------------------------
 * Accumulate cycle counts per sub-phase; main() prints avg microseconds periodically. Off by
 * default (zero overhead) -- purely a bring-up instrument to see what dominates the loop period. */
#if defined(ROSE_PROFILE) && ROSE_PROFILE
static uint32_t pf_imu_fetch, pf_imu_get, pf_tof;   /* accumulated cycles in the current window */
static uint32_t pf_tof_n;                           /* # of real ToF fetches in the window */
static uint32_t pf_est, pf_ctrl, pf_send, pf_iters; /* per-phase cycles + iteration count */
#define PF_NOW()          k_cycle_get_32()
#define PF_ACC(dst, t0)   do { (dst) += k_cycle_get_32() - (t0); } while (0)
#else
#define PF_NOW()          0u
#define PF_ACC(dst, t0)   do { (void)(t0); } while (0)
#endif

/* ---- Down-ToF decoupling (real HW) --------------------------------------------------------------
 * The Zephyr st,vl53l1x driver is single-shot + BLOCKING: channel_get waits a full ranging budget
 * (~66 ms measured) for a fresh sample, which stalled the whole control loop to ~15 Hz (profiling:
 * 98.7% of the loop was this one call). The VL53L1X itself can range continuously up to 100 Hz with
 * a non-blocking data-ready poll (ST AN5263), but this driver exposes neither continuous mode nor a
 * timing-budget knob, and the INT/GPIO1 data-ready pin is not wired on riskybird -- so a non-blocking
 * read would need driver surgery. Instead, run the blocking fetch on its OWN thread at the sensor's
 * natural rate and let the control loop read the latest cached height non-blocking.
 * (RoSE's virtual ToF does not block, so there the fetch stays inline; RoSE lockstep + extra threads
 * is also the known-deadlock combo we avoid.) */
#if HAVE_TOF && !HAVE_ROSE
#define TOF_THREADED 1
K_MUTEX_DEFINE(tof_mtx);
static float g_tof_h = START_Z;
static bool  g_tof_valid;
K_THREAD_STACK_DEFINE(tof_stack, 4096);
static struct k_thread tof_thread_data;
static void tof_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (;;) {
		if (sensor_sample_fetch(tof_dev) == 0) {   /* blocks ~1 ranging budget on this thread */
			struct sensor_value h;
			sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &h);
			float hv = (float)sensor_value_to_double(&h);
			k_mutex_lock(&tof_mtx, K_FOREVER);
			g_tof_h = hv; g_tof_valid = true;
			k_mutex_unlock(&tof_mtx);
		} else {
			k_msleep(5);   /* back off on error so a failing ToF can't spin the I2C bus */
		}
	}
}
#else
#define TOF_THREADED 0
#endif

static bool read_sensor_frame(struct sensor_frame *f)
{
	uint32_t _pf = PF_NOW();
	int rc_a = sensor_sample_fetch(accel_dev);
	int rc_g = sensor_sample_fetch(gyro_dev);
#if HAVE_FLOW
	sensor_sample_fetch(flow_dev);
#endif
	PF_ACC(pf_imu_fetch, _pf);
	f->tof_valid = false;
	if (rc_a < 0 || rc_g < 0) {
		return false;
	}
	_pf = PF_NOW();
	struct sensor_value av[3], gv[3];
	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, av);
	sensor_channel_get(gyro_dev,  SENSOR_CHAN_GYRO_XYZ,  gv);
	float araw[3], graw[3];
	for (int i = 0; i < 3; i++) {
		araw[i] = (float)sensor_value_to_double(&av[i]);
		graw[i] = (float)sensor_value_to_double(&gv[i]);
	}
	IMU_REMAP(f->accel, araw);   /* sensor -> drone body frame (no-op on RoSE) */
	IMU_REMAP(f->gyro,  graw);
	PF_ACC(pf_imu_get, _pf);
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
#if TOF_THREADED
	/* Real HW: read the latest cached height produced by the ToF thread (non-blocking). The
	 * blocking VL53L1X fetch happens on that thread, so it never stalls the control loop. */
	uint32_t _pt = PF_NOW();
	k_mutex_lock(&tof_mtx, K_FOREVER);
	f->tof_valid = g_tof_valid;
	if (g_tof_valid) {
		f->height = g_tof_h;
	}
	k_mutex_unlock(&tof_mtx);
	PF_ACC(pf_tof, _pt);
#if defined(ROSE_PROFILE) && ROSE_PROFILE
	pf_tof_n++;
#endif
#else
	/* RoSE (non-blocking virtual ToF): keep the inline rate-limited fetch + zero-order hold. */
	static int64_t tof_next_ms = 0;
	static float   tof_last_h  = START_Z;
	static bool    tof_have    = false;
	int64_t now_ms = k_uptime_get();
	if (now_ms >= tof_next_ms) {
		tof_next_ms = now_ms + TOF_FETCH_PERIOD_MS;
		if (sensor_sample_fetch(tof_dev) == 0) {
			struct sensor_value h;
			sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &h);
			tof_last_h = (float)sensor_value_to_double(&h);
			tof_have   = true;
		}
	}
	f->tof_valid = tof_have;
	if (tof_have) {
		f->height = tof_last_h;
	}
#endif /* TOF_THREADED */
#endif /* HAVE_TOF */
	return true;
}

/* Control block: run the active controller (TinyMPC or PID) from a 12-DoF state -> 4 motor
 * thrusts. Setpoint/state error handling and the solve live behind IController now. dt is the
 * REAL measured loop period (s) -- pass the same value used for est.update so time bases match. */
static void solve_control(const float *state, float *u, float dt)
{
	ctrl.compute(state, g_setpoint, u, dt);
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

			solve_control(st, u, CTRL_DT);   /* threaded path (experimental): nominal dt */

			k_mutex_lock(&mtx_ctrl, K_FOREVER);
			for (int i = 0; i < NACTIONS; i++) g_ctrl[i] = u[i];
			k_mutex_unlock(&mtx_ctrl);

			if ((tick % (10 * ROSE_CTRL_DIV)) == 0) {
				printk("flight_controller: t=%u z=%s%d.%03d u0=%s%d.%03d\n", tick,
				       FP3(st[2]), FP3(u[0]));
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
	ctrl.init();
	est.init(0.0f, 0.0f, START_Z);

	motors_startup_pulse();   /* optional boot "go" signal (ROSE_START_PULSE_MS); no-op if unset */

#if DT_HAS_COMPAT_STATUS_OKAY(st_vl53l1x)
	/* The st,vl53l1x driver defers the full ST boot (DataInit/StaticInit); nothing runs it unless
	 * the app asks. Trigger it once here (XSHUT rail already raised by board_sensor_init) so the
	 * down-ToF actually ranges -- without this every sample_fetch hits an uninitialized device and
	 * floods "Failed to write". No-op on RoSE (virtual ToF is a different compat). */
	{
		int rc = vl53l1x_reinit(tof_dev);
		printk("flight_controller: vl53l1x_reinit rc=%d (%s)\n", rc,
		       rc == 0 ? "down-ToF ranging" : "ToF init failed -- altitude unaided");
	}
#if defined(ROSE_TOF_CAL_MM) && ROSE_TOF_CAL_MM > 0
	/* One-shot ToF calibration: place a target at ROSE_TOF_CAL_MM (mm) in a dark, low-reflection
	 * setup BEFORE reset. Runs offset + crosstalk cal (the latter also enables xtalk compensation).
	 * Results persist until power cycle. Build with -DROSE_TOF_CAL_MM=<distance> to use. */
	{
		int rc = vl53l1x_calibrate(tof_dev, ROSE_TOF_CAL_MM, ROSE_TOF_CAL_MM);
		printk("flight_controller: vl53l1x_calibrate(%d mm) rc=%d (%s)\n",
		       (int)ROSE_TOF_CAL_MM, rc, rc == 0 ? "offset+xtalk done" : "cal FAILED");
	}
#endif
#endif

#if TOF_THREADED
	/* Start the down-ToF fetcher AFTER vl53l1x_reinit so it never fetches an uninitialized device.
	 * Priority below the main control loop (higher number) -- it mostly blocks on I2C anyway, and
	 * the control loop must always preempt it. */
	k_thread_create(&tof_thread_data, tof_stack, K_THREAD_STACK_SIZEOF(tof_stack),
			tof_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&tof_thread_data, "tof");
	printk("flight_controller: down-ToF on dedicated thread (control loop reads cached height)\n");
#endif

#if ROSE_THREADED
	printk("flight_controller: estimator=%s + controller=%s (%s), THREADED blocks "
	       "(estimate@grant, control every %d) \n",
	       est.name(), ctrl.name(), HAVE_ROSE ? "RoSE co-sim" : "real target", ROSE_CTRL_DIV);
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
	printk("flight_controller: estimator=%s + controller=%s ready (%s), single-loop\n",
	       est.name(), ctrl.name(), HAVE_ROSE ? "RoSE co-sim" : "real target");
	struct sensor_frame f;
	float state[NSTATES], u[NACTIONS];
	/* Use the REAL measured loop period, not the nominal CTRL_DT. On this soft-float target the
	 * loop runs ~15 Hz (dt ~67 ms), not the 200 Hz the design assumes; feeding the fixed 5 ms made
	 * the estimator integrate ~13x too slow, so real tilts barely registered. Measure dt each iter. */
	int64_t t_prev = k_uptime_get();
	int imu_miss = 0;
	for (int iter = 0; iter < CTRL_ITERS; iter++) {
		if (!read_sensor_frame(&f)) {
			printk("flight_controller: IMU fetch error\n");
			if (++imu_miss >= SAFE_MAX_IMU_MISS && !g_estop) {
				g_estop = true;
				printk("flight_controller: EMERGENCY CUTOFF -- IMU lost (%d misses); "
				       "motors OFF (reset to clear)\n", imu_miss);
			}
			if (g_estop) {
				send_control(u);   /* g_estop forces motors to 0 (u not read) */
			}
			continue;
		}
		imu_miss = 0;
		int64_t t_now = k_uptime_get();
		float dt = (float)(t_now - t_prev) * 1e-3f;   /* real loop period (s) */
		if (dt <= 0.0f || dt > 0.5f) dt = CTRL_DT;     /* first-iter / stall guard */
		t_prev = t_now;
		uint32_t _pe = PF_NOW();
		est.update(f.accel, f.gyro, f.flow, f.flow_valid, f.height, f.tof_valid, dt);
		est.get_state(state);
		PF_ACC(pf_est, _pe);
		/* Emergency watchdog: latch a kill if attitude/rate/velocity exceed safe limits. Checked
		 * every iteration before actuation; send_control() enforces the cut. */
		if (!g_estop) {
			const char *why = safety_violation(state, f.gyro);
			if (why != NULL) {
				g_estop = true;
				printk("flight_controller: EMERGENCY CUTOFF -- %s limit exceeded; "
				       "motors OFF (reset to clear)\n", why);
			}
		}
		uint32_t _pc = PF_NOW();
		solve_control(state, u, dt);
		PF_ACC(pf_ctrl, _pc);
		uint32_t _ps = PF_NOW();
		send_control(u);
		PF_ACC(pf_send, _ps);
#if defined(ROSE_PROFILE) && ROSE_PROFILE
		if (++pf_iters >= 30) {
			/* read_sensor_frame was already timed into pf_imu_fetch/get/tof (read total = their
			 * sum). Print avg microseconds per phase over the window, then reset. */
			uint32_t rd = pf_imu_fetch + pf_imu_get + pf_tof;
			printk("PROFILE/%u: read=%uus [imu_fetch=%uus imu_get=%uus tof=%uus x%u] "
			       "est=%uus ctrl=%uus send=%uus\n", pf_iters,
			       k_cyc_to_us_floor32(rd / pf_iters),
			       k_cyc_to_us_floor32(pf_imu_fetch / pf_iters),
			       k_cyc_to_us_floor32(pf_imu_get / pf_iters),
			       pf_tof_n ? k_cyc_to_us_floor32(pf_tof / pf_tof_n) : 0u, pf_tof_n,
			       k_cyc_to_us_floor32(pf_est / pf_iters),
			       k_cyc_to_us_floor32(pf_ctrl / pf_iters),
			       k_cyc_to_us_floor32(pf_send / pf_iters));
			pf_imu_fetch = pf_imu_get = pf_tof = pf_tof_n = 0;
			pf_est = pf_ctrl = pf_send = pf_iters = 0;
		}
#endif
		if ((iter % 10) == 0) {
#if defined(ROSE_IMU_DEBUG) && ROSE_IMU_DEBUG
			/* Body-frame IMU dump for the axis/sign tilt test (see IMU_REMAP note). */
			printk("flight_controller: iter=%d a=[%s%d.%03d %s%d.%03d %s%d.%03d] "
			       "g=[%s%d.%03d %s%d.%03d %s%d.%03d]\n", iter,
			       FP3(f.accel[0]), FP3(f.accel[1]), FP3(f.accel[2]),
			       FP3(f.gyro[0]),  FP3(f.gyro[1]),  FP3(f.gyro[2]));
#else
			/* Attitude + ALL 4 motor commands, so the restoring differential is visible: a
			 * pitch tilt should split the fore/aft motor pair, a roll tilt the left/right pair. */
			printk("flight_controller: it=%d dt=%dms roll=%s%d.%03d pitch=%s%d.%03d yaw=%s%d.%03d "
			       "z=%s%d.%03d u=[%s%d.%03d %s%d.%03d %s%d.%03d %s%d.%03d]\n",
			       iter, (int)(dt * 1000.0f + 0.5f),
			       FP3(state[3]), FP3(state[4]), FP3(state[5]), FP3(state[2]),
			       FP3(u[0]), FP3(u[1]), FP3(u[2]), FP3(u[3]));
#endif
		}
		/* send_control() is issued (and timed) above, before this telemetry print. */
	}
	printk("flight_controller: control loop done (%d iters)\n", CTRL_ITERS);
	return 0;
#endif
}
