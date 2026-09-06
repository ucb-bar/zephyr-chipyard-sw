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
#include "flightlog.h"
#include "side_tof.h"           /* side-ToF wall "bumper" (ROSE_BUMPER); no-op if disabled */
#include "flow.h"               /* PMW3901 optical flow (ROSE_FLOW); no-op if disabled */
#if defined(CONFIG_WIFI)
#include "telem_wifi.h"         /* WiFi SoftAP UDP telemetry downlink (opt-in telem.conf; see plan) */
#endif

/* Repulsion bridge into the PID controller (defined in controller_pid.cpp). Feeding walls only has
 * an effect when the PID controller is active + built with ROSE_BUMPER; harmless otherwise. */
extern "C" void pid_set_walls(int16_t front_mm, int16_t back_mm,
			      int16_t left_mm, int16_t right_mm, bool valid);

#define NSTATES   12
#define NACTIONS  4

/* Optional sensors: present iff the board overlay declares the alias. On the RoSE target
 * both are virtual ucbbar,rose-* devices; on real hardware they bind to PMW3901 / VL53L1x
 * (flow may be absent on a given board -> the app compiles and runs without it). */
#define HAVE_FLOW DT_NODE_EXISTS(DT_ALIAS(flow))
#define HAVE_TOF  DT_NODE_EXISTS(DT_ALIAS(tof))
#define HAVE_BARO DT_NODE_EXISTS(DT_ALIAS(baro))   /* BMP388 (bosch,bmp388) -> `baro` alias */
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
 * runs ~1 kHz, so 5000 iters is only ~7 s -- override (-DCTRL_ITERS=...) for a longer bench run.
 * CTRL_ITERS=0 -> run FOREVER (no cap): the control/telemetry loop never exits, so a tethered
 * debug session (e.g. state_viz) keeps receiving data instead of the link "dropping" when the
 * iteration cap is hit. Flight builds keep a finite cap so the loop ends + flushes the log. */
#ifndef CTRL_ITERS
#define CTRL_ITERS   5000
#endif
#if CTRL_ITERS <= 0
#define CTRL_RUN_FOREVER 1
#else
#define CTRL_RUN_FOREVER 0
#endif
/* In-loop console telemetry (it=/flow:/walls). ON for tethered bring-up (state_viz). MUST be OFF for
 * untethered flight: printk goes to the USB-Serial/JTAG console, which BLOCKS on a full TX buffer when
 * no host is draining it -- measured 30-60 ms stalls, ~22% of a flight, freezing the control loop and
 * corrupting the estimator dt. Untethered we rely on the (non-blocking, background-thread) flightlog. */
#ifndef ROSE_TELEM
#define ROSE_TELEM 1
#endif

/* ---- Battery voltage sense + thrust sag-compensation + low-voltage protection (1S LiPo) -------
 * riskybird v3 senses the pack through a 200k/100k divider (Vsense = Vbat * 100k/(200k+100k) =
 * Vbat/3) into the ADS7128 AIN5/GPIO5 channel (U1 pin 4, ref = AVDD/+3V3). The ADS7128 is already
 * driven for the VL53L1X XSHUT rails (see the st_vl53l1x block below); this reads AIN5 as an ADC.
 * OFF by default -- enable with -DROSE_BATT_SENSE=1. When on:
 *   (1) g_vbat is polled every BATT_CHECK_DIV control iters and smoothed (EMA),
 *   (2) send_control() scales motor duty by BATT_NOMINAL_V/g_vbat (clamped) to hold thrust as the
 *       pack sags, (3) arming is blocked below BATT_ARM_MIN_V, (4) in flight a valid reading below
 *       BATT_CUTOFF_V trips the emergency watchdog (safety_violation() "battery"). */
#ifndef ROSE_BATT_SENSE
#define ROSE_BATT_SENSE 0
#endif
#ifndef BATT_NOMINAL_V
#define BATT_NOMINAL_V 3.8f      /* thrust-scaling reference; sag comp targets this pack voltage */
#endif
#ifndef BATT_ARM_MIN_V
#define BATT_ARM_MIN_V 3.4f      /* refuse to ARM below this (pre-flight gate) */
#endif
#ifndef BATT_CUTOFF_V
#define BATT_CUTOFF_V  3.2f      /* in-flight: trip the emergency estop below this */
#endif
#ifndef BATT_DIVIDER
#define BATT_DIVIDER   3.0f      /* Vbat = Vadc * (R30+R31)/R31 = Vadc * (200k+100k)/100k = Vadc*3 */
#endif
#ifndef BATT_VREF_V
#define BATT_VREF_V    3.3f      /* ADS7128 reference = AVDD (+3V3); 12-bit full-scale = 4096 counts */
#endif
#ifndef BATT_CHECK_DIV
#define BATT_CHECK_DIV 200       /* poll the ADC every N control iters (~1 kHz loop -> ~5 Hz) */
#endif
#ifndef BATT_SMOOTH_ALPHA
#define BATT_SMOOTH_ALPHA 0.20f  /* EMA weight on each new sample (higher = less smoothing) */
#endif
#ifndef BATT_SCALE_MAX
#define BATT_SCALE_MAX 1.30f     /* max sag-comp boost -- never over-drive the motors */
#endif
/* Smoothed pack voltage (V); 0 until the first valid ADC read. Written by battery_poll() (real
 * board only), read by the helpers below. Treated as "invalid / not yet read" outside [1.0, 5.0] V. */
static volatile float g_vbat = 0.0f;

/* Motor-duty multiplier that compensates for pack sag. Returns 1.0 (no-op) when battery sense is
 * disabled or g_vbat looks invalid (0 / absurd); otherwise clamp(BATT_NOMINAL_V/g_vbat, [1, max]).
 * Never REDUCES thrust (a fresh pack above nominal clamps to 1.0). */
static inline float batt_thrust_scale(void)
{
#if ROSE_BATT_SENSE
	float v = g_vbat;
	if (v < 1.0f || v > 5.0f) { return 1.0f; }   /* invalid / not-yet-read -> no scaling */
	float s = BATT_NOMINAL_V / v;
	if (s < 1.0f) { s = 1.0f; }
	if (s > BATT_SCALE_MAX) { s = BATT_SCALE_MAX; }
	return s;
#else
	return 1.0f;
#endif
}
/* Pre-arm battery gate: true = OK to arm. Disabled or a not-yet-read/absurd g_vbat -> permit (so a
 * broken sensor or bring-up race never hard-locks the drone); only a VALID low reading blocks. */
static inline bool batt_ok_to_arm(void)
{
#if ROSE_BATT_SENSE
	float v = g_vbat;
	if (v < 1.0f || v > 5.0f) { return true; }   /* not yet read -> don't block bring-up */
	return v >= BATT_ARM_MIN_V;
#else
	return true;
#endif
}

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

/* Barometer altitude fusion (BMP388 -> smooth relative altitude, fused with the down-ToF in the
 * estimator so altitude survives ToF dropouts under tilt / obstacle step-jumps). OFF by default:
 * the BMP388 is NOT yet on the DT overlay (no `baro` alias), so with ROSE_BARO=1 the read stubs to
 * baro_valid=false and a #warning fires. See the enable notes in the report / README. */
#ifndef ROSE_BARO
#define ROSE_BARO 0
#endif
#if HAVE_BARO
static const struct device *baro_dev = DEVICE_DT_GET(DT_ALIAS(baro));
#define BARO_FETCH_PERIOD_MS 20   /* ~50 Hz: a modest BMP388 ODR, ample for altitude */
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
#define STATUS_LED_CH         7      /* ADS7128 GPIO7 = D8 status LED (active-low: GPIO7 LOW = LED on) */
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

/* Status LED (D8) I2C bus handle. Set by board_sensor_init() ONLY after the GPIO7 config ACKs,
 * so it stays NULL on targets without the ADS7128 (RoSE co-sim) and the LED thread no-ops there. */
static const struct device *g_led_bus;

#if ROSE_BATT_SENSE
/* ---- Battery ADC on ADS7128 AIN5/GPIO5 (U1 pin 4; riskybird v3 200k/100k divider) ------------
 * AIN5 is left as an ANALOG input (PIN_CFG bit 5 = 0, the power-on default -- the XSHUT setup only
 * flips channels 1-4 and 6 to GPIO, never 5). Manual mode (SEQUENCE_CFG default): select the channel
 * once via CHANNEL_SEL, then each bare 2-byte I2C read returns that channel's latest conversion
 * (12-bit, left-justified in the 16-bit frame). One short transaction -> called at a low rate. */
#define BATT_ADC_CH         5
#define ADS7128_CHANNEL_SEL 0x11
static const struct device *g_batt_bus;   /* cached by battery_sense_init() (== the ToF I2C bus) */

static void battery_sense_init(const struct device *bus)
{
	g_batt_bus = bus;
	ads7128_clr_bit(bus, ADS7128_PIN_CFG, BATT_ADC_CH);   /* ensure AIN5 is an analog input */
	ads7128_wr(bus, ADS7128_CHANNEL_SEL, BATT_ADC_CH);    /* manual-mode conversion channel = AIN5 */
}

/* One non-blocking-ish ADC read (single 2-byte I2C transfer) + EMA into g_vbat. Silently skips on
 * a bus error or an implausible result (keeps the previous g_vbat). */
static void battery_poll(void)
{
	const struct device *bus = g_batt_bus;
	if (!bus) { return; }
	uint8_t rx[2];
	if (i2c_read(bus, rx, sizeof(rx), ADS7128_I2C_ADDR) != 0) { return; }
	uint16_t raw = (uint16_t)(((uint16_t)rx[0] << 4) | (rx[1] >> 4));   /* 12-bit, left-justified */
	float vbat = ((float)raw / 4096.0f) * BATT_VREF_V * BATT_DIVIDER;
	if (vbat < 0.5f || vbat > 6.0f) { return; }          /* reject garbage */
	if (g_vbat <= 0.0f) { g_vbat = vbat; }               /* seed the EMA on the first sample */
	else { g_vbat = g_vbat + BATT_SMOOTH_ALPHA * (vbat - g_vbat); }
}
#endif /* ROSE_BATT_SENSE */

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
	/* Status LED D8 = ADS7128 GPIO7 (active-low). Push-pull output, start OFF. RMW set-bit ops
	 * leave GPIO6 (ToF XSHUT) + AIN5 (batt) untouched. Enable the LED thread's writes (g_led_bus)
	 * only if the config ACKs, so a target without the ADS7128 stays silent. */
	int led_rc = ads7128_set_bit(bus, ADS7128_PIN_CFG, STATUS_LED_CH)          /* GPIO mode */
		   | ads7128_set_bit(bus, ADS7128_GPIO_CFG, STATUS_LED_CH)         /* output */
		   | ads7128_set_bit(bus, ADS7128_GPO_DRIVE_CFG, STATUS_LED_CH)    /* push-pull */
		   | ads7128_set_bit(bus, ADS7128_GPO_VALUE, STATUS_LED_CH);       /* HIGH = LED off */
	if (led_rc == 0) {
		g_led_bus = bus;
		printk("board_sensor_init: status LED on ADS7128 GPIO%d\n", STATUS_LED_CH);
	}
#if ROSE_BATT_SENSE
	battery_sense_init(bus);   /* configure ADS7128 AIN5 as ADC for battery-voltage sensing */
	printk("board_sensor_init: battery sense on ADS7128 AIN%d (Vbat = Vadc * %d)\n",
	       BATT_ADC_CH, (int)BATT_DIVIDER);
#endif
	return 0;
}
SYS_INIT(board_sensor_init, POST_KERNEL, 80);   /* before CONFIG_SENSOR_INIT_PRIORITY (90) */

/* Move the down VL53L1X off the shared 0x29 power-on address to 0x30, so once the sides are at
 * 0x31-0x34 NOTHING remains at 0x29. On fully-populated boards the down otherwise shares 0x29 with the
 * sides as they power up + pass through it during readdress, and gets intermittently clobbered (stuck
 * 0mm / crosstalk return). The VL53L1X address is volatile -- a power cycle (XSHUT low->high) resets it
 * to 0x29 -- so board_sensor_init()/side_tof_init() always leave it freshly at 0x29, and we move it
 * here ONCE, right before vl53l1x_reinit(). Mechanism (ST UM2356 / samples/riskybird/sensor_bringup):
 * write the new 7-bit address to 16-bit register 0x0001 (VL53L1_I2C_SLAVE__DEVICE_ADDRESS) at the
 * sensor's current address. VL53L1X_DOWN_ADDR MUST equal the vl53l1x@30 DT node reg so the driver then
 * talks to it there. */
#define VL53L1X_DOWN_ADDR  0x30
static int vl53l1x_readdress_down(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_BUS(DT_INST(0, st_vl53l1x)));
	uint8_t reg01[2] = { 0x00, 0x01 };   /* 16-bit reg 0x0001, MSB first */
	uint8_t probe;

	if (!device_is_ready(bus)) {
		return -ENODEV;
	}
	/* Idempotent: if it already answers at the target (warm reset that kept the rail up), done. */
	if (i2c_write_read(bus, VL53L1X_DOWN_ADDR, reg01, sizeof(reg01), &probe, 1) == 0) {
		return 0;
	}
	uint8_t tx[3] = { 0x00, 0x01, VL53L1X_DOWN_ADDR };
	int rc = i2c_write(bus, tx, sizeof(tx), 0x29);
	k_msleep(10);
	if (rc == 0 && i2c_write_read(bus, VL53L1X_DOWN_ADDR, reg01, sizeof(reg01), &probe, 1) == 0) {
		printk("board_sensor_init: down-ToF readdressed 0x29 -> 0x%02x\n", VL53L1X_DOWN_ADDR);
		return 0;
	}
	printk("board_sensor_init: down-ToF readdress -> 0x%02x FAILED (rc=%d)\n", VL53L1X_DOWN_ADDR, rc);
	return -EIO;
}
#endif /* st_vl53l1x present */

#if !(DT_HAS_COMPAT_STATUS_OKAY(st_vl53l1x) && ROSE_BATT_SENSE)
/* Battery sense disabled, or no ADS7128 on this target (e.g. RoSE co-sim) -> nothing to poll. */
static inline void battery_poll(void) { }
#endif

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
#ifndef SAFE_DEBOUNCE_ITERS
#define SAFE_DEBOUNCE_ITERS 15       /* a limit must be exceeded this many CONSECUTIVE control iters
                                      * (~13 ms @ 1 kHz) before latching estop. Rejects single-sample
                                      * transients -- a motor-vibration gyro spike at spin-up, a
                                      * between-sample velocity blip -- while still catching a genuine
                                      * runaway (which persists for 100s of ms) essentially instantly. */
#endif
static volatile bool g_estop;        /* latched emergency stop (cleared only by a reset -- chip OR soft) */
static volatile bool g_arming;       /* autoflight arm-settle countdown in progress (for status LED) */
static volatile uint32_t g_reset_gen; /* bumped by rose_cmd_reset() -> control loop does a soft reset */
static volatile bool g_arm_enabled;   /* FALSE on boot -> arm gate inert; set TRUE only by a RESET cmd (panel) */

/* Arm gate: motors actuate only when armed. Non-autoflight builds are always armed (normal bench
 * behavior); autoflight starts DISARMED and arms via the on-ground/level check (see below). */
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
static volatile bool g_armed;         /* set true by the arming check; false before/after flight */
static int64_t       g_flight_start_ms;
#else
static const bool    g_armed = true;
#endif

/* ---- Autonomous short-hop flight (build -DROSE_AUTOFLIGHT=1) ---------------------------------
 * On boot: a motor "chirp" (see motors_boot_chirp) alerts that the board reset. Arming then
 * requires a deliberate LIFT-AND-PLACE gesture (pick up > LIFT_ARM_THRESHOLD_M, set down level +
 * on-ground + still for PLACE_CONFIRM_MS) so a glitch reboot on the bench can't auto-take-off.
 * Once armed, fly a fixed altitude profile (ramp-up -> hover -> ramp-down) with a hard FLIGHT_MAX_MS
 * cap, then disarm. There is NO x/y position control on this board (no optical flow) -> expect
 * horizontal drift, so keep the hop short + low. The emergency watchdog stays live throughout, and
 * motors run at FAITHFUL duty (needed to hover) clamped to AUTOFLIGHT_MAX_DUTY as a safety ceiling. */
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
/* Arm gate = lift-and-place gesture: the drone must be PICKED UP (down-ToF > LIFT_ARM_THRESHOLD_M)
 * and then SET DOWN level + on-ground + still, held for PLACE_CONFIRM_MS, before it arms. A glitch
 * reboot on a stationary bench never sees the lift -> never auto-arms (a boot chirp still alerts).*/
#ifndef LIFT_ARM_THRESHOLD_M
#define LIFT_ARM_THRESHOLD_M 0.15f  /* must be lifted above this (m) to enable arming */
#endif
#ifndef PLACE_CONFIRM_MS
#define PLACE_CONFIRM_MS     1500   /* level+ground+still must hold this long after placing */
#endif
#ifndef ARM_SETTLE_MS
#define ARM_SETTLE_MS        3000   /* ROSE_ARM_NO_GESTURE: level+ground+still hold time to auto-arm */
#endif
#ifndef ARM_MAX_TILT_RAD
#define ARM_MAX_TILT_RAD    0.10f   /* must be level (~5.7 deg) to arm */
#endif
#ifndef ARM_MAX_HEIGHT_M
#define ARM_MAX_HEIGHT_M    0.020f  /* must be on the ground (<20 mm) to arm */
#endif
#ifndef ARM_MAX_RATE_RADPS
#define ARM_MAX_RATE_RADPS  0.30f   /* must be still to arm */
#endif
#ifndef HOVER_Z_M
#define HOVER_Z_M           0.30f   /* hover altitude target */
#endif
#ifndef T_CLIMB_MS
#define T_CLIMB_MS          1500    /* ramp setpoint 0 -> HOVER_Z_M */
#endif
#ifndef T_HOVER_MS
#define T_HOVER_MS          2500    /* hold */
#endif
#ifndef T_DESCEND_MS
#define T_DESCEND_MS        1500    /* ramp setpoint HOVER_Z_M -> 0 */
#endif
#ifndef LAND_PUSH_M
#define LAND_PUSH_M        -0.20f   /* floor of the continuous below-ground descent ramp: the setpoint
				     * keeps easing down past 0 to here so the loop descends to touchdown */
#endif
#ifndef LAND_Z_THRESH_M
#define LAND_Z_THRESH_M     0.05f   /* considered landed (cut motors) when actual height < this */
#endif
#ifndef FLIGHT_MAX_MS
#define FLIGHT_MAX_MS       10000   /* hard cap regardless of the profile */
#endif
#ifndef AUTOFLIGHT_MAX_DUTY
#define AUTOFLIGHT_MAX_DUTY 0.45f   /* per-motor duty ceiling in flight (hover~0.15; margin above) */
#endif
#endif /* ROSE_AUTOFLIGHT */

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
#if ROSE_BATT_SENSE
	/* Low-voltage cutoff: only a VALID reading (>= 1.0 V) below the threshold trips -- a garbage-low
	 * read (< 1.0 V, sensor fault) is ignored so it can't false-estop mid-flight. Debounced by the
	 * caller (SAFE_DEBOUNCE_ITERS) like every other reason. */
	if (g_vbat >= 1.0f && g_vbat < BATT_CUTOFF_V) {
		return "battery";
	}
#endif
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
static void motors_boot_chirp(void) { /* no motors on the RoSE target */ }
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
/* HARD MOTOR CUT (telemetry / bench-safety builds). When ROSE_MOTORS_INHIBIT=1 the actuator layer
 * NEVER drives the PWM channels above 0 -- send_control() forces all four to 0 and the boot / ready /
 * startup chirps are skipped entirely -- regardless of arm state, controller output, or estop. Use
 * for any unattended build where props may be attached (e.g. the WiFi telemetry bring-up). Verify
 * "MOTORS INHIBITED" appears in the boot log and that no motor duty is ever commanded. */
#ifndef ROSE_MOTORS_INHIBIT
#define ROSE_MOTORS_INHIBIT 0
#endif
static void send_control(const float *u)
{
#if ROSE_MOTORS_INHIBIT
	/* Motors hard-inhibited: force every channel to 0 and never drive PWM. */
	for (int i = 0; i < NACTIONS; i++) {
		pwm_set_pulse_dt(&motors[i], 0);
	}
	(void)u;
	return;
#endif
	/* Emergency cutoff OR disarmed: force every motor to 0 and ignore the command entirely. */
	if (g_estop || !g_armed) {
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
	/* Controller's normalized thrust (u in ~[-0.583, 0.417]) -> physical per-motor duty [0,1].
	 * Battery sag compensation: multiply the raw thrust command by BATT_NOMINAL_V/g_vbat (clamped to
	 * [1.0, BATT_SCALE_MAX]) so commanded thrust holds as the pack drains. Applied BEFORE the [0,1]
	 * clamp and the AUTOFLIGHT_MAX_DUTY cap below; batt_thrust_scale() returns 1.0 (no-op) when
	 * battery sense is disabled or g_vbat looks invalid. */
	const float batt_scale = batt_thrust_scale();
	float duty[NACTIONS];
	for (int i = 0; i < NACTIONS; i++) {
		duty[i] = (u[i] + 0.583f) * batt_scale;
		if (duty[i] < 0.0f) duty[i] = 0.0f;
		if (duty[i] > 1.0f) duty[i] = 1.0f;
	}
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	/* ATTITUDE-PRIORITY ANTI-SATURATION. The collective (altitude) thrust and the roll/pitch/yaw
	 * differentials share the same motor range. If the peak motor would exceed the safety ceiling,
	 * subtract the excess from ALL FOUR: this lowers the COLLECTIVE thrust while preserving the
	 * differential, so attitude authority always survives -- sacrifice a little altitude, never
	 * attitude. (Per-motor clamping instead flattens the differential once the altitude loop maxes
	 * out -> no control -> tip.) This is THE fix for the "bad down-ToF maxes the altitude loop ->
	 * all four pin -> tip/tumble" failure: the drone now climbs LEVEL and recoverable instead. */
	float peak = 0.0f;
	for (int i = 0; i < NACTIONS; i++) {
		if (duty[i] > peak) peak = duty[i];
	}
	if (peak > AUTOFLIGHT_MAX_DUTY) {
		float cut = peak - AUTOFLIGHT_MAX_DUTY;
		for (int i = 0; i < NACTIONS; i++) {
			duty[i] -= cut;   /* collective cut; a low motor may go < 0 -> floored just below */
		}
	}
	for (int i = 0; i < NACTIONS; i++) {
		if (duty[i] < 0.0f) duty[i] = 0.0f;
		pwm_set_pulse_dt(&motors[i], (uint32_t)(motors[i].period * duty[i]));
	}
#else
	for (int i = 0; i < NACTIONS; i++) {
		float d = duty[i] * MOTOR_MAX_DUTY;                /* bench: scale [0,1] -> [0, cap] */
		if (d > MOTOR_MAX_DUTY) d = MOTOR_MAX_DUTY;
		pwm_set_pulse_dt(&motors[i], (uint32_t)(motors[i].period * d));
	}
#endif
}
/* Optional boot "go" signal: pulse all motors at the safety cap for ROSE_START_PULSE_MS, then
 * stop. Used by the handheld IMU tilt test so the operator knows when the stream has started.
 * Runs once in main() BEFORE the control loop, so it is independent of the actuation timeout. */
static void motors_startup_pulse(void)
{
#if ROSE_MOTORS_INHIBIT
	return;   /* motors hard-inhibited */
#endif
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
/* Boot chirp: sequentially spin motors 1-2-3-4 at 10% duty for 0.25 s each. A deliberate, low
 * (sub-hover) alert so the operator KNOWS the board just reset -- important because a glitch reboot
 * (BU-009) is otherwise silent, and combined with the lift-and-place arm gate it makes "the board
 * rebooted" obvious. Runs once at boot, before arming; drives PWM directly (motors still disarmed). */
static void motors_boot_chirp(void)
{
#if ROSE_MOTORS_INHIBIT
	return;   /* motors hard-inhibited */
#endif
	for (int i = 0; i < NACTIONS; i++) {
		pwm_set_pulse_dt(&motors[i], (uint32_t)(motors[i].period * 0.10f));
		k_msleep(250);
		pwm_set_pulse_dt(&motors[i], 0);
		k_msleep(100);
	}
}
/* Ready-to-arm chirp: DISTINCT from the boot chirp (all 4 motors pulse together, twice) so the two
 * are unmistakable -- boot = "board reset" (1-2-3-4 sweep), ready = "sensors up, arm now" (double
 * all-together blip). Fires once the arming gate goes live; drives PWM directly (still disarmed). */
static void motors_ready_chirp(void)
{
#if ROSE_MOTORS_INHIBIT
	return;   /* motors hard-inhibited */
#endif
	for (int k = 0; k < 2; k++) {
		for (int i = 0; i < NACTIONS; i++) {
			pwm_set_pulse_dt(&motors[i], (uint32_t)(motors[i].period * 0.10f));
		}
		k_msleep(150);
		for (int i = 0; i < NACTIONS; i++) {
			pwm_set_pulse_dt(&motors[i], 0);
		}
		k_msleep(120);
	}
}
#else
static void send_control(const float *u) { (void)u; /* no actuator bound */ }
static void motors_startup_pulse(void) { /* no motors bound */ }
static void motors_boot_chirp(void) { /* no motors bound */ }
static void motors_ready_chirp(void) { /* no motors bound */ }
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

/* Hover setpoint. Non-const so autoflight can drive g_setpoint[2] (altitude) along its profile;
 * on non-autoflight builds it stays at TARGET_Z. */
static float g_setpoint[NSTATES] = {0.0f, 0.0f, TARGET_Z, 0,0,0, 0,0,0, 0,0,0};

#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
/* Runtime-tunable flight profile (climb/hover/descend ms, cap ms, hover altitude m), initialized
 * from the compile-time defaults but adjustable LIVE from the ground station (PROFILE / HOVER_Z
 * uplink commands). Read fresh each control iteration, so a change on the panel takes effect on the
 * next flight (set them on the ground before arming). */
static volatile int   g_t_climb_ms    = T_CLIMB_MS;
static volatile int   g_t_hover_ms    = T_HOVER_MS;
static volatile int   g_t_descend_ms  = T_DESCEND_MS;
static volatile int   g_flight_max_ms = FLIGHT_MAX_MS;
static volatile float g_hover_z_m     = HOVER_Z_M;

/* Desired altitude vs time-since-arm (ms): ramp up -> hold -> ramp down. Returns <0 when the
 * profile is complete (caller then disarms). */
static float autoflight_setpoint_z(int64_t t_ms)
{
	const int   tc = (g_t_climb_ms > 0) ? g_t_climb_ms : 1;
	const int   th = (g_t_hover_ms > 0) ? g_t_hover_ms : 0;
	const int   td = (g_t_descend_ms > 0) ? g_t_descend_ms : 1;
	const float hz = g_hover_z_m;
	if (t_ms < tc) {
		return hz * ((float)t_ms / (float)tc);
	}
	t_ms -= tc;
	if (t_ms < th) {
		return hz;
	}
	t_ms -= th;
	/* Descent + landing as ONE continuous downward ramp: from hover, reaching 0 at td and then
	 * continuing GRADUALLY below ground at the same rate to a real touchdown; clamp at LAND_PUSH_M.
	 * (Motor cut is height-based, in the loop.) Longer td = slower, gentler landing. */
	float rate = hz / (float)td;
	float sp = hz - rate * (float)t_ms;
	return (sp < LAND_PUSH_M) ? LAND_PUSH_M : sp;
}
#endif

struct sensor_frame {
	float accel[3], gyro[3], flow[2], height;
	float baro_rel;                            /* baro altitude relative to the arm ref (m); ROSE_BARO */
	bool flow_valid, tof_valid, baro_valid;
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
static uint32_t pf_flow;                            /* optical-flow read (control-loop side) */
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

/* ---- optical-flow attitude compensation (ROSE_FLOW) --------------------------------------------
 * The PMW3901 measures the ground's ANGULAR velocity across its FOV, which mixes translation with
 * body rotation; the down-ToF gives a SLANT range, not vertical height. Both are attitude effects:
 *   - gyro comp: subtract rotation-induced flow (body pitch rate -> fwd/ax, roll rate -> left/ay) so
 *                only translational flow remains. Exact once FLOW_RAD_PER_COUNT is calibrated (then
 *                flow + gyro share rad/s units); before that it still cancels the SIGN, so an
 *                IN-PLACE rotation test (raw flow swings, compensated ~flat) verifies the signs.
 *   - tilt comp: true vertical height h = d_tof * cos(roll) * cos(pitch).
 * Attitude is the PREVIOUS estimator tick (cached after est.get_state); read_sensor_frame runs
 * before est.update, so it's one loop old (~2-3 ms) -- negligible. Flip a *_SIGN if the in-place
 * test grows that axis instead of cancelling it. */
#ifndef FLOW_GYRO_COMP
#define FLOW_GYRO_COMP 1
#endif
#ifndef FLOW_TILT_COMP
#define FLOW_TILT_COMP 1
#endif
/* Signs verified two ways: bench regression of raw flow vs gyro (pitch slope <0, roll slope >0)
 * AND the Crazyflie flow model, which is (v/h - omega_pitch) for X but (v/h + omega_roll) for Y --
 * i.e. OPPOSITE gyro signs on the two axes, matching the bench result. */
#ifndef FLOW_GYRO_PITCH_SIGN
#define FLOW_GYRO_PITCH_SIGN (-1.0f)   /* ax(fwd)  -= -gyro[1]: v/h = flow + pitch rate */
#endif
#ifndef FLOW_GYRO_ROLL_SIGN
#define FLOW_GYRO_ROLL_SIGN  (+1.0f)   /* ay(left) -=  gyro[0]: v/h = flow - roll rate */
#endif
/* Reject the flow sample when the body roll/pitch RATE exceeds this (rad/s). During a fast rotation
 * the flow is rotation-dominated and the gyro compensation leaves a residual (imperfect), which leaks
 * into the noisier y flow -> inflates vy -> the velocity loop banks harder -> a self-exciting
 * roll<->flow oscillation that trips the velocity watchdog. Gating flow out during fast rolls breaks
 * that loop; gentle hover corrections (well under this) keep their flow. -DFLOW_GYRO_MAX=0 disables. */
#ifndef FLOW_GYRO_MAX
#define FLOW_GYRO_MAX 1.2f
#endif
static float g_att_roll, g_att_pitch, g_att_yaw;   /* last estimator attitude, Gibbs qx/qw,qy/qw,qz/qw */

/* ---- startup gyro-bias auto-calibration -----------------------------------------------------
 * The Mahony filter has no online gyro-bias term and runs a low accel-trim gain, so a fixed gyro
 * bias drifts attitude and roughens the rate loop. While the drone sits still at boot (we already
 * require a level+still settle before arming), average the gyro -- which should read 0 at rest -- to
 * estimate the bias, then subtract it from every frame. Any axis over the "still" threshold restarts
 * the window so a bump can't poison the average. Arming is gated on the cal completing. */
#ifndef GYRO_CAL_SECONDS
#define GYRO_CAL_SECONDS     2.0f      /* seconds of continuous stillness to average */
#endif
#ifndef GYRO_CAL_STILL_RADPS
#define GYRO_CAL_STILL_RADPS 0.30f     /* per-axis rate below which the board counts as "still" */
#endif
/* Continuous ground bias re-tracking -- the fix for "flight 1 good, each later flight worse". The
 * boot cal freezes g_gyro_bias at one temperature, but the BMI088 zero-rate offset drifts as the IMU
 * warms over a session. A soft-RESET keeps the frozen value, and a battery-only unplug does NOT
 * re-measure it if USB keeps the ESP powered -- only a chip reset re-runs the boot cal, which is why
 * re-flashing "fixes" it. The stale residual is injected into the rate loop, Mahony, AND the flow
 * gyro-compensation, growing the drift flight-over-flight. So while DISARMED and very still, slowly
 * pull g_gyro_bias toward the live rate: every flight then arms with a fresh, current-temperature
 * bias -- no chip reset needed. Frozen while armed (no in-flight dynamics) and stillness-gated + slow,
 * so it can't be contaminated the way the old rushed one-shot per-RESET recal was. */
#ifndef GBIAS_TRACK_GAIN
#define GBIAS_TRACK_GAIN 0.0008f       /* per-sample EMA (~1.3 s time constant at 1 kHz) */
#endif
#ifndef GBIAS_TRACK_STILL_RADPS
#define GBIAS_TRACK_STILL_RADPS 0.10f  /* only re-track when very still (tighter than the boot-cal gate) */
#endif
static float g_gyro_bias[3];           /* measured gyro bias (rad/s); 0 until the cal completes */
static volatile bool g_gyro_cal_done;  /* startup bias cal finished -> OK to arm */
/* Gyro-cal accumulators at FILE scope so a soft-reset (rose_cmd_reset) can restart the cal cleanly;
 * if they stayed function-local statics, re-clearing g_gyro_cal_done would leave a stale gstill_since
 * timestamp and the recal would "complete" instantly on garbage. Zero at boot (BSS) = same as before. */
static double  g_gcal_sum[3];
static int     g_gcal_n;
static int64_t g_gcal_since;
/* Barometer reference cal -- established by averaging pressure over the SAME still startup window as
 * the gyro-bias cal (co-calibrated). Reset together so a soft-reset (rose_cmd_reset) re-cals both. */
static double  g_bcal_sum;     /* reference-pressure accumulator (kPa) */
static int     g_bcal_n;
static float   g_baro_p0;      /* reference pressure (kPa); frozen after the cal window */
static bool    g_baro_have;    /* reference established (gyro cal complete) */
/* On a soft-RESET, KEEP the pristine gyro-bias cal measured during the long, untouched boot
 * bringup instead of re-calibrating. The recal window runs right after a landing/crash while the
 * drone is being repositioned by hand -> it captures a contaminated "at-rest" bias that flight 1
 * (boot cal) never has. That residual rate bias is a phase error in the attitude estimate, which
 * erodes the loop's margin, so the ~0.5 Hz velocity-loop oscillation grows flight-over-flight (the
 * "great flight 1, progressively worse" pattern that only clears on a chip reset). Reusing the boot
 * cal makes every soft-RESET flight start bit-identical to flight 1. Set -DROSE_RECAL_ON_RESET=1 to
 * restore per-RESET recal (then each RESET needs a still, hands-off placement to be clean). */
#ifndef ROSE_RECAL_ON_RESET
#define ROSE_RECAL_ON_RESET 0
#endif
static void __attribute__((unused)) gyro_cal_restart(void)
{
	g_gyro_cal_done = false;
	g_gyro_bias[0] = g_gyro_bias[1] = g_gyro_bias[2] = 0.0f;
	g_gcal_sum[0] = g_gcal_sum[1] = g_gcal_sum[2] = 0.0;
	g_gcal_n = 0;
	g_gcal_since = 0;
	g_bcal_sum = 0.0; g_bcal_n = 0; g_baro_p0 = 0.0f; g_baro_have = false;
}

/* Barometric altitude (m) relative to a reference pressure, via the international barometric
 * formula. The p/p0 RATIO cancels units, so p and p0 may be any consistent unit (here kPa, the
 * Zephyr SENSOR_CHAN_PRESS convention). Referencing to p0 keeps the number small and drift-immune. */
#if HAVE_BARO
static float baro_rel_altitude_m(float p_kpa, float p0_kpa)
{
	if (p0_kpa <= 0.0f || p_kpa <= 0.0f) {
		return 0.0f;
	}
	return 44330.0f * (1.0f - powf(p_kpa / p0_kpa, 0.1902949f));   /* exponent = 1/5.255 */
}
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
	/* Startup gyro-bias cal: average the (should-be-zero) gyro while still, then subtract it from
	 * every frame so the whole chain (Mahony attitude, rate loop, flow gyro-comp) sees a debiased
	 * rate. Pre-cal the bias is 0 (subtraction is a no-op); motors stay disarmed until it finishes. */
	if (!g_gyro_cal_done) {
		bool gstill = fabsf(f->gyro[0]) < GYRO_CAL_STILL_RADPS &&
			      fabsf(f->gyro[1]) < GYRO_CAL_STILL_RADPS &&
			      fabsf(f->gyro[2]) < GYRO_CAL_STILL_RADPS;
		int64_t gnow = k_uptime_get();
		if (!gstill) {
			g_gcal_since = 0; g_gcal_sum[0] = g_gcal_sum[1] = g_gcal_sum[2] = 0.0; g_gcal_n = 0;   /* bump -> restart */
		} else {
			if (g_gcal_since == 0) { g_gcal_since = gnow; g_gcal_sum[0] = g_gcal_sum[1] = g_gcal_sum[2] = 0.0; g_gcal_n = 0; }
			g_gcal_sum[0] += f->gyro[0]; g_gcal_sum[1] += f->gyro[1]; g_gcal_sum[2] += f->gyro[2]; g_gcal_n++;
			if (gnow - g_gcal_since >= (int64_t)(GYRO_CAL_SECONDS * 1000.0f) && g_gcal_n > 0) {
				g_gyro_bias[0] = (float)(g_gcal_sum[0] / g_gcal_n);
				g_gyro_bias[1] = (float)(g_gcal_sum[1] / g_gcal_n);
				g_gyro_bias[2] = (float)(g_gcal_sum[2] / g_gcal_n);
				g_gyro_cal_done = true;
				printk("gyro-cal: bias=[%d %d %d] millirad/s (%d samples) -- ready to arm\n",
				       (int)(g_gyro_bias[0] * 1000.0f), (int)(g_gyro_bias[1] * 1000.0f),
				       (int)(g_gyro_bias[2] * 1000.0f), g_gcal_n);
			}
		}
	}
	/* Re-track the gyro bias to the current IMU temperature while parked (see GBIAS_TRACK_* above).
	 * Uses the raw (pre-subtraction) rate: when the board is still, that rate IS the live bias. */
	if (g_gyro_cal_done && !g_armed &&
	    fabsf(f->gyro[0] - g_gyro_bias[0]) < GBIAS_TRACK_STILL_RADPS &&
	    fabsf(f->gyro[1] - g_gyro_bias[1]) < GBIAS_TRACK_STILL_RADPS &&
	    fabsf(f->gyro[2] - g_gyro_bias[2]) < GBIAS_TRACK_STILL_RADPS) {
		g_gyro_bias[0] += GBIAS_TRACK_GAIN * (f->gyro[0] - g_gyro_bias[0]);
		g_gyro_bias[1] += GBIAS_TRACK_GAIN * (f->gyro[1] - g_gyro_bias[1]);
		g_gyro_bias[2] += GBIAS_TRACK_GAIN * (f->gyro[2] - g_gyro_bias[2]);
	}
	f->gyro[0] -= g_gyro_bias[0];
	f->gyro[1] -= g_gyro_bias[1];
	f->gyro[2] -= g_gyro_bias[2];
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
	/* ---- Barometer relative altitude (ROSE_BARO) ----------------------------------------------
	 * Rate-limited fetch (BMP388 I2C read is short, so inline is fine) + zero-order hold. The
	 * reference pressure p0 is CO-CALIBRATED WITH THE GYRO: averaged over the same still startup
	 * window (motors disarmed), then frozen -- so baro_rel is height above the cal point. The
	 * estimator re-anchors this to the ToF floor (baro_bias), so only short-term smoothness /
	 * gap-filling matters. baro_valid stays false until the (gyro+baro) cal completes. */
	f->baro_rel = 0.0f;
	f->baro_valid = false;
#if ROSE_BARO
#if HAVE_BARO
	{
		static int64_t baro_next_ms = 0;
		static float   baro_last = 0.0f;  /* last relative altitude (m), zero-order held */
		int64_t now_ms = k_uptime_get();
		if (now_ms >= baro_next_ms) {
			baro_next_ms = now_ms + BARO_FETCH_PERIOD_MS;
			if (sensor_sample_fetch(baro_dev) == 0) {
				struct sensor_value pv;
				sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &pv);
				float p = (float)sensor_value_to_double(&pv);   /* kPa */
				if (p > 0.0f) {
					if (!g_gyro_cal_done) {
						/* accumulate the reference pressure over the gyro-cal still window */
						g_bcal_sum += p; g_bcal_n++;
					} else {
						if (!g_baro_have) {   /* cal just finished -> freeze the averaged reference */
							g_baro_p0 = (g_bcal_n > 0) ? (float)(g_bcal_sum / g_bcal_n) : p;
							g_baro_have = true;
							printk("baro-cal: p0=%d.%03d kPa (%d samples) -- altitude referenced (fused with ToF)\n",
							       (int)g_baro_p0, ((int)(g_baro_p0 * 1000.0f)) % 1000, g_bcal_n);
						}
						baro_last = baro_rel_altitude_m(p, g_baro_p0);
					}
				}
			}
		}
		f->baro_valid = g_baro_have;
		f->baro_rel = baro_last;
	}
#else
#warning "ROSE_BARO=1 but no `baro` DT alias -- barometer read stubbed (baro_valid stays false). Add a bosch,bmp388 node + `baro` alias + CONFIG_BMP388 (see report / README)."
#endif /* HAVE_BARO */
#endif /* ROSE_BARO */
	/* f->height stays the RAW down-ToF slant range here. The slant->vertical tilt correction now lives
	 * in the estimator (est.update, on its FRESH R[8]) so est_z is the true vertical height, and the
	 * telemetry can show raw slant (tofh) vs corrected estimate (z) side by side. The optical-flow
	 * path below tilt-corrects its OWN local copy (it needs vertical height before est.update runs). */
#if defined(ROSE_FLOW) && ROSE_FLOW
	/* Real optical flow (PMW3901): body velocity (m/s) = body angular flow (rad/s) * height (m).
	 * Needs a valid ToF height; SQUAL + staleness gating is inside flow_get(). Overrides the flow=0
	 * default so the estimator gets TRUE horizontal velocity instead of a forced-zero one. */
	{
		uint32_t _pfl = PF_NOW();
		float ax, ay; int sq; bool fv;
		flow_get(&ax, &ay, &sq, &fv);
		PF_ACC(pf_flow, _pfl);
		/* Gate flow out during fast body rotation (flow is rotation-dominated + gyro-comp residual
		 * leaks into vy -> self-exciting roll<->flow oscillation). FLOW_GYRO_MAX=0 disables the gate. */
		bool rate_ok = (FLOW_GYRO_MAX <= 0.0f) ||
			       (fabsf(f->gyro[0]) < FLOW_GYRO_MAX && fabsf(f->gyro[1]) < FLOW_GYRO_MAX);
		if (fv && rate_ok && f->tof_valid && f->height > 0.02f) {
			/* Gyro-compensate: strip rotation-induced flow so only translation remains. Body
			 * rates (rad/s) share units with the angular flow. */
#if FLOW_GYRO_COMP
			ax -= FLOW_GYRO_PITCH_SIGN * f->gyro[1];   /* pitch rate -> forward flow */
			ay -= FLOW_GYRO_ROLL_SIGN  * f->gyro[0];   /* roll rate  -> left flow */
#endif
			/* f->height is the RAW slant (the estimator tilt-corrects for altitude on its own fresh
			 * R[8]); the flow needs VERTICAL height too, so correct a local copy on the cached attitude.
			 * cos(tilt) = R_zz = (1 - qx^2 - qy^2 + qz^2)/(1 + qx^2 + qy^2 + qz^2) from the Gibbs state. */
			float ga = g_att_roll, gb = g_att_pitch, gc = g_att_yaw;
			float ct = (1.0f - ga*ga - gb*gb + gc*gc) / (1.0f + ga*ga + gb*gb + gc*gc);
			float h = f->height * (ct > 0.0f ? ct : 0.0f);
			/* Clamp the flow-derived velocity to a physical bound. v = angular_flow * height, so
			 * at large ToF height flow NOISE is amplified into >10 m/s spikes (seen at h=2.5 m);
			 * feeding those to the estimator (which then gates them as outliers -> predict-only ->
			 * runaway) is what makes est-v blow up. The drone can't translate faster than this. */
			const float FLOW_VEL_MAX = 3.0f;
			float vfx = ax * h, vfy = ay * h;   /* body-frame horizontal velocity (m/s) */
			f->flow[0] = vfx >  FLOW_VEL_MAX ?  FLOW_VEL_MAX : (vfx < -FLOW_VEL_MAX ? -FLOW_VEL_MAX : vfx);
			f->flow[1] = vfy >  FLOW_VEL_MAX ?  FLOW_VEL_MAX : (vfy < -FLOW_VEL_MAX ? -FLOW_VEL_MAX : vfy);
			f->flow_valid = true;
		} else {
			f->flow[0] = f->flow[1] = 0.0f;
			f->flow_valid = false;   /* predict-only; do NOT force velocity to zero */
		}
	}
#endif
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
#endif /* ROSE_THREADED: thread-block defs; the status LED below is compiled in all configs */

/* ---- Status LED (D8 = ADS7128 GPIO7, active-low) --------------------------------------------
 * A low-priority thread renders a blink pattern DERIVED from the existing flight-state globals
 * (g_estop / g_gyro_cal_done / g_armed / g_arming / g_vbat) -- no scattered setters. It drives
 * GPIO7 via the ADS7128 set/clear-bit RMW, which never disturbs GPIO6 (ToF XSHUT) or AIN5 (batt).
 * The bus is shared with the control loop, but writes happen only on a pattern EDGE (a few per
 * second) and this thread sits below PRIO_IO, so the worst case is the control loop waiting one
 * ~150 us I2C transfer. Patterns match samples/riskybird/status_led (the bench demo). */
enum led_pattern { LEDP_CAL, LEDP_READY, LEDP_ARMING, LEDP_ARMED, LEDP_FAULT, LEDP_LOWBATT, LEDP_LOCKED };

static inline void status_led_write(bool on)
{
	if (!g_led_bus) { return; }
	if (on) { ads7128_clr_bit(g_led_bus, ADS7128_GPO_VALUE, STATUS_LED_CH); }  /* LOW  = on  */
	else    { ads7128_set_bit(g_led_bus, ADS7128_GPO_VALUE, STATUS_LED_CH); }  /* HIGH = off */
}

/* Highest-priority condition wins. */
static enum led_pattern status_led_pattern(void)
{
	if (g_estop)          { return LEDP_FAULT; }   /* watchdog / IMU-lost latch */
	if (!g_gyro_cal_done) { return LEDP_CAL; }     /* boot + sensor init + gyro-bias cal */
	if (g_armed)          { return LEDP_ARMED; }   /* motors live */
	if (g_arming)         { return LEDP_ARMING; }  /* level+still arm countdown */
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	if (!g_arm_enabled)   { return LEDP_LOCKED; }  /* disarmed on boot -- waiting for a RESET cmd to enable */
#endif
#if ROSE_BATT_SENSE
	{ float v = g_vbat; if (v >= 1.0f && v <= 5.0f && v < BATT_ARM_MIN_V) { return LEDP_LOWBATT; } }
#endif
	return LEDP_READY;                             /* disarmed, waiting to arm */
}

#define LED_TICK_MS 20   /* renderer tick; pattern periods are expressed in ticks */
static bool status_led_on(enum led_pattern p, int ph)
{
	switch (p) {
	case LEDP_CAL:     return (ph % 6) < 3;                 /* ~4 Hz busy blink */
	case LEDP_READY:   return (ph % 75) < 3;               /* 60 ms blip / 1.5 s (heartbeat) */
	case LEDP_LOCKED:  return (ph % 50) < 25;              /* ~1 Hz even blink = locked (send RESET to enable) */
	case LEDP_ARMING: {                                     /* accelerating: period 24 -> 4 ticks */
		int per = 24 - ph / 5;
		if (per < 4) { per = 4; }
		return (ph % per) < (per / 2);
	}
	case LEDP_ARMED:   return true;                        /* SOLID ON */
	case LEDP_FAULT:   return (ph % 2) == 0;               /* ~25 Hz strobe */
	case LEDP_LOWBATT: {                                    /* double-blip / 1 s */
		int q = ph % 50;
		return (q < 3) || (q >= 8 && q < 11);
	}
	}
	return false;
}

K_THREAD_STACK_DEFINE(led_stack, 1024);
static struct k_thread led_t;
#define PRIO_LED 10   /* below IO/EST/CTRL (they preempt it); above keepalive */

static void status_led_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	enum led_pattern last_p = (enum led_pattern)-1;
	int ph = 0;
	bool last_on = false, first = true;
	for (;;) {
		enum led_pattern p = status_led_pattern();
		if (p != last_p) { last_p = p; ph = 0; }   /* restart phase on a state change */
		bool on = status_led_on(p, ph);
		if (first || on != last_on) { status_led_write(on); last_on = on; first = false; }
		ph++;
		k_msleep(LED_TICK_MS);
	}
}

#if ROSE_THREADED
static void io_block(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (int iter = 0; CTRL_RUN_FOREVER || iter < CTRL_ITERS; iter++) {
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

		est.update(f.accel, f.gyro, f.flow, f.flow_valid, f.height, f.tof_valid,
			   f.baro_rel, f.baro_valid, CTRL_DT);
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

#if defined(CONFIG_WIFI)
/* Uplink command hooks (declared in telem_wifi.h); the command-RX thread calls these. Each just
 * pokes a control-loop shared flag consumed on the next iteration -- no locks, no blocking. */
extern "C" void rose_cmd_estop(void) { g_estop = true; }   /* latched remote kill */
extern "C" void rose_cmd_disarm(void)
{
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	g_armed = false;   /* clear the arm latch (re-arm via the gate). Non-autoflight g_armed is const. */
#endif
}
extern "C" void rose_cmd_set_hover_z(float m)
{
	if (m < 0.0f) { m = 0.0f; }
	if (m > SAFE_MAX_HEIGHT_M) { m = SAFE_MAX_HEIGHT_M; }
	g_setpoint[2] = m;   /* non-autoflight hover builds regulate to this directly */
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	g_hover_z_m = m;     /* autoflight: sets the profile's hover altitude (used next flight) */
#endif
}
/* Live flight-profile tuning from the ground station: climb / hover / descend durations + hard cap
 * (all ms). Non-positive args are ignored (keep current). Takes effect on the next flight. */
extern "C" void rose_cmd_set_profile(int climb_ms, int hover_ms, int descend_ms, int max_ms)
{
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	if (climb_ms   > 0) { g_t_climb_ms    = climb_ms; }
	if (hover_ms   >= 0) { g_t_hover_ms   = hover_ms; }
	if (descend_ms > 0) { g_t_descend_ms  = descend_ms; }
	if (max_ms     > 0) { g_flight_max_ms = max_ms; }
	printk("AUTOFLIGHT: profile set -- climb=%d hover=%d descend=%d cap=%d ms\n",
	       g_t_climb_ms, g_t_hover_ms, g_t_descend_ms, g_flight_max_ms);
#else
	(void)climb_ms; (void)hover_ms; (void)descend_ms; (void)max_ms;
#endif
}
/* Soft reset: return the FC to the just-booted state (clear estop, disarm, re-run gyro cal, re-init
 * estimator/controller) WITHOUT a chip reset. Done in the control loop; here we just bump the gen. */
extern "C" void rose_cmd_reset(void) { g_arm_enabled = true; g_reset_gen++; }   /* enable arming + soft reset */
#endif /* CONFIG_WIFI */

int main(void)
{
#if defined(ROSE_FLIGHTLOG_DUMP) && ROSE_FLIGHTLOG_DUMP
	/* Dump-only build: read the stored flight log back over USB as CSV, then idle. Runs before any
	 * sensor/actuator setup so it works even with the drone off the bench. */
	k_msleep(500);   /* let USB CDC enumerate before we print */
	printk("flight_controller: FLIGHTLOG DUMP MODE\n");
	flightlog_dump();
	return 0;
#endif
	if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev)) {
		printk("flight_controller: FAIL (IMU not ready)\n");
		return -1;
	}
	ctrl.init();
	est.init(0.0f, 0.0f, START_Z);
#if defined(ROSE_MOTORS_INHIBIT) && ROSE_MOTORS_INHIBIT
	printk("flight_controller: MOTORS INHIBITED (ROSE_MOTORS_INHIBIT=1) -- PWM forced to 0, "
	       "no chirps, no actuation\n");
#endif

#if defined(ROSE_FLIGHTLOG) && ROSE_FLIGHTLOG
	flightlog_init();   /* erase 'storage' partition + ready to append (see flightlog.h) */
#ifndef ROSE_FLIGHTLOG_DIV
#define ROSE_FLIGHTLOG_DIV 20   /* log every Nth control tick (~50 Hz at a 1 kHz loop) */
#endif
#endif

	motors_startup_pulse();   /* optional boot "go" signal (ROSE_START_PULSE_MS); no-op if unset */
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	/* Boot chirp (1-2-3-4 sweep) = "the board just reset". Fires EARLY, before the ~12 s sensor
	 * bring-up. A DISTINCT ready chirp fires later (below), when the arming gate actually goes live. */
	motors_boot_chirp();
#endif

#if defined(ROSE_BUMPER) && ROSE_BUMPER
	/* Bring up the 4 side VL53L5CX wall sensors (readdress 0x31-0x34 via ADS7128, then read on a
	 * background thread). MUST run BEFORE vl53l1x_reinit: the sides default to 0x29 (same as the down
	 * VL53L1X) and pass through it while being reprogrammed, so side_tof_init() holds the down sensor
	 * in reset for the whole readdress, then re-powers it (alone at 0x29) -- and we init it just
	 * below. Adds ~12 s to boot (4x firmware upload). Control loop reads the cache non-blocking. */
	{
		int n = side_tof_init();
		printk("flight_controller: side-ToF bumper: %d/4 sensors up\n", n);
	}
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(st_vl53l1x)
	/* The st,vl53l1x driver defers the full ST boot (DataInit/StaticInit); nothing runs it unless
	 * the app asks. Trigger it once here (XSHUT rail raised by board_sensor_init, or re-raised by
	 * side_tof_init after the side readdress) so the down-ToF actually ranges -- without this every
	 * sample_fetch hits an uninitialized device and floods "Failed to write". No-op on RoSE. */
	{
		/* First move the down sensor off the shared 0x29 -> 0x30 (see vl53l1x_readdress_down); then
		 * run the deferred ST init at its new DT address (vl53l1x@30). */
		vl53l1x_readdress_down();
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

#if defined(ROSE_FLOW) && ROSE_FLOW
	/* Bring up the PMW3901 optical-flow sensor (SPI2) + start its background reader thread. Flow is
	 * read non-blocking (like the ToFs) and feeds the estimator's horizontal-velocity update -> real
	 * position/velocity sensing instead of the ROLL_TRIM dead-reckoning workaround. */
	{
		int rc = flow_init();
		printk("flight_controller: optical flow %s\n",
		       rc == 0 ? "up" : "NOT detected (flow disabled)");
	}
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

#if defined(CONFIG_WIFI)
	/* Bring up the WiFi SoftAP + UDP telemetry downlink ONCE, before the control loop starts. The
	 * heavy WiFi TX runs on telem_wifi's OWN low-priority thread; the control loop only ever does a
	 * non-blocking mutex copy (telem_wifi_publish), so this never stalls the ~1 kHz loop. Opt-in via
	 * telem.conf (docs/TELEMETRY_PLAN.md phase 2); no-op / not compiled without CONFIG_WIFI. */
	{
		int rc = telem_wifi_init();
		printk("flight_controller: WiFi telemetry SoftAP %s\n",
		       rc == 0 ? "starting (join 'riskybird-<id>', UDP :14550)" : "FAILED to start");
	}
#endif

	/* Status LED: start the state-derived pattern renderer (only if the ADS7128 LED config ACK'd
	 * at board_sensor_init). Low priority + edge-only I2C writes -> negligible load on the loop. */
	if (g_led_bus) {
		k_thread_create(&led_t, led_stack, K_THREAD_STACK_SIZEOF(led_stack),
				status_led_thread, NULL, NULL, NULL, PRIO_LED, 0, K_NO_WAIT);
		k_thread_name_set(&led_t, "status_led");
		printk("flight_controller: status LED up (ADS7128 GPIO%d, state-derived patterns)\n",
		       STATUS_LED_CH);
	}

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
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	/* "Ready to arm" chirp (distinct double all-together blip): sensors are up and the arming gate is
	 * now live -- THIS is the cue to do the lift-and-place gesture (untethered = no console). */
	motors_ready_chirp();
#endif
	struct sensor_frame f;
	float state[NSTATES], u[NACTIONS];
	/* Use the REAL measured loop period, not the nominal CTRL_DT. On this soft-float target the
	 * loop runs ~15 Hz (dt ~67 ms), not the 200 Hz the design assumes; feeding the fixed 5 ms made
	 * the estimator integrate ~13x too slow, so real tilts barely registered. Measure dt each iter. */
	int64_t t_prev = k_uptime_get();
	int imu_miss = 0;
	for (int iter = 0; CTRL_RUN_FOREVER || iter < CTRL_ITERS; iter++) {
		if (!read_sensor_frame(&f)) {
			printk("flight_controller: IMU fetch error\n");
			if (++imu_miss >= SAFE_MAX_IMU_MISS && !g_estop) {
				g_estop = true;
				printk("flight_controller: EMERGENCY CUTOFF -- IMU lost (%d misses); "
				       "motors OFF (reset to clear)\n", imu_miss);
#if defined(ROSE_FLIGHTLOG) && ROSE_FLIGHTLOG
				flightlog_flush();   /* persist the log at flight-end (estop) */
#endif
			}
			if (g_estop) {
				send_control(u);   /* g_estop forces motors to 0 (u not read) */
			}
			continue;
		}
		imu_miss = 0;
		/* Soft RESET (uplink cmd): return to the just-booted state WITHOUT a chip reset -- clear the
		 * estop latch, disarm, and re-init the estimator + controller (which clears all integrators).
		 * The gyro-bias cal is KEPT from boot (see gyro_cal_restart above): re-calibrating in the
		 * rushed post-landing window contaminated the bias and made each flight progressively worse.
		 * Runs BEFORE est.update so it takes effect this iteration. (g_reset_gen==reset_seen==0 at
		 * boot -> no spurious reset.) */
		static uint32_t reset_seen;
		if (g_reset_gen != reset_seen) {
			reset_seen = g_reset_gen;
			g_estop = false;
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
			g_armed = false; g_arming = false; g_flight_start_ms = 0;
#endif
#if ROSE_RECAL_ON_RESET
			gyro_cal_restart();
#endif
			est.init(0.0f, 0.0f, 0.0f);
			ctrl.init();
			g_setpoint[2] = 0.0f;
			printk("flight_controller: SOFT RESET -- estop cleared, disarmed"
			       "%s\n", ROSE_RECAL_ON_RESET ? ", recalibrating gyro" : " (keeping boot gyro cal)");
		}
		/* Battery voltage: poll the ADS7128 ADC at a LOW rate (every BATT_CHECK_DIV iters). One short
		 * I2C transfer shared with the ToF bus -- infrequent so it adds negligible average load; no-op
		 * unless -DROSE_BATT_SENSE=1 on a board that has the ADS7128. */
		if ((iter % BATT_CHECK_DIV) == 0) {
			battery_poll();
		}
		int64_t t_now = k_uptime_get();
		float dt = (float)(t_now - t_prev) * 1e-3f;   /* real loop period (s) */
		if (dt <= 0.0f || dt > 0.5f) dt = CTRL_DT;     /* first-iter / stall guard */
		t_prev = t_now;
		uint32_t _pe = PF_NOW();
		est.update(f.accel, f.gyro, f.flow, f.flow_valid, f.height, f.tof_valid,
			   f.baro_rel, f.baro_valid, dt);
		est.get_state(state);
		g_att_roll = state[3]; g_att_pitch = state[4]; g_att_yaw = state[5];   /* cache for next frame's comp */
		PF_ACC(pf_est, _pe);
		/* Emergency watchdog: latch a kill if attitude/rate/velocity exceed safe limits. Checked
		 * every iteration before actuation; send_control() enforces the cut. */
		/* FLIGHT-only watchdog: gate on g_armed. The pre-arm lift-and-place swings the board by hand
		 * (fast enough to spike the flow-velocity / tilt / rate limits), which must NOT latch estop
		 * before takeoff. Motors are already forced off while disarmed (send_control), so there is
		 * nothing to guard until armed; once armed this protects the entire flight. */
		if (g_armed && !g_estop) {
			const char *why = safety_violation(state, f.gyro);
			static int viol_count;   /* consecutive iters in violation (debounce transient spikes) */
			viol_count = (why != NULL) ? (viol_count + 1) : 0;
			if (why != NULL && viol_count >= SAFE_DEBOUNCE_ITERS) {
				g_estop = true;
				printk("flight_controller: EMERGENCY CUTOFF -- %s limit exceeded (%d iters); "
				       "motors OFF (reset to clear)\n", why, viol_count);
#if defined(ROSE_FLIGHTLOG) && ROSE_FLIGHTLOG
				/* Final record: encode WHICH limit tripped in flags[2..4] (0=none 1=tilt 2=rate
				 * 3=velocity 4=height 5=battery; first char of `why` is unique per reason) and carry
				 * the raw gyro x/y in the fvx/fvy columns -- the rate/gyro path isn't otherwise logged,
				 * so this is how we tell a vibration rate-spike from a real tilt/velocity runaway. */
				uint8_t rc = (why[0]=='t')?1 : (why[0]=='r')?2 : (why[0]=='v')?3 : (why[0]=='h')?4 :
					     (why[0]=='b')?5 : 0;
				struct flight_rec er = {0};
				er.t_ms     = (uint32_t)k_uptime_get();
				er.roll_mrad  = (int16_t)(state[3] * 1000.0f);
				er.pitch_mrad = (int16_t)(state[4] * 1000.0f);
				er.yaw_mrad   = (int16_t)(state[5] * 1000.0f);
				er.z_mm     = (int16_t)(state[2] * 1000.0f);
				er.vz_mmps  = (int16_t)(state[8] * 1000.0f);
				er.vx_mmps  = (int16_t)(state[6] * 1000.0f);
				er.vy_mmps  = (int16_t)(state[7] * 1000.0f);
				er.fvx_mmps = (int16_t)(f.gyro[0] * 1000.0f);   /* gyro x (rad/s * 1000), estop record only */
				er.fvy_mmps = (int16_t)(f.gyro[1] * 1000.0f);   /* gyro y */
				er.flags = (uint8_t)(FLIGHT_FLAG_ESTOP |
						     (f.tof_valid ? FLIGHT_FLAG_TOF_VALID : 0) | (rc << 2));
				flightlog_write(&er);
				flightlog_flush();   /* persist the log at flight-end (estop) */
#endif
			}
		}
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
		/* Arming = LIFT-AND-PLACE gesture (glitch-reboot-safe): must be picked up past
		 * LIFT_ARM_THRESHOLD_M, then set down level + on-ground + still held for PLACE_CONFIRM_MS.
		 * Then fly the altitude profile until it completes or the hard cap, then disarm.
		 * send_control() gates motors on g_armed the entire time. */
		{
			static bool     announced;
			static int64_t  arm_since;    /* ms since arm conditions held (0 = not yet) */
			static bool     flight_done;  /* one-shot: latch after a completed flight, no auto re-arm */
			static uint32_t arm_reset_seen;
			if (arm_reset_seen != g_reset_gen) {   /* soft RESET: clear the arm-gate latches so it re-arms */
				arm_reset_seen = g_reset_gen;
				announced = false; arm_since = 0; flight_done = false;
			}
			g_arming = false;             /* status LED: default; set true below while counting down */
			if (!g_armed && !g_estop && !flight_done) {
				bool level  = fabsf(state[3]) < ARM_MAX_TILT_RAD &&
					      fabsf(state[4]) < ARM_MAX_TILT_RAD;
				bool ground = f.tof_valid && state[2] < ARM_MAX_HEIGHT_M;
				bool still  = fabsf(f.gyro[0]) < ARM_MAX_RATE_RADPS &&
					      fabsf(f.gyro[1]) < ARM_MAX_RATE_RADPS &&
					      fabsf(f.gyro[2]) < ARM_MAX_RATE_RADPS;
#if defined(ROSE_ARM_NO_GESTURE) && ROSE_ARM_NO_GESTURE
				/* Simple autoflight: NO gesture. Auto-arm once the drone has been level + still
				 * continuously for ARM_SETTLE_MS. Deliberately does NOT require the down-ToF "on-
				 * ground" check (near-field ToF validity when sitting flush is unreliable, and it was
				 * the likely arm blocker) -- just place it down and let it settle. Easy for bench
				 * iteration; NOT glitch-reboot safe -- drop ROSE_ARM_NO_GESTURE for real ops. */
				(void)ground;
				if (!announced) {
					announced = true;
					if (g_arm_enabled) {
						printk("AUTOFLIGHT(no-gesture): arming ENABLED -- place down level + still; auto-arm in %d ms\n",
						       (int)ARM_SETTLE_MS);
					} else {
						printk("AUTOFLIGHT: disarmed on boot -- send RESET from the panel to enable arming\n");
					}
				}
				bool ready = g_arm_enabled                       /* boot-safe: won't arm until a RESET cmd */
					     && level && still && g_gyro_cal_done   /* level + still + gyro-bias cal */
					     && batt_ok_to_arm();                /* refuse to arm on a low pack */
				int64_t hold_ms = ARM_SETTLE_MS;
#else
				/* Arming = LIFT-AND-PLACE gesture (glitch-reboot-safe): pick up past
				 * LIFT_ARM_THRESHOLD_M, then set down level + on-ground + still for PLACE_CONFIRM_MS. */
				static bool lifted;
				if (!announced) {
					announced = true;
					printk("AUTOFLIGHT: lift-and-place to arm -- pick up (>%d mm), set level on "
					       "ground, hold still\n", (int)(LIFT_ARM_THRESHOLD_M * 1000.0f));
				}
				if (f.tof_valid && f.height > LIFT_ARM_THRESHOLD_M && !lifted) {
					lifted = true;
					printk("AUTOFLIGHT: lift detected -- now set level on the ground and hold still\n");
				}
				bool ready = g_arm_enabled                                          /* boot-safe: RESET cmd first */
					     && lifted && level && ground && still && g_gyro_cal_done   /* gesture + gyro cal */
					     && batt_ok_to_arm();                                   /* refuse to arm on a low pack */
				int64_t hold_ms = PLACE_CONFIRM_MS;
#endif
				if (ready) {
					if (arm_since == 0) {
						arm_since = t_now;
					} else if (t_now - arm_since >= hold_ms) {
						g_armed = true;
						g_flight_start_ms = t_now;
						printk("AUTOFLIGHT: ARMED -- taking off (hover %d mm, cap %d ms)\n",
						       (int)(g_hover_z_m * 1000.0f), g_flight_max_ms);
					}
				} else {
					arm_since = 0;   /* condition broke -> restart the hold timer */
				}
				g_arming = (arm_since != 0 && !g_armed);   /* status LED: countdown in progress */
			}
			if (g_armed) {
				int64_t tf = t_now - g_flight_start_ms;
				float zsp = autoflight_setpoint_z(tf);
				/* Landing phase = past the descend ramp (setpoint now LAND_PUSH_M). Cut motors on
				 * ACTUAL touchdown (height-based), not a fixed time, so it never disarms mid-air. */
				bool in_landing = tf >= (int64_t)(g_t_climb_ms + g_t_hover_ms + g_t_descend_ms);
				bool landed = in_landing && f.tof_valid && state[2] < LAND_Z_THRESH_M;
				if (landed || tf >= g_flight_max_ms) {
					g_armed = false;
					flight_done = true;   /* one-shot: don't auto re-arm (reset to fly again) */
					g_setpoint[2] = 0.0f;
					printk("AUTOFLIGHT: %s (%d ms, z=%dmm) -- motors OFF (reset to fly again)\n",
					       landed ? "landed" : "flight cap", (int)tf, (int)(state[2] * 1000.0f));
				} else {
					g_setpoint[2] = zsp;
				}
			}
		}
#endif
#if defined(ROSE_BUMPER) && ROSE_BUMPER
		/* Feed the latest cached wall snapshot into the controller's repulsion term. Non-blocking:
		 * side_tof_get() just copies a mutex-protected struct the background thread updates. Only
		 * apply while armed/flying -- on the ground a lean command would fight the arm gesture. */
		{
			struct side_walls w;
			side_tof_get(&w);
			bool apply = (w.seq != 0);
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
			apply = apply && g_armed;
#endif
			pid_set_walls(w.front_mm, w.back_mm, w.left_mm, w.right_mm, apply);
		}
#endif
		uint32_t _pc = PF_NOW();
		solve_control(state, u, dt);
		PF_ACC(pf_ctrl, _pc);
		uint32_t _ps = PF_NOW();
		send_control(u);
		PF_ACC(pf_send, _ps);
#if defined(CONFIG_WIFI)
		/* Publish the latest state to the WiFi downlink. Non-blocking: just a mutex-guarded struct
		 * copy the telemetry thread drains at 50 Hz -- same fields as the ROSE_TELEM printk line. */
		{
			struct telem_snapshot ts;
			ts.iter = iter;
			ts.dt_ms = (int32_t)(dt * 1000.0f + 0.5f);
			ts.roll = state[3]; ts.pitch = state[4]; ts.yaw = state[5]; ts.z = state[2];
			ts.tof_valid = (int32_t)f.tof_valid; ts.height = f.height;
			ts.u[0] = u[0]; ts.u[1] = u[1]; ts.u[2] = u[2]; ts.u[3] = u[3];
			ts.x = state[0]; ts.y = state[1];
			ts.vx = state[6]; ts.vy = state[7]; ts.vz = state[8];
			ts.zsp = g_setpoint[2]; ts.vbat = g_vbat;
			ts.flags = (uint32_t)((g_armed         ? TELEM_FLAG_ARMED   : 0u) |
					      (g_estop         ? TELEM_FLAG_ESTOP   : 0u) |
					      (g_arming        ? TELEM_FLAG_ARMING  : 0u) |
					      (g_gyro_cal_done ? TELEM_FLAG_CALDONE : 0u));
			telem_wifi_publish(&ts);
		}
#endif
#if defined(ROSE_FLIGHTLOG) && ROSE_FLIGHTLOG
		if (!g_estop && (iter % ROSE_FLIGHTLOG_DIV) == 0) {
			struct flight_rec rec;
			rec.t_ms       = (uint32_t)k_uptime_get();
			rec.roll_mrad  = (int16_t)(state[3] * 1000.0f);
			rec.pitch_mrad = (int16_t)(state[4] * 1000.0f);
			rec.yaw_mrad   = (int16_t)(state[5] * 1000.0f);
			rec.z_mm       = (int16_t)(state[2] * 1000.0f);
			rec.vz_mmps    = (int16_t)(state[8] * 1000.0f);
			rec.vx_mmps    = (int16_t)(state[6] * 1000.0f);   /* est horizontal velocity (flow-fed) */
			rec.vy_mmps    = (int16_t)(state[7] * 1000.0f);
			rec.fvx_mmps   = (int16_t)(f.flow[0] * 1000.0f);  /* raw flow input to the estimator */
			rec.fvy_mmps   = (int16_t)(f.flow[1] * 1000.0f);
			for (int i = 0; i < NACTIONS; i++) {
				float d = u[i] + 0.583f;   /* controller-commanded duty [0,1] (pre-cap) */
				if (d < 0.0f) { d = 0.0f; } else if (d > 1.0f) { d = 1.0f; }
				rec.duty[i] = (uint8_t)(d * 200.0f);   /* 0.5% units */
			}
			rec.flags = (uint8_t)((g_estop ? FLIGHT_FLAG_ESTOP : 0) |
					      (f.tof_valid ? FLIGHT_FLAG_TOF_VALID : 0));
			rec._pad = 0;
			flightlog_write(&rec);
		}
#endif
#if defined(ROSE_PROFILE) && ROSE_PROFILE
		if (++pf_iters >= 30) {
			/* read_sensor_frame was already timed into pf_imu_fetch/get/tof (read total = their
			 * sum). Print avg microseconds per phase over the window, then reset. */
			uint32_t rd = pf_imu_fetch + pf_imu_get + pf_tof + pf_flow;
			printk("PROFILE/%u: loop=%uus read=%uus [imu_fetch=%uus imu_get=%uus tof=%uus x%u flow=%uus] "
			       "est=%uus ctrl=%uus send=%uus\n", pf_iters,
			       k_cyc_to_us_floor32((rd + pf_est + pf_ctrl + pf_send) / pf_iters),
			       k_cyc_to_us_floor32(rd / pf_iters),
			       k_cyc_to_us_floor32(pf_imu_fetch / pf_iters),
			       k_cyc_to_us_floor32(pf_imu_get / pf_iters),
			       pf_tof_n ? k_cyc_to_us_floor32(pf_tof / pf_tof_n) : 0u, pf_tof_n,
			       k_cyc_to_us_floor32(pf_flow / pf_iters),
			       k_cyc_to_us_floor32(pf_est / pf_iters),
			       k_cyc_to_us_floor32(pf_ctrl / pf_iters),
			       k_cyc_to_us_floor32(pf_send / pf_iters));
			pf_imu_fetch = pf_imu_get = pf_tof = pf_tof_n = 0;
			pf_est = pf_ctrl = pf_send = pf_flow = pf_iters = 0;
		}
#endif
#if defined(ROSE_BUMPER_GRID) && ROSE_BUMPER_GRID
		if (0) {   /* grid-validation build: suppress periodic telemetry so GRID lines own the console */
#else
		if (ROSE_TELEM && (iter % 10) == 0) {   /* ROSE_TELEM=0 (flight) -> compiled out, no printk stall */
#endif
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
			       "z=%s%d.%03d tofv=%d tofh=%s%d.%03d u=[%s%d.%03d %s%d.%03d %s%d.%03d %s%d.%03d]\n",
			       iter, (int)(dt * 1000.0f + 0.5f),
			       FP3(state[3]), FP3(state[4]), FP3(state[5]), FP3(state[2]),
			       (int)f.tof_valid, FP3(f.height),
			       FP3(u[0]), FP3(u[1]), FP3(u[2]), FP3(u[3]));
#endif
#if defined(ROSE_BUMPER) && ROSE_BUMPER
			/* Wall distances (mm; -1 = no target/no wall) so bring-up + facing can be verified on
			 * the ground (hand-wave each side) before flight. Repulsion itself is armed-gated above. */
			{
				struct side_walls w;
				side_tof_get(&w);
				printk("  walls[seq=%u]: front=%d back=%d left=%d right=%d\n",
				       w.seq, w.front_mm, w.back_mm, w.left_mm, w.right_mm);
			}
#endif
#if defined(ROSE_FLOW) && ROSE_FLOW
			/* Flow-derived body velocity (m/s) + estimator vx/vy, so bench validation shows the
			 * flow feeding through to the estimated velocity. squal/ok = raw sample quality/gate. */
			{
				float ax, ay; int sq; bool fv;
				flow_get(&ax, &ay, &sq, &fv);   /* ax/ay = RAW angular flow (rad/s), pre-comp */
				/* aRaw = raw angular flow; gyro = body roll/pitch rate (what gyro-comp subtracts);
				 * v = final compensated body velocity (m/s); est = estimator velocity. In-place
				 * rotation: aRaw and gyro swing together, v should stay ~flat if the comp signs fit. */
				printk("  flow: aRaw=[%s%d.%03d %s%d.%03d] gyro=[%s%d.%03d %s%d.%03d] "
				       "v=[%s%d.%03d %s%d.%03d] sq=%d %s | est v=[%s%d.%03d %s%d.%03d]\n",
				       FP3(ax), FP3(ay), FP3(f.gyro[0]), FP3(f.gyro[1]),
				       FP3(f.flow[0]), FP3(f.flow[1]), sq, fv ? "ok" : "--",
				       FP3(state[6]), FP3(state[7]));
			}
#endif
		}
		/* send_control() is issued (and timed) above, before this telemetry print. */
	}
#if defined(ROSE_FLIGHTLOG) && ROSE_FLIGHTLOG
	flightlog_flush();
	printk("flight_controller: flight log flushed to flash (dump with -DROSE_FLIGHTLOG_DUMP=1)\n");
#endif
	printk("flight_controller: control loop done (%d iters)\n", CTRL_ITERS);
	return 0;
#endif
}
