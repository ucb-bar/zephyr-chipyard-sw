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
#define HAVE_ESP_UART DT_NODE_EXISTS(DT_ALIAS(esp_uart))
#if HAVE_ESP_UART
/* The FPGA full-drone shell exposes the ESP link as `esp-uart`. Keep the
 * shared ESP32-C6 build byte-for-byte unchanged: it has no alias, so neither
 * the UART dependency nor this writer exists there. If the ESP UART is already
 * the console (`--mode telem`), printk emits the line and a second copy would
 * exceed the rate budget. */
#define ESP_UART_IS_CONSOLE DT_SAME_NODE(DT_ALIAS(esp_uart), DT_CHOSEN(zephyr_console))
#if !ESP_UART_IS_CONSOLE
#include <zephyr/drivers/uart.h>
#include <stdio.h>

static const struct device *const esp_uart_dev = DEVICE_DT_GET(DT_ALIAS(esp_uart));

static void esp_uart_write_line(const char *line, size_t length)
{
	if (!device_is_ready(esp_uart_dev)) {
		return;
	}

	for (size_t i = 0; i < length; ++i) {
		uart_poll_out(esp_uart_dev, line[i]);
	}
}
#endif /* !ESP_UART_IS_CONSOLE */
#endif /* HAVE_ESP_UART */
/*
 * The D8 status LED hangs off the ADS7128 expander, but every helper it needs -- g_led_bus,
 * ads7128_set_bit/clr_bit, STATUS_LED_CH, ADS7128_GPO_VALUE -- is defined inside the
 * `#if DT_HAS_COMPAT_STATUS_OKAY(st_vl53l1x)` block below, while the LED thread and its start-up
 * call sit at top level. A board with the expander but no VL53L1X declared therefore fails to
 * compile with "'g_led_bus' was not declared in this scope" -- which is what the RiskyBird FPGA
 * carrier does while the down-ToF is still being brought up.
 *
 * Matching the existing condition rather than widening it keeps the ESP build byte-identical:
 * there the VL53L1X is declared, so the LED compiles in exactly as before. Widening this to the
 * expander's own presence is the right long-term fix, but it is a different change from making
 * the file portable.
 */
#define HAVE_STATUS_LED DT_HAS_COMPAT_STATUS_OKAY(st_vl53l1x)

/*
 * Selecting the flow HARDWARE without selecting the flow FEATURE is silent
 * otherwise.
 *
 * A board that names a `flow-spi` alias plainly intends to use the PMW3901, but
 * the real flow path is gated on the ROSE_FLOW CMake knob, not on the alias --
 * so a build that declares the sensor and forgets the knob compiles flow.c,
 * links it, and never calls it. The image looks right and the estimator gets a
 * forced-zero horizontal velocity.
 *
 * Costs nothing when the two agree, and names the missing flag when they do not.
 */
#if DT_NODE_EXISTS(DT_ALIAS(flow_spi)) && !(defined(ROSE_FLOW) && ROSE_FLOW)
#pragma message("flow-spi alias present but ROSE_FLOW=0: optical flow is NOT compiled in. Add -DROSE_FLOW=1 to enable it.")
#endif
#if HAVE_FLOW
#include <rose/rose_sensor.h>   /* RoSE private optical-flow channels */
#endif

/* Control period — MUST match the co-sim rate (gym_timestep = firesim_step/firesim_freq):
 * 0.005 = 200 Hz. The 50 Hz TinyMPC LQR gain is rate-tolerant; running it faster tightens
 * the loop (phase margin for the fast attitude dynamics with the estimator in the loop). */
#define CTRL_DT      0.005f
#define START_Z      0.9f     /* gentle takeoff from near the setpoint */
/* Hover altitude the NON-autoflight builds regulate to, and the reason a bench drone commands
 * full thrust the moment it boots: 1.0 m is a CO-SIM number (the simulated drone starts at
 * START_Z = 0.9 m, so the loop begins 0.1 m from its setpoint). A real drone on a bench starts at
 * 0.02 m, which is a 0.98 m step error applied at t=0 with no climb ramp -- the altitude loop
 * saturates on the first iteration and never leaves saturation. #ifndef-guarded and wired to the
 * build (CMakeLists TARGET_Z) so the bench can lower it -- -DTARGET_Z=0.0f makes a stationary
 * drone regulate to where it actually is -- without editing this file. Default unchanged. */
#ifndef TARGET_Z
#define TARGET_Z     1.0f
#endif
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
/*
 * Print one telemetry line every ROSE_TELEM_DIV iterations.
 *
 * Was hardcoded to 10, which is ~100 lines/s at a 1 kHz loop. That is free on a USB console but
 * not on a real UART: ~150 chars/line x 100 lines/s is ~150 kbps, more than a 115200 link can
 * carry, so printk blocks and the control loop inherits the console's backlog -- the loop then
 * measures its own dt as the UART's, not the controller's. The RiskyBird FPGA carrier's console
 * is a 115200 FTDI link, so it needs roughly 200 here for a ~5 Hz line.
 *
 * A divisor rather than a faster console because losing the console loses all observability,
 * while a slower telemetry line costs nothing during bring-up.
 */
#ifndef ROSE_TELEM_DIV
#define ROSE_TELEM_DIV 10
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

/*
 * The v1.1 telemetry tail -- position, velocity, setpoint, battery and state.
 *
 * The ground station renders its state banner, 3D position view, drift arrow
 * and battery gauge from these; riskybird_panel.py parses them with TAIL_RE
 * (vx vy vz zsp vbat st, in that order) and POS_RE (x y). Without them the
 * dashboard has attitude and altitude and nothing else, which is what the FPGA
 * build showed.
 *
 * Every value here is ALREADY computed each iteration -- the CONFIG_WIFI block
 * below packs exactly these into a telem_snapshot. On the FPGA that block is
 * compiled out (the radio is a UART away, not on-chip) and telem_wifi.c is not
 * in this checkout at all, so the values were being computed and discarded.
 * This only prints what the loop already knows; it adds no estimation work.
 *
 * Defined once and used by both the printk and the esp-uart mirror below, so
 * the tethered console and the radio cannot drift into different formats.
 */
#define TELEM_TAIL_FMT \
	" x=%s%d.%03d y=%s%d.%03d vx=%s%d.%03d vy=%s%d.%03d vz=%s%d.%03d " \
	"zsp=%s%d.%03d vbat=%s%d.%03d st=%u"

/*
 * Flag bits, matching riskybird_panel.py's FLAG_ARMED/ESTOP/ARMING/CALDONE.
 * Spelled out rather than taken from telem_wifi.h, which this checkout does
 * not have -- the same reason the CONFIG_WIFI path cannot be relied on here.
 */
#define ROSE_TELEM_FLAG_ARMED   1u
#define ROSE_TELEM_FLAG_ESTOP   2u
#define ROSE_TELEM_FLAG_ARMING  4u
#define ROSE_TELEM_FLAG_CALDONE 8u

#define TELEM_TAIL_ARGS \
	FP3(state[0]), FP3(state[1]), \
	FP3(state[6]), FP3(state[7]), FP3(state[8]), \
	FP3(g_setpoint[2]), FP3(g_vbat), \
	(unsigned int)((g_armed         ? ROSE_TELEM_FLAG_ARMED   : 0u) | \
		       (g_estop         ? ROSE_TELEM_FLAG_ESTOP   : 0u) | \
		       (g_arming        ? ROSE_TELEM_FLAG_ARMING  : 0u) | \
		       (g_gyro_cal_done ? ROSE_TELEM_FLAG_CALDONE : 0u))

/*
 * THE WHOLE TELEMETRY LINE, in one place.
 *
 * Two independent lines of work both wrote this string and both had to be
 * kept: the v1.1 state tail above (x/y/vx/vy/vz/zsp/vbat/st, which is what the
 * ground station's banner, 3D view, drift arrow and battery gauge are parsed
 * out of) and the safety guard's own fields (tilt in DEGREES, |a|, accskip --
 * see the note at the print site: roll/pitch/yaw are Rodrigues parameters, not
 * radians, so tilt= is the only honest angle on the line).
 *
 * They are combined rather than chosen between, and in this order:
 *
 *   base ... duty=[...]   <- v1 core, unchanged
 *   TELEM_TAIL_FMT        <- v1.1 tail, in the position riskybird_panel.py's
 *                            TAIL_RE/POS_RE already expect it
 *   tilt= |a|= accskip=   <- appended last, so every existing parser keeps
 *                            working and the new fields are additive
 *
 * Defined once, as a format/arguments PAIR, and used by both the printk that
 * feeds the tethered console and the esp-uart mirror that feeds the radio --
 * which is the point: two copies of a 15-line format string is how the console
 * and the radio drift into different formats, and the merge that produced this
 * file started with exactly that duplication.
 *
 * The argument list names loop locals (iter, dt, state, f, u), so it is usable
 * only inside the control loop. That is deliberate; there is nowhere else this
 * line should be emitted from.
 */
#define TELEM_LINE_FMT \
	"flight_controller: it=%d dt=%dms roll=%s%d.%03d pitch=%s%d.%03d yaw=%s%d.%03d " \
	"z=%s%d.%03d tofv=%d tofh=%s%d.%03d u=[%s%d.%03d %s%d.%03d %s%d.%03d %s%d.%03d] " \
	"duty=[%s%d.%03d %s%d.%03d %s%d.%03d %s%d.%03d]" \
	TELEM_TAIL_FMT \
	" tilt=est%ddeg/acc%ddeg |a|=%s%d.%03d accskip=%u\n"

#define TELEM_LINE_ARGS \
	iter, (int)(dt * 1000.0f + 0.5f), \
	FP3(state[3]), FP3(state[4]), FP3(state[5]), FP3(state[2]), \
	(int)f.tof_valid, FP3(f.height), \
	FP3(u[0]), FP3(u[1]), FP3(u[2]), FP3(u[3]), \
	FP3(g_last_duty[0]), FP3(g_last_duty[1]), \
	FP3(g_last_duty[2]), FP3(g_last_duty[3]), \
	TELEM_TAIL_ARGS, \
	(int)(g_guard_est_rad  * (180.0f / 3.14159265f) + 0.5f), \
	(int)(g_guard_tilt_rad * (180.0f / 3.14159265f) + 0.5f), \
	FP3(g_guard_amag), (unsigned)g_guard_acc_skips

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

/* ===============================================================================================
 * THE 2026-09-21 TILT-GUARD FAILURE, AND WHAT WAS ACTUALLY WRONG
 * ===============================================================================================
 * Measured on the bench: the drone was rotated by hand to roughly 90 degrees while
 * `--mode esp-motors` was running, and nothing cut the motors. The console showed
 *
 *     it=1020  roll=-0.949  pitch=-0.029  duty=[0.100 0.100 0.100 0.100]
 *
 * and the ESP kept reporting flags=0x01 (ARMED) the whole way over.
 *
 * The estimator was NOT lagging and NOT mis-scaled. It was right, and it was MISREAD.
 *
 * state[3..5] are RODRIGUES (Gibbs) parameters, r = q_xyz / qw -- see estimator.hpp, and
 * ComplementaryEstimator::get_state() / EkfEstimator::get_state(), both of which divide the
 * quaternion vector part by qw. For a rotation of theta about one axis that is
 *
 *     r = tan(theta / 2)
 *
 * so the reported -0.949 is a true roll of 2*atan(0.949) = 1.518 rad = 87.0 DEGREES. That is the
 * ~90 degrees the operator applied, to within how well anyone eyeballs 90 degrees. The estimator
 * tracked the rotation correctly.
 *
 * What failed is that safety_violation() compared that Gibbs number against a constant named
 * SAFE_MAX_TILT_RAD as though it were radians. 1.0 in Gibbs units is tan(theta/2) = 1, i.e.
 * theta = 90.0 DEGREES EXACTLY -- not the ~57 degrees its own comment claimed. The guard's real
 * trip point was 90 degrees and the operator reached 87. It missed by three degrees, and the
 * comment had been describing a limit the code did not have since the constant was written.
 *
 * The same misreading was in the arm gate: ARM_MAX_TILT_RAD 0.10 "(~5.7 deg)" is really
 * 2*atan(0.1) = 11.4 degrees.
 *
 * The codebase already knew, in one place. read_sensor_frame()'s flow tilt-compensation derives
 * cos(tilt) from "the Gibbs state" with (1 - ga^2 - gb^2 + gc^2)/(1 + ga^2 + gb^2 + gc^2); put
 * ga = 0.949 through it and it returns 0.052, which is cos(87.0 deg). Two independent routes to
 * the same answer, so this is not a guess.
 *
 * THE FIX IS IN THREE PARTS, because one would not have been enough:
 *
 *   1. Convert. tilt_rad_from_gibbs() turns the state into actual radians, once, and every
 *      attitude limit now compares radians against radians. The constants finally mean what they
 *      are named.
 *   2. Do not depend on the estimator at all for the handling case. accel_envelope() reads the
 *      accelerometer directly -- gravity is a true attitude reference with no filter, no
 *      convergence and no parameterisation to misread -- and adds FREE FALL and IMPACT, which
 *      NOTHING in this file had. Tilt cannot catch a drop: a drone dropped flat stays flat the
 *      whole way down and only |a| gives it away. The approach and the thresholds come from
 *      integration/zephyr/safety/motor_guard.{h,c}, which is proven to compile and run on the
 *      bench workloads; see the note on accel_envelope() for why the module is not linked here.
 *   3. Debounce in MILLISECONDS, not iterations. SAFE_DEBOUNCE_ITERS was 15 consecutive iterations,
 *      which is 20 ms under the PID cascade (1.3 ms/iter) and 525 ms under TinyMPC (35 ms/iter) --
 *      a factor of TWENTY-SIX between two builds of the same guard, and the offload run that
 *      failed was the slow one. A brisk tilt-and-return cannot outlast half a second. Time is the
 *      unit that means the same thing in both builds.
 *
 * WHAT THE ACCELEROMETER GUARD IS AND IS NOT GOOD AT, stated so nobody over-trusts it. It reads
 * SPECIFIC FORCE. Sitting still or hovering, that is gravity and the tilt it reports is true. In a
 * hard translational acceleration the thrust vector dominates and body-frame horizontal accel
 * stays small even at a real tilt, so it UNDER-reports during aggressive flight -- which is
 * exactly where the estimator-based check is strong. They are a complementary pair on purpose, and
 * the direction test is skipped entirely while |a| is outside a plausible band, because the
 * direction of a vector that is not gravity says nothing about attitude.
 * =============================================================================================== */

/* Rodrigues/Gibbs magnitude of the roll+pitch part -> true tilt angle from vertical, in radians.
 * Exact for a single-axis rotation; for a combined roll/pitch it is the half-angle of the combined
 * rotation, which is what an envelope wants anyway. Saturates gracefully: r -> inf as theta -> 180,
 * and atanf() simply approaches pi/2, so a tumbling frame reads pi rather than wrapping. */
static inline float tilt_rad_from_gibbs(const float *state)
{
	return 2.0f * atanf(sqrtf(state[3] * state[3] + state[4] * state[4]));
}

#ifndef SAFE_MAX_TILT_RAD
/* 0.70 rad = 40 deg, and now genuinely 40 deg. The old 1.0 was documented as 57 deg and behaved as
 * 90 deg; a hover test that reaches 40 deg at 300 mm is already not going to be recovered, and a
 * frame being picked up passes 40 deg long before it passes 90. */
#define SAFE_MAX_TILT_RAD   0.70f
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

/* ---- accelerometer envelope (no estimator in the path) --------------------------------------
 * Thresholds carried over from integration/zephyr/safety/motor_guard.h so the two guards can be
 * compared and tuned against each other. That module states tilt as sin^2(theta) in percent to stay
 * integer-only for FPU-less builds; this file is float throughout, so the same limit is written as
 * an angle and squared once at compile time. 40 deg here is sin^2 = 41%, against the module's
 * bench default of 50% (45 deg) -- tighter, because this guard runs while motors may be driving. */
#ifndef SAFE_ACC_TILT_RAD
#define SAFE_ACC_TILT_RAD   0.70f    /* 40 deg, measured against gravity directly */
#endif
#ifndef SAFE_FREEFALL_MPS2
#define SAFE_FREEFALL_MPS2  3.92f    /* |a| below ~0.4 g: the airframe is falling */
#endif
#ifndef SAFE_IMPACT_MPS2
#define SAFE_IMPACT_MPS2    29.43f   /* |a| above ~3 g: it hit something */
#endif
/* Trust the accelerometer's DIRECTION only inside this band. Outside it, |a| is not gravity and the
 * two magnitude tests above own the case.
 *
 * WHETHER TO WIDEN THIS WAS THE HARDEST CALL IN THE 2026-09-21 SAFETY AUDIT, because this band is
 * the one place a test in this guard can switch itself off, and prop vibration is exactly what
 * pushes |a| out of it. The decision is NOT TO WIDEN. The reasoning, in full, so that the next
 * person to reach for these numbers argues with it rather than guessing at it:
 *
 * WHAT IS STILL TRUE OUTSIDE THE BAND. Free-fall and impact are magnitude tests evaluated BEFORE
 * this check, so they never stop. est tilt, rate, velocity and height are computed after this
 * returns, from the estimator and the gyro, and never touch this path. Exactly ONE of the seven
 * tests suspends -- the accelerometer's DIRECTION -- and the estimator's tilt covers the same
 * failure by a different route. That is not a hole; it is a degradation to six tests from seven.
 * On the bench it was the estimator test that actually fired ("47.457 deg est-tilt") precisely
 * BECAUSE hand-tilting the frame had pushed |a| out of this band. The pair works.
 *
 * WHY WIDENING WOULD NOT BUY WHAT IT LOOKS LIKE IT BUYS. Outside the band |a| is not gravity, so
 * asin(|horiz|/|a|) is not attitude. Under thrust- or vibration-dominated |a| the horizontal part
 * stays small relative to a LARGER total, so the formula UNDER-reports the tilt. A widened band
 * therefore does not gain a test that trips; it gains a test that reads a plausible small number
 * and does not trip. Against a limit, "absent" and "silently under-reporting" fail identically.
 *
 * WHY WIDENING WOULD ACTIVELY COST SOMETHING. g_guard_tilt_rad is what the telemetry line prints
 * as accNdeg, and the whole pre-flight proof that the guard is alive is "tilt the frame by hand and
 * watch acc follow" -- see docs/tethered-flight-attempt.md. Suspended, it publishes pi, which
 * prints as acc180deg: an unmistakable "I do not know". Widened, it would publish a believable
 * wrong angle instead. Trading an honest refusal for a plausible fiction is the wrong trade in a
 * device whose previous failure was a guard that looked like it was working.
 *
 * WHAT THE MEASUREMENT SAYS. Props ON, on the bench: |a| ranged 7.85 .. 12.46 m/s^2 and 0 of 44
 * samples fell outside this band. It was not being hit. The honest caveat is that this was at the
 * 10 % bench cap, and nobody has sampled |a| at the 65-75 % duty a tethered hover needs.
 *
 * SO INSTEAD OF WIDENING, MAKE IT COUNTABLE. g_guard_acc_skips counts every sample the direction
 * test sits out. At rest it stays 0; if it climbs with props spinning, the accelerometer test is
 * effectively off and the estimator test is carrying the attitude envelope alone -- which is a
 * thing to know and act on, not a thing to hide behind a wider band. The telemetry line and
 * hardware/flight/guard-check.gdb both report it. */
#ifndef SAFE_ACC_TRUST_LO_MPS2
#define SAFE_ACC_TRUST_LO_MPS2 5.89f   /* 0.6 g */
#endif
#ifndef SAFE_ACC_TRUST_HI_MPS2
#define SAFE_ACC_TRUST_HI_MPS2 14.72f  /* 1.5 g */
#endif

/* ---- debounce, in milliseconds ---------------------------------------------------------------
 * A limit must be exceeded continuously for this long, AND across at least
 * SAFE_DEBOUNCE_MIN_SAMPLES samples, before the estop latches. The sample floor is what rejects a
 * single garbage reading on a fast loop; the time is what makes the guard behave identically under
 * the PID cascade (~1.3 ms/iter) and TinyMPC (~35 ms/iter).
 *
 * 120 ms for attitude/rate/velocity/height/battery: long enough to ride out a spin-up vibration
 * spike, short enough that a deliberate tilt-and-return is caught -- the failed run crossed and
 * returned inside about 600 ms, so the old 375-525 ms could not have caught it either.
 * 40 ms for free fall and impact: those are not transients to be ridden out. A 40 ms fall is 8 mm.
 */
#ifndef SAFE_DEBOUNCE_MS
#define SAFE_DEBOUNCE_MS       120
#endif
#ifndef SAFE_SHOCK_DEBOUNCE_MS
#define SAFE_SHOCK_DEBOUNCE_MS 40
#endif
#ifndef SAFE_DEBOUNCE_MIN_SAMPLES
#define SAFE_DEBOUNCE_MIN_SAMPLES 2
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
/* TRUE radians, compared against tilt_rad_from_gibbs(), not against the raw Gibbs parameter.
 * The old 0.10 was commented "~5.7 deg" and, read as Gibbs, was really 2*atan(0.10) = 11.4 deg --
 * the same units bug that let the safety envelope sit at 90 deg while claiming 57. 0.20 rad is
 * 11.5 deg, so the gate keeps the angle it has actually been enforcing all along and only its name
 * changes. Tighten it deliberately if the bench wants a flatter start; do not tighten it by
 * accident, because arming that cannot complete is its own kind of failure. */
#define ARM_MAX_TILT_RAD    0.20f   /* 11.5 deg, and now genuinely 11.5 deg */
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
/*
 * Per-motor duty ceiling in flight. READ THIS BEFORE BELIEVING THE DEFAULT.
 *
 * The comment that used to sit on this line said "hover~0.15; margin above". That is wrong, and
 * it is wrong in the dangerous direction. 0.15 is MOTOR_BREAKAWAY_DUTY, ~90 lines down: the duty
 * at which a motor first turns at all. It was evidently copied here and nothing about this
 * airframe supports it as a hover figure. There are three numbers in this tree claiming to be
 * hover duty and they differ by about 5x:
 *
 *   0.583  the CONTROLLER'S LINEARISATION POINT, not a measurement. It is the +0.583f in
 *          actuator_duty() and the box TinympcController::init() builds around it
 *          (u_min -0.583, u_max 1-0.583). It says what the controller ASSUMES hover is.
 *   0.71   MEASURED IN FLIGHT. docs/FLIGHT_TUNING_LOG.md, platform line: "~71 % hover duty, raw
 *          command saturating 42-65 % of the flight". Those flights ran with
 *          AUTOFLIGHT_MAX_DUTY=0.95 (same file) and docs/FLIGHT_BUILD.md's recipe passes 0.8f
 *          with the note "hover needs ~65%".
 *   0.15   the copied break-away number. Not a hover duty at all.
 *
 * So 0.45 IS PROBABLY BELOW HOVER, and a ceiling below hover does not make a gentle flight: the
 * anti-saturation cut holds the collective at the ceiling, the drone sits on the ground at full
 * command, and the honest-looking console shows every motor pinned. The reaction that costs an
 * airframe is to assume something is broken and start raising limits in a hurry. Raise this
 * deliberately, in one step, having decided the number first -- 0.8f is what the recorded flight
 * recipe used.
 *
 * The default is left at 0.45 rather than raised here: it is the conservative direction, it is
 * what every existing autoflight build has been tested against, and a flight cap is not
 * something a file edit should hand out. The #pragma below makes the situation impossible to
 * miss at build time instead. NOTE this knob is the AUTOFLIGHT path only; --mode esp-motors is a
 * non-autoflight build and is capped by MOTOR_MAX_DUTY (a scale) plus the ESP's own ceiling.
 */
#ifndef AUTOFLIGHT_MAX_DUTY
#define AUTOFLIGHT_MAX_DUTY 0.45f
#endif
/* The comparison against the measured hover duty is made at RUN TIME, in safety_banner(), not
 * here: `#if` cannot compare floats (a float in a preprocessor expression is an error, not a
 * comparison), and a line in a build log is read once while the boot banner is read at the
 * bench every time the board comes up. */
#endif /* ROSE_AUTOFLIGHT */

/* Returns a short reason if any safety limit is exceeded (state = 12-DoF body state, gyro = body
 * rates rad/s), else NULL. Attitude from the estimate; rate straight from the gyro (no filter lag);
 * velocity from the estimate. */
/* Live guard view, published every iteration so telemetry and a debugger can see WHAT THE GUARD
 * SEES. Without these, "it did not trip" and "it is not looking" produce identical output, which is
 * the one thing a safety device must never do -- and is precisely how the 90-degree miss went
 * unnoticed until someone rotated the drone by hand. */
static volatile float g_guard_tilt_rad;   /* accelerometer tilt from vertical (rad) */
static volatile float g_guard_est_rad;    /* estimator tilt from vertical (rad), Gibbs converted */
static volatile float g_guard_amag;       /* |accel| (m/s^2) */
/* How many samples the accel DIRECTION test has sat out because |a| left the trust band. The one
 * test in this guard that can switch itself off must be countable, or "it never tripped" and "it
 * was not running" produce the same console again -- see the note on the trust band. */
static volatile uint32_t g_guard_acc_skips;

/*
 * ACCELEROMETER ENVELOPE -- the estimator is deliberately not in this path.
 *
 * Reuses the tests and the thresholds from integration/zephyr/safety/motor_guard.{h,c}. It is NOT
 * linked in here, and the reason is worth writing down: that module runs its own cooperative thread
 * that calls sensor_sample_fetch() on the BMI088 and zeroes an array of pwm_dt_spec on a trip.
 * Neither fits this application. The control loop already fetches the same accelerometer every
 * iteration, and a second thread fetching it concurrently would contend for the same bus behind a
 * driver that does not expect two samplers; and on the ESP-offload path there are NO pwm_dt_spec
 * motors at all -- the FPGA drives no gate -- so a guard that stops motors by writing PWM would be
 * a guard that does nothing. Here the trip latches g_estop instead, which both actuator backends
 * already honour, and which on the offload path transmits an explicit zero-duty ESTOP frame on the
 * very next tick because a flags change bypasses the frame rate limiter.
 *
 * Returns a reason or NULL. *need_ms is how long THIS reason must persist, *meas / *unit are the
 * number that tripped it, for the console.
 */
static const char *accel_envelope(const float *accel, int *need_ms, float *meas, const char **unit)
{
	const float ax = accel[0], ay = accel[1], az = accel[2];
	const float horiz2 = ax * ax + ay * ay;
	const float total2 = horiz2 + az * az;
	const float amag = sqrtf(total2);

	g_guard_amag = amag;

	/* Free fall first: it is the most urgent and the only one tilt can never see. A drone dropped
	 * flat stays flat the whole way down. */
	if (total2 < (SAFE_FREEFALL_MPS2) * (SAFE_FREEFALL_MPS2)) {
		*need_ms = SAFE_SHOCK_DEBOUNCE_MS;
		*meas = amag; *unit = "m/s2 |a|";
		return "free-fall";
	}
	if (total2 > (SAFE_IMPACT_MPS2) * (SAFE_IMPACT_MPS2)) {
		*need_ms = SAFE_SHOCK_DEBOUNCE_MS;
		*meas = amag; *unit = "m/s2 |a|";
		return "impact";
	}
	/* Direction is attitude only while the magnitude is plausibly gravity. Outside the band the
	 * two tests above own the case and this one stays quiet rather than guessing. */
	if (amag >= (SAFE_ACC_TRUST_LO_MPS2) && amag <= (SAFE_ACC_TRUST_HI_MPS2)) {
		/* tilt from vertical, straight out of gravity: sin(theta) = |horizontal| / |a|. */
		const float tilt = asinf(sqrtf(horiz2 / total2));

		g_guard_tilt_rad = tilt;
		if (tilt > (SAFE_ACC_TILT_RAD)) {
			*need_ms = SAFE_DEBOUNCE_MS;
			*meas = tilt * (180.0f / 3.14159265f); *unit = "deg accel-tilt";
			return "accel-tilt";
		}
	} else {
		/* Out of the trust band: publish "unknown, assume the worst" rather than leaving the
		 * last good value in place. The arm gate reads this, and a stale level reading is the
		 * one way a cross-check can silently permit what it was added to forbid. */
		g_guard_tilt_rad = 3.14159265f;
		g_guard_acc_skips++;
	}
	return NULL;
}

/*
 * Full envelope: the accelerometer tests above, then the estimator-based ones.
 *
 * Attitude is compared in REAL RADIANS now. state[3..4] are Rodrigues parameters, r = tan(theta/2)
 * per axis -- comparing them directly against a constant named *_RAD is the bug that let a
 * 87-degree hand rotation sit under a limit everyone believed was 57 degrees and was actually 90.
 * See the long note above the thresholds.
 */
static const char *safety_violation(const float *state, const float *gyro, const float *accel,
				    int *need_ms, float *meas, const char **unit)
{
	const char *why;
	float tilt;

	*need_ms = SAFE_DEBOUNCE_MS;
	*meas = 0.0f;
	*unit = "";

	why = accel_envelope(accel, need_ms, meas, unit);
	if (why != NULL) {
		return why;
	}
	*need_ms = SAFE_DEBOUNCE_MS;

	tilt = tilt_rad_from_gibbs(state);
	g_guard_est_rad = tilt;
	if (tilt > SAFE_MAX_TILT_RAD) {
		*meas = tilt * (180.0f / 3.14159265f); *unit = "deg est-tilt";
		return "tilt";
	}
	if (fabsf(gyro[0]) > SAFE_MAX_RATE_RADPS || fabsf(gyro[1]) > SAFE_MAX_RATE_RADPS ||
	    fabsf(gyro[2]) > SAFE_MAX_RATE_RADPS) {
		float m = fabsf(gyro[0]);

		if (fabsf(gyro[1]) > m) m = fabsf(gyro[1]);
		if (fabsf(gyro[2]) > m) m = fabsf(gyro[2]);
		*meas = m; *unit = "rad/s";
		return "rate";
	}
	if (fabsf(state[6]) > SAFE_MAX_VEL_MPS || fabsf(state[7]) > SAFE_MAX_VEL_MPS ||
	    fabsf(state[8]) > SAFE_MAX_VEL_MPS) {
		float m = fabsf(state[6]);

		if (fabsf(state[7]) > m) m = fabsf(state[7]);
		if (fabsf(state[8]) > m) m = fabsf(state[8]);
		*meas = m; *unit = "m/s";
		return "velocity";
	}
	if (state[2] > SAFE_MAX_HEIGHT_M) {   /* z = altitude (up); one-sided ceiling guard */
		*meas = state[2]; *unit = "m";
		return "height";
	}
#if ROSE_BATT_SENSE
	/* Low-voltage cutoff: only a VALID reading (>= 1.0 V) below the threshold trips -- a garbage-low
	 * read (< 1.0 V, sensor fault) is ignored so it can't false-estop mid-flight. Debounced by the
	 * caller like every other reason. */
	if (g_vbat >= 1.0f && g_vbat < BATT_CUTOFF_V) {
		*meas = g_vbat; *unit = "V";
		return "battery";
	}
#endif
	return NULL;
}

/* Say what the envelope IS, at boot, in units a human can check against a protractor. A guard
 * nobody can read is a guard nobody can verify, and this one was wrong for as long as it has
 * existed precisely because its numbers were only ever printed as source comments. */
static void safety_banner(void)
{
	printk("flight_controller: ENVELOPE GUARD -- accel tilt >%d deg, free-fall <%s%d.%03d m/s2, "
	       "impact >%s%d.%03d m/s2 (no estimator in that path)\n",
	       (int)((SAFE_ACC_TILT_RAD) * (180.0f / 3.14159265f) + 0.5f),
	       FP3((float)(SAFE_FREEFALL_MPS2)), FP3((float)(SAFE_IMPACT_MPS2)));
	printk("flight_controller: ENVELOPE GUARD -- est tilt >%d deg, rate >%d rad/s, vel >%s%d.%03d m/s, "
	       "height >%s%d.%03d m\n",
	       (int)((SAFE_MAX_TILT_RAD) * (180.0f / 3.14159265f) + 0.5f),
	       (int)(SAFE_MAX_RATE_RADPS),
	       FP3((float)(SAFE_MAX_VEL_MPS)), FP3((float)(SAFE_MAX_HEIGHT_M)));
	printk("flight_controller: ENVELOPE GUARD -- dwell %d ms (%d ms for free-fall/impact), "
	       "min %d samples; ACTIVE ONLY WHILE ARMED\n",
	       (int)(SAFE_DEBOUNCE_MS), (int)(SAFE_SHOCK_DEBOUNCE_MS),
	       (int)(SAFE_DEBOUNCE_MIN_SAMPLES));
	/* "ACTIVE ONLY WHILE ARMED" is true and is read as narrower than it is, so say what armed
	 * MEANS in this build. In every non-autoflight mode -- which is every mode that will be
	 * flown on the offload path -- g_armed is a compile-time `true` (see its definition), so the
	 * guard latches from the first control tick to the last and there is no unarmed window. */
	printk("flight_controller: ENVELOPE GUARD -- armed=%s in this build, so the latch is %s\n",
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	       "the run-time arm gate", "live only after arming");
#else
	       "COMPILE-TIME TRUE", "live from the first control tick");
#endif
	/* The one test that can suspend itself. accel_envelope() skips the DIRECTION check while
	 * |a| is outside the trust band, because the direction of a vector that is not gravity says
	 * nothing about attitude -- and prop vibration is exactly what pushes |a| out of that band.
	 * It is visible, not silent: g_guard_tilt_rad is published as pi, which the telemetry line
	 * prints as acc180deg. Free-fall, impact, est-tilt, rate, velocity and height are unaffected
	 * and keep running; they are tested before the band, or do not use the accelerometer at all. */
	printk("flight_controller: ENVELOPE GUARD -- accel-tilt test is SUSPENDED while |a| is "
	       "outside %s%d.%03d..%s%d.%03d m/s2; telemetry then reads acc180deg and counts it in "
	       "accskip=. The other six tests keep running.\n",
	       FP3((float)(SAFE_ACC_TRUST_LO_MPS2)), FP3((float)(SAFE_ACC_TRUST_HI_MPS2)));
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	/* Checked at run time because `#if` cannot compare floats. 0.71 is the only MEASURED hover
	 * duty this tree has (docs/FLIGHT_TUNING_LOG.md); a ceiling below it is a safe failure that
	 * looks exactly like broken hardware, and the reaction that costs an airframe is to start
	 * raising limits in a hurry. Say it here instead. */
	if ((float)(AUTOFLIGHT_MAX_DUTY) < 0.71f) {
		printk("flight_controller: *** AUTOFLIGHT_MAX_DUTY %s%d.%03d IS BELOW THE MEASURED "
		       "0.71 HOVER DUTY *** -- this build may be unable to leave the ground. That is "
		       "a SAFE failure. Do not react to it by raising limits in a hurry.\n",
		       FP3((float)(AUTOFLIGHT_MAX_DUTY)));
	}
#endif
}

/* ---- COMMITMENT GATE -------------------------------------------------------------------------
 *
 * THE REQUIREMENT, in the operator's words: "make sure we have reliable ways of stopping and
 * motors dont spin for too long until we are COMMITTED (meaning im actively aware) to a flight
 * test." Two separate things: stopping must be reliable, and an UNCOMMITTED build must not be
 * able to run motors for long. Everything else in this file -- estop, the arm gate, the duty
 * ceilings, the bench actuation timeout -- answers the first. This answers the second.
 *
 * WHY IT IS NEEDED AT ALL. In every non-autoflight mode g_armed is a compile-time `true`, so a
 * board that boots `--mode motors` or `--mode esp-motors` is armed from its first control tick.
 * The controller regulates to TARGET_Z, so it asks for thrust immediately and keeps asking for as
 * long as the loop runs -- CTRL_ITERS iterations, or forever with CTRL_ITERS=0. The only bound was
 * ROSE_ACTUATE_TIMEOUT_MS, which DEFAULTS TO 0 = never (see the CMake cache entry), so the default
 * build had no time bound on motor output whatsoever.
 *
 * THE DEFAULT. Motors may run for ROSE_MOTOR_WINDOW_MS (5 s) measured from the first instant a
 * non-zero duty would actually have been written. Then all four are commanded to 0 and LATCHED --
 * whatever the controller asks for afterwards, and whatever the arm gate says. 5 s is chosen to be
 * long enough to hear each motor spin up and see a duty in telemetry, and far too short to fly.
 *
 * COMMITTING, and why it cannot happen by accident. It takes TWO independent acts:
 *
 *   1. BUILD TIME  -DROSE_FLIGHT_COMMIT=1. Deliberately NOT selected by any --mode (see
 *      tools/rb/boards.py): no mode name can quietly imply it, so it has to be typed, as
 *      RB_CMAKE_ARGS="-DROSE_FLIGHT_COMMIT=1". Without it, g_flight_commit_key below is not
 *      compiled at all -- so the run-time act is not merely refused, the symbol does not exist and
 *      the GDB script fails with "No symbol". That absence is the proof the image cannot fly.
 *
 *   2. RUN TIME    the operator writes ROSE_COMMIT_KEY into g_flight_commit_key from a tethered
 *      debugger (hardware/flight/commit.gdb), on the bench, with the drone in front of them. It is
 *      a specific 32-bit constant, so neither uninitialised RAM, nor a stray `set var x = 1`, nor a
 *      wild store can produce it; and it is polled, not latched at boot, so it cannot survive into
 *      a later session.
 *
 * Neither act alone does anything. A committed image that is never keyed behaves exactly like the
 * default. A key written into an uncommitted image has nowhere to land.
 *
 * Committed, the window becomes ROSE_COMMIT_MAX_MS (60 s) rather than unbounded: a commitment is
 * permission for A FLIGHT TEST, not permission forever. FLIGHT_MAX_MS (10 s) still ends an
 * autoflight long before this, so the 60 s is the backstop for the backstop.
 *
 * ANNOUNCED, ALWAYS. "Actively aware" is the requirement, so every transition prints: the banner at
 * boot, the window opening on the first live duty, a 1 Hz heartbeat while motors may run, the
 * latch, the commit being accepted or refused, and a soft reset that does NOT clear the latch. A
 * state the console does not name is a state the operator is not aware of.
 *
 * WHAT THE GATE DOES NOT COVER, said plainly: the autoflight boot/ready chirps drive PWM directly
 * rather than through send_control(), so they do not open the window. They are bounded by
 * construction (MOTOR_CHIRP_MS per pulse, six pulses) and are the signal that the board reset,
 * which is itself a safety feature. There is no chirp at all on the ESP-offload path.
 */
#ifndef ROSE_FLIGHT_COMMIT
#define ROSE_FLIGHT_COMMIT 0
#endif
#ifndef ROSE_MOTOR_WINDOW_MS
#define ROSE_MOTOR_WINDOW_MS 5000    /* uncommitted: motor-live time before the latch */
#endif
#ifndef ROSE_COMMIT_MAX_MS
#define ROSE_COMMIT_MAX_MS 60000     /* committed: still bounded, just usefully longer */
#endif
#ifndef ROSE_MOTOR_LIVE_NOTE_MS
#define ROSE_MOTOR_LIVE_NOTE_MS 1000 /* heartbeat cadence while the window is open */
#endif
/* "FLY!" -- chosen so the value is recognisable in a memory dump and impossible to hit by
 * accident. Do not make it 1, and do not make it derivable from anything the program computes. */
#define ROSE_COMMIT_KEY 0x464C5921u

/* ---- Actuator output: RoSE bridge (co-sim) vs PWM motors (real) ---- */
#define HAVE_ROSE DT_HAS_COMPAT_STATUS_OKAY(ucbbar_roseadapter)

#if HAVE_ROSE
/* Co-sim drives no physical motor, so there is nothing here to time-bound: a simulated flight that
 * cut out after 5 s would be a broken simulation, not a safe one. Stubs, named so the call sites
 * below read identically on both targets. */
static inline void motor_gate_banner(void) { }
static inline bool motor_gate_permit(const float *duty) { (void)duty; return true; }
static inline bool motor_gate_committed(void) { return false; }
static inline void motor_gate_note_reset(void) { }
#else
#if ROSE_FLIGHT_COMMIT
/* NOT static, and volatile: the operator writes it from GDB, nothing in the image ever writes it,
 * and --gc-sections must not drop a variable whose only writer is a human. Kept out of the image
 * entirely on an uncommitted build -- that is the build-time half of the interlock. */
volatile uint32_t g_flight_commit_key;
#endif
static volatile bool    g_motor_committed;    /* the run-time key was accepted */
static volatile bool    g_motor_latched;      /* window expired: OFF until a board reset */
static volatile int64_t g_motor_live_since;   /* 0 = no non-zero duty has been commanded yet */

static inline bool motor_gate_committed(void) { return g_motor_committed; }

static inline int motor_gate_limit_ms(void)
{
	return g_motor_committed ? (int)(ROSE_COMMIT_MAX_MS) : (int)(ROSE_MOTOR_WINDOW_MS);
}

static void motor_gate_banner(void)
{
#if ROSE_FLIGHT_COMMIT
	printk("flight_controller: COMMIT GATE -- build is COMMIT-CAPABLE (ROSE_FLIGHT_COMMIT=1) but "
	       "NOT COMMITTED: motors latch OFF %d ms after they first spin.\n",
	       (int)(ROSE_MOTOR_WINDOW_MS));
	printk("flight_controller: COMMIT GATE -- to commit, write 0x%08x to g_flight_commit_key "
	       "(hardware/flight/commit.gdb). Committed cap is %d ms.\n",
	       (unsigned)ROSE_COMMIT_KEY, (int)(ROSE_COMMIT_MAX_MS));
#else
	printk("flight_controller: COMMIT GATE -- UNCOMMITTED BUILD: motors latch OFF %d ms after "
	       "they first spin, and CANNOT be committed at run time.\n",
	       (int)(ROSE_MOTOR_WINDOW_MS));
	printk("flight_controller: COMMIT GATE -- a flight test needs BOTH a rebuild with "
	       "RB_CMAKE_ARGS=\"-DROSE_FLIGHT_COMMIT=1\" AND hardware/flight/commit.gdb.\n");
#endif
}

/* A soft reset (rose_cmd_reset / hardware/flight/reset.gdb) clears estop and disarms, and the
 * operator reasonably expects it to clear everything. It does NOT clear this latch -- "stays there
 * until reset" means a real reset -- so say so, once, rather than leaving them to wonder why the
 * motors never came back. */
static void motor_gate_note_reset(void)
{
	if (g_motor_latched) {
		printk("MOTOR GATE: soft reset does NOT clear the window latch -- motors stay OFF "
		       "until the board is reset.\n");
	}
}

/*
 * Called once per control tick with the duty that WOULD have been written, after every other
 * safety layer has had its say. Returns false when the caller must write zeros instead.
 *
 * Deliberately last in send_control() on both actuator backends: taking the duty as an argument
 * rather than re-deriving it means no branch above can reach a motor without passing through here,
 * and the gate cannot disagree with the actuator about what was about to be commanded.
 */
static bool motor_gate_permit(const float *duty)
{
	const int64_t now = k_uptime_get();
	bool live = false;

#if ROSE_FLIGHT_COMMIT
	if (!g_motor_committed && g_flight_commit_key == ROSE_COMMIT_KEY) {
		if (g_motor_latched) {
			static bool refused;

			if (!refused) {
				refused = true;
				printk("MOTOR GATE: COMMIT REFUSED -- the window already latched. "
				       "Reset the board, then commit BEFORE the motors run.\n");
			}
		} else {
			g_motor_committed = true;
			printk("MOTOR GATE: *** COMMITTED TO A FLIGHT TEST *** -- motor budget is now "
			       "%d ms (was %d ms). Props are live.\n",
			       (int)(ROSE_COMMIT_MAX_MS), (int)(ROSE_MOTOR_WINDOW_MS));
		}
	}
#endif
	if (g_motor_latched) {
		return false;
	}
	for (int i = 0; i < NACTIONS; i++) {
		if (duty[i] > 0.0f) {
			live = true;
			break;
		}
	}
	if (g_motor_live_since == 0) {
		if (!live) {
			return true;   /* nothing has spun yet: the clock has not started */
		}
		g_motor_live_since = now;
		printk("MOTOR GATE: WINDOW OPEN -- motors are live; %d ms budget starts now (%s)\n",
		       motor_gate_limit_ms(), g_motor_committed ? "COMMITTED" : "uncommitted");
	}
	{
		const int64_t elapsed = now - g_motor_live_since;

		if (elapsed >= (int64_t)motor_gate_limit_ms()) {
			g_motor_latched = true;
			printk("MOTOR GATE: WINDOW EXPIRED after %d ms -- ALL FOUR MOTORS COMMANDED OFF "
			       "AND LATCHED (board reset to clear)\n", (int)elapsed);
			return false;
		}
		{
			static int64_t next_note;

			if (now >= next_note) {
				next_note = now + (int64_t)(ROSE_MOTOR_LIVE_NOTE_MS);
				printk("MOTOR GATE: MOTORS LIVE %d/%d ms (%s)\n", (int)elapsed,
				       motor_gate_limit_ms(),
				       g_motor_committed ? "COMMITTED" : "uncommitted");
			}
		}
	}
	return true;
}
#endif /* HAVE_ROSE */

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
static inline void motor_power_banner(void) { /* no physical motor on the RoSE target */ }
static void motors_startup_pulse(void) { /* no motors on the RoSE target */ }
static void motors_boot_chirp(void) { /* no motors on the RoSE target */ }
static void motors_shutdown(void) { /* no motors on the RoSE target */ }
#else /* real target: drive 4 PWM motors (thrust ~ duty). Actuator parity is future work. */
#include <zephyr/drivers/pwm.h>
/* Last duty actually written per motor, and the last return code from writing
 * it. Both exist because one motor of four failing is invisible in u. */
static volatile float g_last_duty[NACTIONS];
static volatile int   g_last_pwm_rc[NACTIONS];

/* SAFETY (early bench bring-up): hard ceiling on motor duty. The controller
 * regulates to a hover setpoint, so its raw command ramps toward hover/takeoff
 * thrust; this scales the full [0,1] duty into [0, MOTOR_MAX_DUTY] so the
 * controller can respond and be observed, but CANNOT produce flight thrust.
 * Raise deliberately only for actual flight testing.
 *
 * IT IS A SCALE, NOT A CLAMP, and the difference decides whether a flight build
 * can hover. actuator_duty() MULTIPLIES by it (the loop at the end of that
 * function); the clamp on the following line only catches the already
 * impossible. So every duty -- collective AND differential -- is multiplied by
 * this number. Thrust goes as roughly duty^2, so a cap of s gives about s^2 of
 * full thrust while leaving the torque/thrust ratio unchanged, which is what
 * makes it a usable power limiter rather than something that eats attitude
 * authority. What it also means: the PID altitude loop has no integrator, so it
 * cannot claw the scale back. A cap below the real hover duty is a drone that
 * sits on the floor at full command, not a drone that climbs slowly.
 *
 * IT IS NOT THE ONLY CEILING. On the ESP-offload path the last word belongs to
 * ESP_MOTORS_MAX_DUTY_PCT in workloads/esp_motors (default 25 %), applied in
 * motors_apply() after the frame arrives, and it deliberately does not trust
 * this side. Raising only this one is silently clamped there and looks exactly
 * like a thrust failure or a tuning problem. See the check under
 * ROSE_ESP_MOTORS below, which is there to make that mistake noisy.
 *
 * These three knobs used to live inside the `#if DT_NODE_EXISTS(MOTORS_NODE)`
 * block below, with MOTOR_MAX_DUTY doubly #ifndef-guarded so that a build which
 * already defined it (every build does -- the CMakeLists passes all three)
 * skipped the break-away and chirp defaults too. Hoisted and de-nested here
 * because the ESP-offload backend needs the same numbers and must not carry a
 * second copy of them. No build changes: CMake defines all three either way. */
#ifndef MOTOR_MAX_DUTY
#define MOTOR_MAX_DUTY 0.10f
#endif

/* ---- MOTOR_MAX_DUTY IS A SCALE, NOT A CLAMP -- and why a second knob exists.
 *
 * Read actuator_duty() below before setting either of these for a flight test.
 * On the bench (non-autoflight) path MOTOR_MAX_DUTY is applied as the LAST
 * step, as a MULTIPLIER on a duty that is already physical:
 *
 *     duty[i] = (u[i] + 0.583) ... anti-saturation cut at 1.0 ... * MOTOR_MAX_DUTY
 *
 * That is right for a propellers-off bench run -- every motor still responds
 * and the differential is visible at a duty too low to lift anything -- and it
 * is WRONG as a flight cap, for a reason that is easy to miss and expensive to
 * discover with props on:
 *
 *   the controller's output IS the physical duty it wants. u + 0.583 round-trips
 *   controller_pid.cpp's forceToVoltage() exactly. Hover on this airframe needs
 *   ~0.65-0.71 duty (FLIGHT_BUILD.md: "hover needs ~65%"; FLIGHT_TUNING_LOG.md:
 *   "~71 % hover duty"). Set MOTOR_MAX_DUTY to 0.75 "as a 75 % cap" and the
 *   motors get 0.71 * 0.75 = 0.53 at the controller's hover command -- about
 *   55 % of hover THRUST, since thrust goes as duty^2. The drone cannot hover.
 *   Worse, it does not fail cleanly: while the altitude error is large the loop
 *   saturates, the scaled command sits at 0.75 and the drone climbs hard; as it
 *   approaches the setpoint the command falls back toward 0.71, the scale bites,
 *   and thrust collapses. That is a porpoise, with props on, on a tether.
 *
 * MOTOR_DUTY_CEILING is the knob that means what "cap" sounds like. It replaces
 * the hard-coded 1.0 the bench path's ATTITUDE-PRIORITY ANTI-SATURATION cut
 * works against, so exceeding it lowers the COLLECTIVE and preserves the
 * differential -- the same semantics AUTOFLIGHT_MAX_DUTY already has on the
 * autoflight path, and the reason a per-motor clamp is the wrong tool here (a
 * clamp flattens four commands into four equal numbers = no attitude authority,
 * exactly when the vehicle is asking for the most thrust).
 *
 * Default 1.0f, so every existing build is bit-identical: the cut compares
 * against 1.0 as before and no clamp fires below it.
 *
 * FOR A TETHERED FLIGHT ATTEMPT the pair is:
 *
 *     RB_CMAKE_ARGS="-DMOTOR_MAX_DUTY=1.0f -DMOTOR_DUTY_CEILING=0.75f"
 *
 * Both are CMake cache variables listed in target_compile_definitions (see
 * CMakeLists.txt), so RB_CMAKE_ARGS is what reaches them; a bare -D of a name
 * that is NOT listed there is discarded, which is the trap that CMakeLists
 * already documents for PID_MASS_KG and CS_GPIO_PIN. Cache variables also
 * PERSIST in a build tree, so use --pristine always when switching a build tree
 * between a flight configuration and a bench one, and read the value back off
 * the DUTY CHAIN boot banner rather than trusting the command line.
 *
 * AND IT IS NOT THE LAST CEILING. workloads/esp_motors applies its own,
 * ESP_MOTORS_MAX_DUTY_PCT (default 25 %), in the last component before the
 * gate, and it does not trust this image. Raising only these two still clamps
 * at 25 % at the ESP and looks exactly like a thrust failure. See
 * docs/tethered-flight-attempt.md. */
#ifndef MOTOR_DUTY_CEILING
#define MOTOR_DUTY_CEILING 1.0f
#endif
static_assert((float)(MOTOR_DUTY_CEILING) > 0.0f && (float)(MOTOR_DUTY_CEILING) <= 1.0f,
	      "MOTOR_DUTY_CEILING must be in (0, 1]: it is a fraction of full duty");
static_assert((float)(MOTOR_MAX_DUTY) > 0.0f && (float)(MOTOR_MAX_DUTY) <= 1.0f,
	      "MOTOR_MAX_DUTY must be in (0, 1]: it is a fraction of full duty");

/* ---- Break-away duty -------------------------------------------------------
 * The lowest duty at which EVERY motor on this airframe reliably starts.
 *
 * Measured on riskybird v3, driving one motor at a time: 15% starts all four,
 * 10% starts three. Motor 4 does not break away at 10% -- it sits stalled.
 *
 * WHAT A STALLED MOTOR ACTUALLY COSTS, corrected. An earlier version of this
 * comment said the motor's ESC latches locked-rotor protection and that only
 * removing power clears it. THERE ARE NO ESCs ON THIS AIRFRAME. The motors are
 * brushed, driven low-side by SI2302 FETs straight off the pack; there is no
 * controller in between to latch anything. The hazard is real but different: a
 * stalled brushed motor is very nearly a short across the pack through the FET,
 * dissipating in the winding and the channel with no rotation to cool either.
 * It heats in seconds. So the rule is unchanged -- do not command a duty below
 * break-away -- but the reason is thermal, the damage is cumulative rather than
 * a latch, and NOTHING clears itself when you power-cycle. Do not leave the
 * drone sitting at a sub-break-away duty while you read the console.
 *
 * ALSO: 15% IS AN FPGA-PATH NUMBER. It was measured with the FPGA driving the
 * gates, and all three of the things that set it have changed under
 * ROSE_ESP_MOTORS: the gate is now driven from the ESP's 3.3 V through its own
 * 47R rather than from FPGA VCCO through the FPGA's, which is a different V_GS
 * on the SI2302; the FPGA path's motor outputs are INVERTED in the generated
 * Verilog and nobody has re-derived whether a commanded duty was the duty the
 * gate saw; and the quantisation differs (sifive comparator 1/2500 vs LEDC
 * 11-bit). Treat 0.15 as an order of magnitude on the offload path, not as a
 * measurement, until it is re-measured there with workloads/motor_duty.
 *
 * This is a property of the airframe, not of the code. Re-measure it after any
 * motor or wiring change: drive each motor alone, step the duty up, and take
 * the highest value at which any of them first turns -- then leave margin.
 */
#ifndef MOTOR_BREAKAWAY_DUTY
#define MOTOR_BREAKAWAY_DUTY 0.15f
#endif
/* Long enough for a stationary rotor to actually spin up. 250 ms was the old
 * value and is marginal: a motor that has not broken away by the time the pulse
 * ends has spent the whole pulse stalled. */
#ifndef MOTOR_CHIRP_MS
#define MOTOR_CHIRP_MS 400
#endif

/* ---- Motor offload to the ESP32-C6 ----------------------------------------
 * ROSE_ESP_MOTORS=1 replaces the PWM actuator with a duty-frame emitter on
 * uart1. The FPGA then drives NO motor gate at all: ball F13 is dead as an
 * output so motor4 could never be driven from here, and every gate is a
 * wired-OR of an FPGA 47R and an ESP 47R into a 10k pulldown, so handing all
 * four to the ESP removes the dual-driver contention instead of working around
 * one pin. See integration/zephyr/motor_link/include/riskybird/motor_link.h. */
#ifndef ROSE_ESP_MOTORS
#define ROSE_ESP_MOTORS 0
#endif

/*
 * THE SECOND CEILING, named here because raising the first one alone is silent.
 *
 * workloads/esp_motors applies ESP_MOTORS_MAX_DUTY_PCT (default 25) in
 * motors_apply(), as the last thing before the gate, to every duty it receives.
 * That is deliberate -- the receiver does not trust the sender -- and it means a
 * flight-configured FPGA build flashed against a default ESP image is clamped at
 * 25 % and produces no useful thrust. From the FPGA console that is
 * indistinguishable from a broken motor, a dead pack or a mis-tuned controller,
 * which is the failure mode this message exists to pre-empt.
 *
 * A #pragma message rather than an #error: the two images are built and flashed
 * separately (different toolchains, different boards), so this side cannot know
 * what the ESP is running, and refusing the build would be refusing something
 * that may well be correct. It prints at compile time, in the build log, right
 * where the operator raised the cap.
 */
/* The warning itself is issued at RUN TIME by motors_startup_pulse() on the offload path, for
 * two reasons: `#if` cannot compare floats, and the operator reads the boot console at the
 * bench while a compile-time message scrolls past once, days earlier, on another machine. */

/* ---- Controller thrust -> per-motor duty -----------------------------------
 * The ONE copy of this math. Both actuator backends call it, so the PWM path
 * and the ESP-offload path cannot drift apart in the battery scaling, the
 * clamps, the anti-saturation cut or the bench ceiling -- which is exactly the
 * kind of divergence that would only show up in flight. */
static inline void actuator_duty(const float *u, float duty[NACTIONS])
{
	/* Controller's normalized thrust (u in ~[-0.583, 0.417]) -> physical per-motor duty [0,1].
	 * Battery sag compensation: multiply the raw thrust command by BATT_NOMINAL_V/g_vbat (clamped to
	 * [1.0, BATT_SCALE_MAX]) so commanded thrust holds as the pack drains. Applied BEFORE the
	 * anti-saturation cut and the ceiling below; batt_thrust_scale() returns 1.0 (no-op) when
	 * battery sense is disabled or g_vbat looks invalid. */
	const float batt_scale = batt_thrust_scale();
	/* The ceiling the anti-saturation cut works against, IN THE UNITS duty[] carries here:
	 *
	 *   autoflight -- duty[] is already physical, so the ceiling is the flight cap itself.
	 *   bench      -- duty[] is still NORMALIZED [0,1]; MOTOR_MAX_DUTY is applied as a SCALE
	 *                 at the end. The saturation that matters on that path is therefore the
	 *                 one at 1.0, not the one at the cap.
	 */
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	const float ceiling = ((float)(AUTOFLIGHT_MAX_DUTY) < 1.0f) ? (float)(AUTOFLIGHT_MAX_DUTY) : 1.0f;
#else
	/* Was a hard-coded 1.0f. See the MOTOR_DUTY_CEILING note above: this is the
	 * ceiling the collective cut works against, and it is the ONLY duty limit on
	 * this path that preserves the roll/pitch/yaw differential. */
	const float ceiling = (float)(MOTOR_DUTY_CEILING);
#endif
	float peak = 0.0f;

	for (int i = 0; i < NACTIONS; i++) {
		duty[i] = (u[i] + 0.583f) * batt_scale;
		if (duty[i] < 0.0f) duty[i] = 0.0f;
		/* NOTE the absence of a per-motor clamp to the ceiling here, and that it is the whole
		 * point of this function. Clamping each motor to 1.0 at this line is what destroyed the
		 * differential: with every u >= 0.417 (which is every iteration of a bench run that
		 * regulates to TARGET_Z from the floor) all four raw duties exceed 1.0, all four clamp
		 * to exactly 1.0, and four identical numbers stay identical through anything applied
		 * afterwards -- including the cut below, which then subtracted the same amount from
		 * four equal values and produced four equal values. Measured on the bench: u =
		 * [1.082 0.977 1.429 1.541] -> duty = [0.100 0.100 0.100 0.100] for 1200 iterations. */
		if (duty[i] > peak) peak = duty[i];
	}
	/* ATTITUDE-PRIORITY ANTI-SATURATION, on EVERY path -- not just autoflight. The collective
	 * (altitude) thrust and the roll/pitch/yaw differentials share the same motor range. If the
	 * peak motor would exceed the ceiling, subtract the excess from ALL FOUR: this lowers the
	 * COLLECTIVE thrust while preserving the differential, so attitude authority always survives
	 * -- sacrifice a little altitude, never attitude. (Per-motor clamping instead flattens the
	 * differential once the altitude loop maxes out -> no control -> tip.) This is THE fix for the
	 * "bad down-ToF maxes the altitude loop -> all four pin -> tip/tumble" failure: the drone
	 * climbs LEVEL and recoverable instead.
	 *
	 * It is applied on the bench path too because "all four pinned at the cap" is not a safe
	 * bench behaviour either -- it is the SAME loss of attitude authority, just at a duty too low
	 * to demonstrate it -- and because the bench is where the flight path's saturation behaviour
	 * has to be observable BEFORE props go on. In the unsaturated region (peak <= ceiling) this
	 * is bit-identical to what it replaces: the cut does not run and no clamp ever fired. */
	if (peak > ceiling) {
		const float cut = peak - ceiling;
		for (int i = 0; i < NACTIONS; i++) {
			duty[i] -= cut;   /* collective cut; a low motor may go < 0 -> floored here */
			if (duty[i] < 0.0f) duty[i] = 0.0f;
		}
	}
#if !(defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT)
	for (int i = 0; i < NACTIONS; i++) {
		duty[i] *= MOTOR_MAX_DUTY;                 /* bench: scale [0,1] -> [0, cap] */
		if (duty[i] > MOTOR_MAX_DUTY) duty[i] = MOTOR_MAX_DUTY;
	}
#endif
}

/*
 * Say, at boot, the WHOLE duty chain and the worst-case number that can come out
 * of it -- in the same spirit as safety_banner(): a limit that is only a source
 * comment is a limit nobody can check against the thing in front of them.
 *
 * The operator's second hard requirement is "it never runs too powerfully". The
 * answer to that is one number, and this is where it gets printed. It is the
 * duty this IMAGE can command; it is NOT what reaches a gate on the offload
 * path, because workloads/esp_motors applies its own ceiling afterwards and this
 * image has no way to read it. That asymmetry is stated out loud rather than
 * left for someone to assume the printed number is the final one.
 */
static void motor_power_banner(void)
{
#if defined(ROSE_AUTOFLIGHT) && ROSE_AUTOFLIGHT
	const float ceiling = ((float)(AUTOFLIGHT_MAX_DUTY) < 1.0f) ? (float)(AUTOFLIGHT_MAX_DUTY) : 1.0f;
	const float scale = 1.0f;
	const char *which = "AUTOFLIGHT_MAX_DUTY";
#else
	const float ceiling = (float)(MOTOR_DUTY_CEILING);
	const float scale = (float)(MOTOR_MAX_DUTY);
	const char *which = "MOTOR_DUTY_CEILING";
#endif
	const float worst = ceiling * scale;

	printk("flight_controller: DUTY CHAIN -- collective ceiling %s = %s%d.%03d (anti-saturation "
	       "cut, differential preserved), then x MOTOR_MAX_DUTY %s%d.%03d\n",
	       which, FP3(ceiling), FP3(scale));
	printk("flight_controller: DUTY CHAIN -- WORST CASE THIS IMAGE CAN COMMAND: %d.%02d %% per "
	       "motor%s\n", (int)(worst * 100.0f), ((int)(worst * 10000.0f)) % 100,
	       (worst < 0.20f) ? "  (bench cap -- CANNOT LIFT; hover needs ~65-71 %)" : "");
	/*
	 * The one way to get this wrong that fails TOWARDS more power, said loudly.
	 *
	 * Setting MOTOR_MAX_DUTY=1.0f turns the scale off, which is correct for a flight build --
	 * but if MOTOR_DUTY_CEILING is then forgotten this image has NO duty limit of its own and
	 * the only remaining one is the ESP's. Every other mistake in this chain fails towards a
	 * drone that cannot lift; this one does not, so it gets its own line rather than leaving the
	 * operator to divide two numbers on a banner.
	 *
	 * 0.85 rather than 1.0: on this airframe hover is ~0.65-0.71 duty and thrust goes as duty^2,
	 * so 0.85 is already ~1.43x weight. Anything above it is not a cap in any useful sense.
	 */
	if (worst > 0.85f) {
		printk("flight_controller: *** DUTY CHAIN -- NO MEANINGFUL POWER CAP ON THIS SIDE *** "
		       "%d %% is ~%d.%02dx hover thrust (hover ~0.71 duty, thrust ~ duty^2). If you "
		       "meant to cap power, set -DMOTOR_DUTY_CEILING; MOTOR_MAX_DUTY alone is a SCALE "
		       "and detunes the controller instead of capping it.\n",
		       (int)(worst * 100.0f),
		       (int)((worst * worst) / (0.71f * 0.71f)),
		       ((int)((worst * worst) / (0.71f * 0.71f) * 100.0f)) % 100);
	}
#if ROSE_ESP_MOTORS
	printk("flight_controller: DUTY CHAIN -- the ESP applies its OWN ceiling "
	       "(ESP_MOTORS_MAX_DUTY_PCT) after this one and does not trust this image; read it "
	       "off the ESP console's 'duty ceiling NN%%' boot line. The LOWER of the two wins.\n");
#endif
}

#define MOTORS_NODE DT_ALIAS(motors)
#if DT_NODE_EXISTS(MOTORS_NODE)
static const struct pwm_dt_spec motors[NACTIONS] = {
	PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 0),
	PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 1),
	PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 2),
	PWM_DT_SPEC_GET_BY_IDX(MOTORS_NODE, 3),
};
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
	/*
	 * ONE write path, and one place that decides the duty.
	 *
	 * Every reason a motor might be off -- inhibit, estop, disarm, the bench actuation timeout,
	 * and now the commitment gate -- converges on the single loop at the bottom instead of each
	 * returning early with its own copy of "write zeros". The early returns were not equivalent:
	 * the actuation-timeout branch zeroed the PWM registers but left g_last_duty holding the last
	 * flying values, so telemetry reported a drone still under power after the bench cut. The
	 * estop branch already carried a comment about exactly that hazard; the timeout branch had
	 * the bug the comment describes. Converging them fixes it by construction.
	 */
	float duty[NACTIONS] = { 0.0f, 0.0f, 0.0f, 0.0f };

#if ROSE_MOTORS_INHIBIT
	(void)u;   /* hard-inhibited: duty stays all-zero, and 0 is still WRITTEN and reported */
#else
	bool blocked = g_estop || !g_armed;
#if ROSE_ACTUATE_TIMEOUT_MS > 0
	/* Bench safety: cut all motors ROSE_ACTUATE_TIMEOUT_MS after boot. The
	 * controller/estimator keep running (still logging) -- only the actuator stops.
	 * NOTE its default is 0, i.e. DISABLED; the commitment gate below is what bounds
	 * motor run time in a default build. */
	if (!blocked && k_uptime_get() >= (int64_t)ROSE_ACTUATE_TIMEOUT_MS) {
		static bool stopped;

		blocked = true;
		if (!stopped) {
			printk("send_control: actuation timeout (%d ms) reached -- motors OFF\n",
			       (int)ROSE_ACTUATE_TIMEOUT_MS);
			stopped = true;
		}
	}
#endif
	if (!blocked) {
		/* The battery scaling, the [0,1] clamp, the autoflight anti-saturation cut
		 * and the bench ceiling all live in actuator_duty() now, so the ESP-offload
		 * backend applies exactly the same numbers. */
		actuator_duty(u, duty);
	}
#endif
	/* Commitment gate LAST, on the duty that would actually have been written: no branch above
	 * can reach a motor without passing through it, and it is the only thing here that latches. */
	if (!motor_gate_permit(duty)) {
		for (int i = 0; i < NACTIONS; i++) {
			duty[i] = 0.0f;
		}
	}
	for (int i = 0; i < NACTIONS; i++) {
		int prc = pwm_set_pulse_dt(&motors[i],
					   (uint32_t)(motors[i].period * duty[i]));

		/* What was actually asked of the pin, kept for telemetry. The loop
		 * prints u, which is the CONTROLLER's normalized thrust -- it does
		 * not survive the sag scale, the [0,1] clamp, the collective
		 * anti-saturation cut or the floor below zero. A motor sitting at
		 * exactly 0 duty while its u reads 1.3 is a state the old telemetry
		 * could not show, and it is the first thing worth ruling out when
		 * one motor of four does not spin. */
		g_last_duty[i] = duty[i];

		/* pwm_set_pulse_dt's return was discarded here. motor4 is the only
		 * output on pwm1 rather than pwm0, so it is the one channel whose
		 * write can fail on its own -- and a silent failure looks exactly
		 * like a dead motor or a broken wire from outside. */
		if (prc != 0 && g_last_pwm_rc[i] != prc) {
			printk("send_control: motor%d pwm_set rc=%d\n", i + 1, prc);
		}
		g_last_pwm_rc[i] = prc;
	}
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
		/* At or above break-away, never below: a chirp that stalls a motor
		 * is worse than no chirp. Not because anything latches -- there are
		 * no ESCs here, the motors are brushed on low-side SI2302 FETs --
		 * but because a stalled brushed motor is a resistor across the pack
		 * drawing locked-rotor current with no back-EMF and no airflow, so
		 * motor and FET heat within seconds. See MOTOR_BREAKAWAY_DUTY. */
		pwm_set_pulse_dt(&motors[i],
				 (uint32_t)(motors[i].period * MOTOR_BREAKAWAY_DUTY));
		k_msleep(MOTOR_CHIRP_MS);
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
			pwm_set_pulse_dt(&motors[i],
					 (uint32_t)(motors[i].period *
						    MOTOR_BREAKAWAY_DUTY));
		}
		/* 150 ms was shorter than the boot chirp's 250 and drove all four
		 * at once, so this was the more likely of the two to leave a motor
		 * stalled rather than spinning. */
		k_msleep(MOTOR_CHIRP_MS);
		for (int i = 0; i < NACTIONS; i++) {
			pwm_set_pulse_dt(&motors[i], 0);
		}
		k_msleep(120);
	}
}
/*
 * End of run: say OFF, do not merely stop talking.
 *
 * The control loop is finite (CTRL_ITERS) in most builds. When it ends, main() returns and nothing
 * calls send_control() again. On the PWM path the comparators keep whatever they hold, so they must
 * be written to zero. On the ESP-offload path silence alone WOULD stop the motors -- the receiver's
 * 120 ms failsafe sees the frames stop -- but "commanded off" and "link dead" are different things
 * and the receiver should be told which one this is, at once, rather than inferring it 120 ms later
 * from an absence. Three frames, because the last word on a wire should not depend on one frame.
 */
static void motors_shutdown(void)
{
	for (int i = 0; i < NACTIONS; i++) {
		pwm_set_pulse_dt(&motors[i], 0);
		g_last_duty[i] = 0.0f;
	}
	printk("flight_controller: motors parked at 0 duty (end of run)\n");
}
#elif ROSE_ESP_MOTORS
/* ---- Actuator: motor offload to the ESP32-C6 --------------------------------
 *
 * This backend drives NO motor pin. It encodes the same duties the PWM backend
 * would have written and sends them to the ESP32-C6 over uart1; the ESP owns all
 * four gates and generates the PWM. See the protocol header for the frame, the
 * latency budget and the resynchronisation rule.
 *
 * Why there is no PWM here at all: ball F13, motor4's gate driver, is dead as an
 * output. A bare-metal bitstream driving the four pads at 4/8/12/16 % reads back
 * M1=40 M2=80 M3=120 M4=0 per mille at the gates, and the same ball configured
 * as an input reads the carrier's 10k pulldown -- so the net is fine and the IOB
 * is not. Handing only motor4 to the ESP would leave the other three gates
 * driven by two 47R sources at once; handing over all four removes that as well.
 *
 * SAFETY, and specifically how this is BETTER than what it replaces. The SiFive
 * PWM comparators are free-running hardware: halting the Rocket core -- which
 * `rb debug` does every session -- leaves them driving the gates at the last
 * programmed duty, forever, until a reconfigure or a power cycle. Nothing in the
 * current design stops that. Here, a halted core simply stops emitting frames,
 * and the ESP cuts all four motors after its receive timeout. A crash, a reset
 * and an unplugged ribbon all behave the same way.
 *
 * "Off" is also sent EXPLICITLY rather than being left to the timeout: while
 * disarmed, estopped, past the bench actuation timeout, or built with motors
 * inhibited, this still transmits a frame -- with duty 0 and the ARMED bit
 * clear -- so the receiver distinguishes "commanded off" from "link dead".
 */
#include <zephyr/drivers/uart.h>
#include <riskybird/motor_link.h>

#if !DT_NODE_EXISTS(DT_ALIAS(esp_uart))
#error "ROSE_ESP_MOTORS=1 needs the esp-uart alias: build a shell that elaborates \
serial@10021000 so rb appends hardware/zephyr/fpga-esp-uart.overlay."
#endif
/*
 * uart1 now carries TWO things: these binary duty frames and the mirrored ASCII
 * telemetry line (see the esp-uart mirror at the print site). That multiplex is
 * safe because both writers are the control-loop thread and are strictly
 * sequential -- but only while printk is somewhere else. Point zephyr,console at
 * esp-uart as well (what --mode telem does, and it does NOT set ROSE_ESP_MOTORS)
 * and every printk in the image, from any thread and at any moment, becomes a
 * third writer that can land in the middle of a 14-byte frame. The receiver's
 * sliding window would resynchronise on the next frame, but that frame is lost
 * and the loss is invisible from this end. Refuse the build instead.
 */
#if defined(ESP_UART_IS_CONSOLE) && ESP_UART_IS_CONSOLE
#error "ROSE_ESP_MOTORS=1 with zephyr,console on esp-uart: printk would interleave \
ASCII into the middle of a binary duty frame. Keep the console on uart0 -- see \
hardware/zephyr/targets/fpga/workloads/flight_controller-esp-motors.overlay."
#endif
static const struct device *const esp_link = DEVICE_DT_GET(DT_ALIAS(esp_uart));

/*
 * Frame rate, fixed and independent of the loop rate.
 *
 * The control loop is 25-35 ms under the default controller but ~1.3 ms with the
 * PID cascade, and a frame per iteration at that rate would be 10.8 kB/s -- 94 %
 * of a 115200 link. 50 Hz is 700 B/s (6.1 %), leaves room for the ASCII
 * telemetry line if the two are ever merged onto this wire, and is 4x the
 * receiver's failsafe timeout in margin. A loop slower than 50 Hz simply sends
 * once per iteration.
 */
#ifndef ROSE_ESP_MOTORS_HZ
#define ROSE_ESP_MOTORS_HZ 50
#endif
#define ESP_MOTORS_PERIOD_MS (1000 / (ROSE_ESP_MOTORS_HZ))

/* Visible to the debugger and to whatever telemetry wants them: a link that is
 * not being fed is otherwise indistinguishable from motors that are commanded
 * to zero. */
static volatile uint32_t g_esp_frames;
static volatile uint8_t  g_esp_flags;

static void esp_motors_tx(const float duty[NACTIONS], uint8_t flags)
{
	static uint8_t seq;
	uint8_t frame[MOTOR_LINK_FRAME_LEN];
	uint16_t wire[MOTOR_LINK_NMOTORS];

	for (int i = 0; i < NACTIONS; i++) {
		wire[i] = motor_link_duty_from_float(duty[i]);
		g_last_duty[i] = duty[i];   /* what telemetry reports, as sent */
	}
	motor_link_encode(frame, ++seq, flags, wire);
	/* Polled, like every other UART write on this carrier. uart_sifive_poll_out
	 * spins only while the TX FIFO is FULL, so the cost charged to the control
	 * loop is the part of the 1.215 ms wire time the FIFO cannot absorb. */
	for (unsigned i = 0; i < MOTOR_LINK_FRAME_LEN; i++) {
		uart_poll_out(esp_link, frame[i]);
	}
	g_esp_frames++;
	g_esp_flags = flags;
}

/*
 * Re-send the last command, with a fresh sequence number.
 *
 * Called either side of the long telemetry write that shares this UART, so the
 * receiver's 120 ms failsafe is not being asked to tolerate a gap made of
 * console bytes. Read the note at the call site for the numbers; the two things
 * that matter here are that the seq MUST advance (a receiver drops a frame whose
 * seq is not newer and that path does not feed the failsafe, so a byte-identical
 * re-send would pet nothing) and that the payload is whatever send_control()
 * last decided -- identical to what the 50 Hz rate limiter emits between two
 * control updates, so this adds no new command, only a repeat of the live one.
 *
 * g_last_duty is volatile (it is read by telemetry and by the debugger), so it
 * is copied out before being handed back to the encoder.
 */
static inline void esp_motors_keepalive(void)
{
	float duty[NACTIONS];

	for (int i = 0; i < NACTIONS; i++) {
		duty[i] = g_last_duty[i];
	}
	esp_motors_tx(duty, g_esp_flags);
}

/*
 * "The duty-frame emitter above is the actuator in THIS build."
 *
 * Not the same statement as ROSE_ESP_MOTORS=1. The actuator backends are an
 * #if/#elif chain and the PWM branch is selected first, so a shell that both
 * declares a `motors` alias and sets ROSE_ESP_MOTORS compiles the PWM actuator
 * and none of this. The telemetry mirror's keepalive calls have to be guarded by
 * what was actually compiled, not by what was asked for, or such a build fails
 * with an undeclared esp_motors_keepalive 1500 lines away from the cause.
 */
#define ROSE_ESP_LINK_ACTIVE 1

static void send_control(const float *u)
{
	static int64_t next_ms;
	static uint8_t last_flags = 0xFFu;   /* nothing matches -> first call always sends */
	float duty[NACTIONS] = { 0.0f, 0.0f, 0.0f, 0.0f };
	uint8_t block = 0u;      /* every reason the motors must be off, as wire flags */
	bool    armed_now = false;
	uint8_t flags;
	int64_t now;

#if ROSE_MOTORS_INHIBIT
	block = MOTOR_LINK_FLAG_INHIBIT;
	(void)u;
#else
	if (g_estop) {
		block |= MOTOR_LINK_FLAG_ESTOP;
	}
#if ROSE_ACTUATE_TIMEOUT_MS > 0
	/* Bench safety: stop commanding thrust ROSE_ACTUATE_TIMEOUT_MS after boot.
	 * The controller and estimator keep running; only the actuator stops.
	 * NOTE its default is 0, i.e. DISABLED; the commitment gate below is what
	 * bounds motor run time in a default build. */
	if (k_uptime_get() >= (int64_t)ROSE_ACTUATE_TIMEOUT_MS) {
		block |= MOTOR_LINK_FLAG_TIMEOUT;
	}
#endif
	/* Armed AND nothing objecting. duty[] stays all-zero otherwise, so a frame
	 * that is not ARMED also carries zeros -- the receiver gets the same answer
	 * from the flag and from the payload. */
	if (g_armed && block == 0u) {
		armed_now = true;
		actuator_duty(u, duty);
	}
#endif
	/* Commitment gate LAST, on the duty that would actually have been sent: no branch above can
	 * reach the wire without passing through it. A refusal both zeroes the payload and clears
	 * ARMED, so the receiver is told to stop by the flag and by the data. */
	if (!motor_gate_permit(duty)) {
		for (int i = 0; i < NACTIONS; i++) {
			duty[i] = 0.0f;
		}
		block |= MOTOR_LINK_FLAG_WINDOW;
		armed_now = false;
	}
	flags = (uint8_t)(block | (armed_now ? MOTOR_LINK_FLAG_ARMED : 0u) |
			  (motor_gate_committed() ? MOTOR_LINK_FLAG_COMMIT : 0u));

	now = k_uptime_get();
	/* Rate-limited -- except that any change in the flags goes out at once. A
	 * disarm or an estop must not wait up to a frame period to be transmitted,
	 * and it is the one transition where latency has a cost. */
	if (flags == last_flags && now < next_ms) {
		return;
	}
	last_flags = flags;
	next_ms = now + ESP_MOTORS_PERIOD_MS;
	esp_motors_tx(duty, flags);
}

/*
 * No chirps on this path, deliberately.
 *
 * The boot / ready / startup chirps spin motors at MOTOR_BREAKAWAY_DUTY to tell
 * the operator the board reset. Reproducing them over the link means holding the
 * frame stream up during each k_msleep, which is exactly the pattern the
 * receiver's failsafe exists to cut. They are worth having back once this path
 * is proven on the bench -- as duty frames emitted from a loop, not as sleeps --
 * and an autoflight offload mode should not ship without them.
 */
static void motors_startup_pulse(void)
{
	printk("flight_controller: MOTOR OFFLOAD -- FPGA drives no gate; duty frames "
	       "at %d Hz on %s to the ESP32-C6\n", ROSE_ESP_MOTORS_HZ, esp_link->name);
	/*
	 * THE TWO CEILINGS, at boot, in one line, because raising one alone is silent.
	 *
	 * This side scales every duty by MOTOR_MAX_DUTY. The ESP then clamps whatever arrives to
	 * ESP_MOTORS_MAX_DUTY_PCT in its own motors_apply(), as the last thing before the gate,
	 * and it deliberately does not trust this side. A flight-configured FPGA image flashed
	 * against a default (25 %) ESP image is clamped there and produces no useful thrust, and
	 * from this console that is indistinguishable from a dead pack, a broken motor or a
	 * mis-tuned controller. This image cannot read the ESP's ceiling -- separate build,
	 * separate board, separate flash -- so it prints what it is asking for and names where the
	 * other half lives. The ESP prints its own ceiling in its boot banner; compare the two.
	 */
	printk("flight_controller: DUTY CEILING (this side) = %s%d.%03d of full scale. THE ESP HAS "
	       "ITS OWN, applied last: ESP_MOTORS_MAX_DUTY_PCT in workloads/esp_motors, default "
	       "25%%. Both must be raised in the same session or the motors clamp at the lower.\n",
	       FP3((float)(MOTOR_MAX_DUTY)));
	if (!device_is_ready(esp_link)) {
		printk("flight_controller: esp-uart NOT READY -- no frames will be sent, "
		       "so the ESP failsafe keeps every motor off\n");
		return;
	}
	/* One explicit disarmed, all-zero frame before the loop starts, so the ESP
	 * has a good frame to point at rather than inferring "off" from silence. */
	static const float zero[NACTIONS] = { 0.0f, 0.0f, 0.0f, 0.0f };
	esp_motors_tx(zero, 0u);
}
static void motors_boot_chirp(void) { /* see the note above */ }
static void motors_ready_chirp(void) { /* see the note above */ }
/*
 * End of run: say OFF, do not merely stop talking.
 *
 * The control loop is finite (CTRL_ITERS) in most builds. When it ends, main() returns and nothing
 * calls send_control() again. On the PWM path the comparators keep whatever they hold, so they must
 * be written to zero. On the ESP-offload path silence alone WOULD stop the motors -- the receiver's
 * 120 ms failsafe sees the frames stop -- but "commanded off" and "link dead" are different things
 * and the receiver should be told which one this is, at once, rather than inferring it 120 ms later
 * from an absence. Three frames, because the last word on a wire should not depend on one frame.
 */
static void motors_shutdown(void)
{
	static const float zero[NACTIONS] = { 0.0f, 0.0f, 0.0f, 0.0f };

	for (int i = 0; i < 3; i++) {
		esp_motors_tx(zero, 0u);
	}
	printk("flight_controller: sent explicit disarmed zero frames (end of run); the ESP "
	       "failsafe covers the silence that follows\n");
}
#else
static void send_control(const float *u) { (void)u; /* no actuator bound */ }
static void motors_startup_pulse(void) { /* no motors bound */ }
static void motors_boot_chirp(void) { /* no motors bound */ }
static void motors_ready_chirp(void) { /* no motors bound */ }
static void motors_shutdown(void) { /* no motors bound */ }
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

/*
 * THE THREADED BLOCKS HAVE NO ENVELOPE GUARD. Refuse to build them against a real actuator.
 *
 * io_block() reads a frame, hands it to est_block()/ctrl_block() and calls send_control() --
 * and that is the whole pipeline. safety_violation() is called from exactly one place in this
 * file, the single-loop control loop in main(); grep it. So -DROSE_THREADED=1 on a board that
 * can drive motors produces an image that commands thrust with NO accel-tilt, free-fall,
 * impact, est-tilt, rate, velocity or height check anywhere in the path -- and nothing at run
 * time says so, because safety_banner() is called from main() before the blocks start, so the
 * ENVELOPE GUARD banner still prints at boot. A guard that announces itself and is then never
 * evaluated is worse than no guard at all.
 *
 * The first hard requirement for a tethered flight attempt is that the watchdog is ALWAYS on.
 * This is the one build knob that could silently clear it, so it is refused at compile time
 * rather than documented. ROSE_THREADED is not set by any --mode (see tools/rb/boards.py) and
 * its cache default is 0, so this cannot fire by accident -- only a hand-written RB_CMAKE_ARGS
 * can reach it, which is exactly the case worth catching, since a flight build already has to
 * pass RB_CMAKE_ARGS for -DROSE_FLIGHT_COMMIT=1.
 *
 * HAVE_ROSE (co-sim) is exempt: there is no physical motor there, the RoSE actuator is a
 * bridge packet, and the threaded blocks exist to be debugged against the lockstep protocol.
 *
 * To lift this, MOVE the guard rather than deleting the check: the estimator state and the raw
 * accel/gyro must reach a guard evaluation on the same path that reaches send_control(), i.e.
 * inside io_block() or ctrl_block(), latching g_estop as the single loop does.
 */
#if ROSE_THREADED && !HAVE_ROSE
#error "ROSE_THREADED=1 on a real target: the threaded blocks never call safety_violation(), \
so the envelope guard would be absent while motors are commanded. Build the single loop \
(-DROSE_THREADED=0, the default), or port the guard into io_block()/ctrl_block() first."
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
	/* Both return codes are reported, because a silent `!= 0` here is
	 * indistinguishable from a sensor that is simply never ranging: tofv stays 0
	 * and the estimator dead-reckons altitude off accel with nothing on the
	 * console to say why. The errno is what separates the candidates -- -EIO is
	 * the bus, -EBUSY/-ETIMEDOUT a ranging budget that never completed, -ENOTSUP
	 * the driver wanting the data-ready pin this board does not wire (see the
	 * block comment above).
	 *
	 * Rate-limited: on a fetch that fails every iteration the back-off is 5 ms,
	 * so an unguarded print is 200 lines/s into a 115200 console, which both
	 * drowns the telemetry and changes the timing being measured. First failure
	 * prints immediately, then only on a CHANGE of code or once a second. */
	int last_rc = 1;            /* not a plausible rc, so the first failure always prints */
	uint32_t fails = 0;
	int64_t next_report = 0;
	for (;;) {
		int rc = sensor_sample_fetch(tof_dev);   /* blocks ~1 ranging budget on this thread */
		if (rc == 0) {
			struct sensor_value h;
			int grc = sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &h);

			if (grc != 0) {
				/* A fetch that succeeds and a get that does not is a different
				 * fault from a fetch that fails, and the old code merged the two
				 * by ignoring this rc and publishing whatever was in `h`. */
				if (grc != last_rc || k_uptime_get() >= next_report) {
					printk("flight_controller: down-ToF channel_get rc=%d\n", grc);
					last_rc = grc;
					next_report = k_uptime_get() + 1000;
				}
				k_msleep(5);
				continue;
			}
			if (fails != 0U) {
				printk("flight_controller: down-ToF recovered after %u failed fetches\n",
				       fails);
				fails = 0;
				last_rc = 1;
			}
			float hv = (float)sensor_value_to_double(&h);
			k_mutex_lock(&tof_mtx, K_FOREVER);
			g_tof_h = hv; g_tof_valid = true;
			k_mutex_unlock(&tof_mtx);
		} else {
			fails++;
			if (rc != last_rc || k_uptime_get() >= next_report) {
				printk("flight_controller: down-ToF sample_fetch rc=%d (%u consecutive)\n",
				       rc, fails);
				last_rc = rc;
				next_report = k_uptime_get() + 1000;
			}
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
#if defined(ROSE_ESTIMATOR_ONLY) && ROSE_ESTIMATOR_ONLY
	/*
	 * Estimator-only build: the controller is never asked for a command.
	 *
	 * This exists so the estimator can be judged on its own. With the
	 * controller in the loop on a bench-stationary drone, a starved
	 * altitude estimate (down-ToF invalid -> z dead-reckons off accel)
	 * makes the controller wind up chasing it, and the telemetry line then
	 * shows a ramping u[] on top of a drifting z -- two symptoms of one
	 * cause, which reads like a controller fault and is not. Forcing u to
	 * zero here leaves exactly one thing under test.
	 *
	 * Actuation is separately impossible in this build: main only drives
	 * PWM when the `motors` alias exists, and it does not here.
	 */
	(void)state; (void)dt;
	for (int i = 0; i < NACTIONS; i++) { u[i] = 0.0f; }
#else
	ctrl.compute(state, g_setpoint, u, dt);
#endif
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

#if HAVE_STATUS_LED
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
#endif /* HAVE_STATUS_LED */

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
	/* Say, at boot and in real units, what will stop the motors and how long they may run.
	 * The operator's requirement is to be ACTIVELY AWARE; a state nobody printed is a state
	 * nobody knows they are in. */
	safety_banner();
	motor_power_banner();
	motor_gate_banner();

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
	 * at board_sensor_init). Low priority + edge-only I2C writes -> negligible load on the loop.
	 *
	 * Guarded to match the definitions above: g_led_bus, led_t, led_stack,
	 * status_led_thread, PRIO_LED and STATUS_LED_CH all live inside the same
	 * HAVE_STATUS_LED block, so a board with the expander but no VL53L1X
	 * declared fails to compile here otherwise. */
#if HAVE_STATUS_LED
	if (g_led_bus) {
		k_thread_create(&led_t, led_stack, K_THREAD_STACK_SIZEOF(led_stack),
				status_led_thread, NULL, NULL, NULL, PRIO_LED, 0, K_NO_WAIT);
		k_thread_name_set(&led_t, "status_led");
		printk("flight_controller: status LED up (ADS7128 GPIO%d, state-derived patterns)\n",
		       STATUS_LED_CH);
	}
#endif /* HAVE_STATUS_LED */

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
	motors_shutdown();   /* transmit/write OFF explicitly; never leave it to silence */
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
				/* COMMAND ZERO FIRST, REPORT AFTERWARDS.
				 *
				 * Everything below this line is slow: printk on this carrier is a
				 * POLLING 115200 console that busy-waits (measured 30-60 ms stalls),
				 * flightlog_flush() writes flash, and after this block the loop still
				 * runs a full controller solve (~30 ms under TinyMPC) before it would
				 * otherwise reach send_control(). Latching g_estop and then talking
				 * would leave the motors driving for all of it. g_estop is already set,
				 * so this call forces every motor to 0 -- and on the ESP-offload path it
				 * transmits an explicit zero-duty ESTOP frame on this tick, because a
				 * flags change bypasses the frame rate limiter. `u` is not read. */
				send_control(u);
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
			motor_gate_note_reset();   /* the window latch is NOT one of the things this clears */
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
		/* Emergency watchdog: latch a kill if the airframe leaves its envelope.
		 *
		 * EVALUATED EVERY ITERATION, ARMED OR NOT; only the LATCH is gated on g_armed. The
		 * pre-arm lift-and-place swings the board by hand, fast enough to spike tilt, rate and
		 * flow velocity, and that must not estop before takeoff -- but the guard still has to
		 * LOOK the whole time, for two reasons. The arm gate now cross-checks the
		 * accelerometer's own tilt (g_guard_tilt_rad), which is only published by evaluating
		 * this; and a guard that is not looking while disarmed publishes nothing, so a console
		 * showing 0 could mean "level" or "not running" -- the ambiguity that let a guard sit
		 * three degrees under its trip point for months without anyone noticing.
		 *
		 * Debounced in TIME, not iterations. Same guard, same behaviour, whether the loop is
		 * the PID cascade at ~1.3 ms or TinyMPC at ~35 ms -- the iteration count it used to use
		 * meant 20 ms in one build and 525 ms in the other. */
		{
			int need_ms = SAFE_DEBOUNCE_MS;
			float meas = 0.0f;
			const char *unit = "";
			const char *why = safety_violation(state, f.gyro, f.accel, &need_ms, &meas, &unit);
			static int64_t viol_since;   /* ms at which the current violation began (0 = none) */
			static int     viol_count;   /* samples seen in the current violation */
			static bool    guard_announced;

			if (g_armed && !guard_announced) {
				guard_announced = true;
				printk("flight_controller: ENVELOPE GUARD ARMED -- watching accel tilt, "
				       "free-fall, impact, est tilt, rate, velocity, height\n");
			}
			if (!g_armed || g_estop) {
				why = NULL;   /* keep looking and keep publishing; do not latch */
			}
			if (why == NULL) {
				viol_since = 0; viol_count = 0;
			} else {
				if (viol_since == 0) { viol_since = t_now; viol_count = 0; }
				viol_count++;
			}
			if (why != NULL && viol_count >= SAFE_DEBOUNCE_MIN_SAMPLES &&
			    (t_now - viol_since) >= (int64_t)need_ms) {
				g_estop = true;
				/* COMMAND ZERO FIRST, REPORT AFTERWARDS.
				 *
				 * Everything below this line is slow: printk on this carrier is a
				 * POLLING 115200 console that busy-waits (measured 30-60 ms stalls),
				 * flightlog_flush() writes flash, and after this block the loop still
				 * runs a full controller solve (~30 ms under TinyMPC) before it would
				 * otherwise reach send_control(). Latching g_estop and then talking
				 * would leave the motors driving for all of it. g_estop is already set,
				 * so this call forces every motor to 0 -- and on the ESP-offload path it
				 * transmits an explicit zero-duty ESTOP frame on this tick, because a
				 * flags change bypasses the frame rate limiter. `u` is not read. */
				send_control(u);
				printk("flight_controller: *** EMERGENCY CUTOFF *** %s limit exceeded: "
				       "measured %s%d.%03d %s, held %d ms over %d samples; motors OFF "
				       "(reset to clear)\n", why, FP3(meas), unit,
				       (int)(t_now - viol_since), viol_count);
#if defined(ROSE_FLIGHTLOG) && ROSE_FLIGHTLOG
				/* Final record: encode WHICH limit tripped in flags[2..4] (0=none 1=tilt 2=rate
				 * 3=velocity 4=height 5=battery; first char of `why` is unique per reason) and carry
				 * the raw gyro x/y in the fvx/fvy columns -- the rate/gyro path isn't otherwise logged,
				 * so this is how we tell a vibration rate-spike from a real tilt/velocity runaway. */
				/* 6/7/8 are the accelerometer envelope's, added with it: 'f'ree-fall,
				 * 'i'mpact, 'a'ccel-tilt. Still one unique first character per reason. */
				uint8_t rc = (why[0]=='t')?1 : (why[0]=='r')?2 : (why[0]=='v')?3 : (why[0]=='h')?4 :
					     (why[0]=='b')?5 : (why[0]=='f')?6 : (why[0]=='i')?7 :
					     (why[0]=='a')?8 : 0;
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
				/* Level by BOTH references. The estimator's tilt (converted out of Gibbs)
				 * and the accelerometer's tilt straight from gravity must agree that the
				 * frame is flat, so an estimator that has not converged -- or has been
				 * misread, which is exactly what happened here -- cannot on its own permit
				 * arming. g_guard_tilt_rad is 0 until the accelerometer is in its trust
				 * band, which for a drone sitting on a bench it always is. */
				bool level  = tilt_rad_from_gibbs(state) < ARM_MAX_TILT_RAD &&
					      g_guard_tilt_rad < ARM_MAX_TILT_RAD;
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
		if (ROSE_TELEM && (iter % ROSE_TELEM_DIV) == 0) {   /* ROSE_TELEM=0 (flight) -> compiled out, no printk stall */
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
			/* roll/pitch/yaw here are the RODRIGUES (Gibbs) state, r = tan(theta/2) per
			 * axis -- NOT radians and NOT degrees. Reading them as radians is what put the
			 * tilt guard's real trip point at 90 deg while its constant said 57. The
			 * appended tilt= field is the honest number, in degrees: est is the estimator's
			 * (converted), acc is straight out of gravity with no estimator involved. Watch
			 * those two, not roll/pitch, when checking the guard. Fields are appended rather
			 * than substituted so existing parsers keep working. */
			printk(TELEM_LINE_FMT, TELEM_LINE_ARGS);
#if HAVE_ESP_UART
#if !ESP_UART_IS_CONSOLE
			/*
			 * Mirror the SAME line to the ESP link, while printk keeps the tethered
			 * console. One format string, one argument list, shared with the printk
			 * above -- so the radio sees the v1.1 state tail AND the guard fields,
			 * and the two sinks cannot drift into different formats.
			 *
			 * SHARING uart1 WITH THE BINARY DUTY FRAMES -- verified, not assumed.
			 * In an --mode esp-motors build this same UART also carries the 14-byte
			 * motor frames that esp_motors_tx() emits, so the two writers have to be
			 * shown not to interleave:
			 *
			 *   - BOTH writers are THIS thread. The control loop is single-threaded
			 *     (ROSE_THREADED=0, and no --mode sets it); send_control() is called
			 *     from this loop earlier in this same iteration and returns before
			 *     this block starts. Under ROSE_THREADED=1 the telemetry print does
			 *     not exist at all -- io_block() only actuates -- so there is no
			 *     configuration in which the two run concurrently.
			 *   - BOTH writers use uart_poll_out(), which returns only once the byte
			 *     is in the FIFO. There is no buffering layer that could reorder them
			 *     and no yield point inside either writer.
			 *   - NOTHING ELSE opens esp-uart. The down-ToF, flow, side-ToF,
			 *     flight-log and status-LED threads write the console (uart0) at
			 *     most, and no ISR touches this device.
			 *
			 * So a duty frame can never be split by a telemetry write: the two are
			 * strictly sequential, not merely unlikely to collide. The one build that
			 * WOULD break this -- the console pointed at esp-uart while duty frames
			 * are on it -- is refused at compile time by the check next to
			 * ROSE_ESP_MOTORS's esp-uart #error above.
			 *
			 * The other direction is the receiver's problem and is already solved:
			 * workloads/esp_bridge demux_feed() holds the text back by exactly one
			 * frame length and RETRACTS those bytes when the sliding-window decoder
			 * accepts a frame, because a duty field can legitimately contain 0x0A and
			 * would otherwise corrupt and split a telemetry line.
			 *
			 * Static storage rather than a ~300-byte frame on the control loop's
			 * stack. 384 bytes, not 256: with both the v1.1 tail and the guard fields
			 * the line runs ~290 bytes and a 256-byte buffer would silently truncate
			 * the end of it -- which is exactly where the new fields are.
			 */
			static char esp_line[384];
			int esp_len = snprintf(esp_line, sizeof(esp_line),
					       TELEM_LINE_FMT, TELEM_LINE_ARGS);
#if defined(ROSE_ESP_LINK_ACTIVE)
			/*
			 * Keep the motor link fed ACROSS the telemetry burst.
			 *
			 * Budget, measured: 115200 = 11520 B/s, and printk on this carrier is
			 * POLLED (no CONFIG_UART_INTERRUPT_DRIVEN), so every byte of both copies
			 * is charged to the control loop. ~290 bytes to the console plus ~290 to
			 * the radio is ~50 ms, the flow line adds ~10 ms, and the next iteration's
			 * own work is ~29 ms -- so without this the gap between two duty frames on
			 * a telemetry iteration is ~89 ms against the ESP's 120 ms failsafe. That
			 * is inside the timeout by 1.3x, which is not margin.
			 *
			 * Raising ROSE_TELEM_DIV does NOT fix it. A divisor makes the long gap
			 * RARER, not SHORTER, and the failsafe fires on the worst gap, not the
			 * average. Emitting a frame either side of the mirror does fix it: the
			 * worst gap becomes ~39 ms and the margin goes back to ~3x.
			 *
			 * What goes out is the duty and flags send_control() last decided, with a
			 * FRESH sequence number -- the same thing the 50 Hz rate limiter emits
			 * between control updates. It must be a new seq: handle_frame() on the
			 * receiver drops a frame whose seq is not newer (delta <= 0) and that path
			 * deliberately does not feed the failsafe, so a verbatim re-send would be
			 * counted stale and would pet nothing.
			 *
			 * This cannot paper over a real fault. The keepalive runs in the same
			 * thread as everything else, so a hung, crashed or JTAG-halted core stops
			 * emitting it too and the ESP still cuts all four motors within 120 ms.
			 */
			esp_motors_keepalive();
#endif
			if (esp_len > 0) {
				size_t length = MIN((size_t)esp_len, sizeof(esp_line) - 1U);
				esp_uart_write_line(esp_line, length);
			}
#if defined(ROSE_ESP_LINK_ACTIVE)
			esp_motors_keepalive();   /* see the note above the first one */
#endif
#endif /* !ESP_UART_IS_CONSOLE */
#endif /* HAVE_ESP_UART */
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
	motors_shutdown();   /* transmit/write OFF explicitly; never leave it to silence */
	printk("flight_controller: control loop done (%d iters)\n", CTRL_ITERS);
	return 0;
#endif
}
