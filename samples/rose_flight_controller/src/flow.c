/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * PMW3901 optical-flow front-end (see flow.h). Reuses the standalone pmw3901.c driver
 * (samples/riskybird/pmw3901_test) with the optimized busy_wait_us read path (~285 us/read).
 */
#include "flow.h"
#include "pmw3901.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <math.h>

/* --- tunables (override via -D) --- */
#ifndef FLOW_RAD_PER_COUNT
/* rad of ground-feature angle per PMW3901 motion count, taken from the Crazyflie flow deck (same
 * sensor + same Bitcraze register init, field-proven). Their measurement model is
 *   pixels = (Npix/thetapix) * angular_flow * dt,   measured_pixels = deltaX * FLOW_RESOLUTION
 * with Npix=35, thetapix=2*sin(42deg/2)=0.71674 rad, and FLOW_RESOLUTION=0.10 (the sensor reports
 * motion in 10x pixels). So rad/count = FLOW_RESOLUTION * thetapix / Npix = 0.10*0.71674/35 = 0.00205.
 * (An earlier 0.021 here omitted the 0.10 "10x pixels" factor -> ~10x too large; bench gyro-ratio and
 * slide-integration data bracketed 0.001-0.0035, consistent with 0.00205.) */
#define FLOW_RAD_PER_COUNT 0.00205f
#endif
#ifndef FLOW_MIN_SQUAL
#define FLOW_MIN_SQUAL 19        /* drop low-quality samples (surface-quality floor) */
#endif
#ifndef FLOW_PERIOD_MS
#define FLOW_PERIOD_MS 10        /* ~100 Hz background read; each read is ~285 us so it mostly idles */
#endif
#ifndef FLOW_STALE_MS
#define FLOW_STALE_MS 50         /* flow marked invalid if no fresh sample within this window */
#endif
/* Per-axis low-pass on the angular flow, applied HERE in the ~100 Hz sensor thread (NOT in the 1 kHz
 * estimator, where re-fusing the same stale sample every control tick converges right back to the raw
 * value and defeats any per-tick gain). Time-constant based (uses the measured dt) so it's independent
 * of the read period. The PMW3901's left (y) axis is far noisier (std ~1.3 m/s, railing to +/-3), yet a
 * real drift survives underneath (2:1 sign-imbalanced) -- a heavy y low-pass collapses the noise while
 * keeping that drift; x is usually cleaner so it needs only a light touch. Tune via -DFLOW_LP_TAU_{X,Y}. */
#ifndef FLOW_LP_TAU_X
#define FLOW_LP_TAU_X 0.0f       /* x low-pass time constant (s); 0 = off (pass raw) */
#endif
#ifndef FLOW_LP_TAU_Y
#define FLOW_LP_TAU_Y 0.0f       /* y low-pass time constant (s); 0 = off */
#endif

#define CS_GPIO_PIN 19           /* software chip-select (v3: SCLK=6 MOSI=7 MISO=18 CS=19) */

/* Manually-constructed pmw3901 device (the driver takes cfg/data via a struct device, same as the
 * pmw3901_test bring-up sample). */
static struct pmw3901_config pmw3901_cfg;
static struct pmw3901_data   pmw3901_data;
static const struct device   pmw3901_device = {
	.name = "FLOW", .config = &pmw3901_cfg, .data = &pmw3901_data,
};
#define PMW3901_DEV (&pmw3901_device)

/* --- cached flow snapshot --- */
static K_MUTEX_DEFINE(flow_mtx);
static float   g_ax, g_ay;       /* body-frame angular flow rate (rad/s), remapped */
static int     g_squal;
static bool    g_squal_ok;
static uint32_t g_seq;
static int64_t g_last_ms = -100000;

K_THREAD_STACK_DEFINE(flow_stack, 2048);
static struct k_thread flow_thread;

static void flow_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	uint32_t t_prev = k_cycle_get_32();
	for (;;) {
		motionBurst_t m;
		int rc = pmw3901_read_motion_burst(PMW3901_DEV, &m);
		uint32_t t_now = k_cycle_get_32();
		float dt = (float)k_cyc_to_us_floor32(t_now - t_prev) * 1e-6f;
		t_prev = t_now;

		if (rc == 0 && dt > 1e-4f && dt < 0.5f) {
			/* remap sensor deltas -> drone body frame, convert counts -> angular rate (rad/s).
			 * drone +x (fwd) = -deltaX ; drone +y (left) = +deltaY  (validated on HW). */
			float ax = -(float)m.deltaX * FLOW_RAD_PER_COUNT / dt;
			float ay =  (float)m.deltaY * FLOW_RAD_PER_COUNT / dt;
			/* Per-sample low-pass at the sensor rate. k = 1 - exp(-dt/tau): a longer tau (smaller k)
			 * smooths harder + lags more. tau=0 -> k=1 -> pass-through (raw). Seeded on first sample. */
			static float axf, ayf; static bool lp_seed;
			if (!lp_seed) { axf = ax; ayf = ay; lp_seed = true; }
			float kx = (FLOW_LP_TAU_X > 0.0f) ? (1.0f - expf(-dt / FLOW_LP_TAU_X)) : 1.0f;
			float ky = (FLOW_LP_TAU_Y > 0.0f) ? (1.0f - expf(-dt / FLOW_LP_TAU_Y)) : 1.0f;
			axf += kx * (ax - axf);
			ayf += ky * (ay - ayf);
			k_mutex_lock(&flow_mtx, K_FOREVER);
			g_ax = axf; g_ay = ayf;
			g_squal = m.squal; g_squal_ok = (m.squal >= FLOW_MIN_SQUAL);
			g_last_ms = k_uptime_get();
			g_seq++;
			k_mutex_unlock(&flow_mtx);
		}
		k_msleep(FLOW_PERIOD_MS);
	}
}

int flow_init(void)
{
	const struct device *spi_dev  = DEVICE_DT_GET(DT_NODELABEL(spi2));
	const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(spi_dev) || !device_is_ready(gpio_dev)) {
		printk("flow: SPI/GPIO not ready -- flow disabled\n");
		return -ENODEV;
	}

	pmw3901_cfg.spi.bus = spi_dev;
	pmw3901_cfg.spi.config.operation =
		SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	pmw3901_cfg.spi.config.frequency = 2000000;   /* PMW3901 SPI max */
	pmw3901_cfg.spi.config.slave = 0;
	pmw3901_cfg.cs_gpio.port = gpio_dev;
	pmw3901_cfg.cs_gpio.pin = CS_GPIO_PIN;
	pmw3901_cfg.cs_gpio.dt_flags = GPIO_ACTIVE_LOW;
	/* v3: NRESET tied high (R37), MOTION -> FPGA -- no GPIO reset/LED here (driver does a soft POR) */
	pmw3901_cfg.reset_gpio.port = NULL;
	pmw3901_cfg.led_gpio.port = NULL;

	int rc = -1;
	for (int i = 0; i < 5 && rc != 0; i++) {
		rc = pmw3901_init(PMW3901_DEV);
		if (rc != 0) {
			k_msleep(50);
		}
	}
	if (rc != 0) {
		printk("flow: PMW3901 not detected (rc=%d) -- flow disabled\n", rc);
		return rc;
	}

	k_thread_create(&flow_thread, flow_stack, K_THREAD_STACK_SIZEOF(flow_stack),
			flow_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&flow_thread, "flow");
	printk("flow: PMW3901 up -- background read ~%d Hz (remap +x=-dX +y=+dY)\n", 1000 / FLOW_PERIOD_MS);
	return 0;
}

void flow_get(float *ang_x, float *ang_y, int *squal, bool *valid)
{
	k_mutex_lock(&flow_mtx, K_FOREVER);
	*ang_x = g_ax; *ang_y = g_ay; *squal = g_squal;
	bool fresh = (k_uptime_get() - g_last_ms) < FLOW_STALE_MS;
	*valid = g_squal_ok && fresh && (g_seq != 0);
	k_mutex_unlock(&flow_mtx);
}
