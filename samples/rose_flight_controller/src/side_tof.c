/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Side wall sensors (see side_tof.h). Bring-up mirrors the validated samples/riskybird/vl35l5cx_test
 * flow (enable one via ADS7128 GPIO, readdress off 0x29, persistence-aware), but:
 *   - preserves ADS7128 GPIO6 (the down VL53L1X) via read-modify-write, and
 *   - after readdressing, enables ALL four (LPn retains state) so they coexist at 0x31-0x34, then
 *     reads them on a background thread -> the control loop never blocks on these I2C transactions.
 */
#include "side_tof.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/vl53l5cx.h>
#include <zephyr/sys/printk.h>

#define ADS7128_I2C_ADDR      0x17
#define ADS7128_CMD_REG_WRITE 0x08
#define ADS7128_CMD_REG_READ  0x10
#define ADS7128_PIN_CFG       0x05
#define ADS7128_GPIO_CFG      0x07
#define ADS7128_GPO_DRIVE_CFG 0x09
#define ADS7128_GPO_VALUE     0x0B

#define DOWN_XSHUT_CH   6      /* preserve: the down VL53L1X (managed by board_sensor_init) */
#define VL53L5_DEFAULT  0x29

/* Sensor table: enable channel, target address, body direction. (See side_tof.h for the map.) */
enum { S_BACK, S_FRONT, S_RIGHT, S_LEFT, N_SIDE };
static const uint8_t  s_ch[N_SIDE]   = { 1, 2, 3, 4 };
static const uint8_t  s_addr[N_SIDE] = { 0x31, 0x32, 0x33, 0x34 };
static const char    *s_name[N_SIDE] = { "back", "front", "right", "left" };
static const struct device *s_dev[N_SIDE];

static const struct device *g_bus;

/* ---- ADS7128 (RMW so we never disturb GPIO6 / the down sensor) ---- */
static int ads_rd(uint8_t reg, uint8_t *v)
{
	uint8_t tx[2] = { ADS7128_CMD_REG_READ, reg };
	int rc = i2c_write(g_bus, tx, sizeof(tx), ADS7128_I2C_ADDR);
	return rc ? rc : i2c_read(g_bus, v, 1, ADS7128_I2C_ADDR);
}
static int ads_wr(uint8_t reg, uint8_t v)
{
	uint8_t tx[3] = { ADS7128_CMD_REG_WRITE, reg, v };
	return i2c_write(g_bus, tx, sizeof(tx), ADS7128_I2C_ADDR);
}
static void ads_gpio_output(uint8_t ch)
{
	uint8_t v;
	if (ads_rd(ADS7128_PIN_CFG, &v) == 0)       { ads_wr(ADS7128_PIN_CFG, v | (1U << ch)); }
	if (ads_rd(ADS7128_GPIO_CFG, &v) == 0)      { ads_wr(ADS7128_GPIO_CFG, v | (1U << ch)); }
	if (ads_rd(ADS7128_GPO_DRIVE_CFG, &v) == 0) { ads_wr(ADS7128_GPO_DRIVE_CFG, v | (1U << ch)); }
}
/* Set the side-enable bits (1-4) to `mask`, leaving GPIO6 (down) untouched. */
static void ads_set_sides(uint8_t mask)
{
	uint8_t v;
	if (ads_rd(ADS7128_GPO_VALUE, &v) != 0) {
		return;
	}
	v &= ~((1U << 1) | (1U << 2) | (1U << 3) | (1U << 4));   /* clear side bits, keep GPIO6 */
	v |= (uint8_t)(mask & ((1U << 1) | (1U << 2) | (1U << 3) | (1U << 4)));
	ads_wr(ADS7128_GPO_VALUE, v);
}
static inline uint8_t bit(uint8_t ch) { return (uint8_t)(1U << ch); }

/* ---- VL53L5CX raw readdress (ULD sequence), used before driver init ---- */
static int l5_wr8(uint8_t addr7, uint16_t reg, uint8_t val)
{
	uint8_t tx[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val };
	return i2c_write(g_bus, tx, sizeof(tx), addr7);
}
static bool l5_alive(uint8_t addr7)
{
	uint8_t id = 0, rb[2] = { 0x00, 0x00 };
	(void)l5_wr8(addr7, 0x7FFF, 0x00);
	int rc = i2c_write_read(g_bus, addr7, rb, 2, &id, 1);
	(void)l5_wr8(addr7, 0x7FFF, 0x02);
	return (rc == 0 && id == 0xF0);
}
static int l5_readdress(uint8_t old7, uint8_t new7)
{
	int rc = l5_wr8(old7, 0x7FFF, 0x00);
	if (rc) { return rc; }
	k_msleep(2);
	rc = l5_wr8(old7, 0x0004, new7);   /* program new 7-bit address */
	k_msleep(2);
	(void)l5_wr8(new7, 0x7FFF, 0x02);  /* best-effort page restore at the new address */
	return rc;
}

/* ---- background reader ---- */
static struct side_walls g_walls;
K_MUTEX_DEFINE(g_walls_mtx);
K_THREAD_STACK_DEFINE(g_side_stack, 4096);
static struct k_thread g_side_thread;
static bool g_running[N_SIDE];

/* Wall distance for one sensor = AVERAGE of the valid zones over the top WALL_CLEAR_ROWS rows of
 * the grid. The live-heatmap survey showed these rows are the least occluded on all four sensors
 * (higher rows are blocked by the PCB / motor ports), and that an "invalid" zone almost always
 * means out-of-range -- i.e. no wall there -- so invalid zones are simply excluded from the mean.
 * Returns the mean distance (mm) of the valid clear-row zones, or -1 if none are valid (no wall). */
#ifndef WALL_CLEAR_ROWS
#define WALL_CLEAR_ROWS 2
#endif
static int16_t read_one(int i)
{
	if (!g_running[i] || sensor_sample_fetch(s_dev[i]) != 0) {
		return -1;
	}
	struct vl53l5cx_grid g;
	if (vl53l5cx_get_grid(s_dev[i], &g) != 0) {
		return -1;
	}
	int side = (g.resolution == 64) ? 8 : 4;   /* 8x8 or 4x4 */
	int n = WALL_CLEAR_ROWS * side;             /* leading zones = top WALL_CLEAR_ROWS rows */
	if (n > g.resolution) {
		n = g.resolution;
	}
	int32_t sum = 0;
	int cnt = 0;
	for (int z = 0; z < n; z++) {
		if (g.target_status[z] == 5 && g.distance_mm[z] > 0) {   /* 5 = valid; else out-of-range */
			sum += g.distance_mm[z];
			cnt++;
		}
	}
	return (cnt > 0) ? (int16_t)(sum / cnt) : -1;
}

#if defined(ROSE_BUMPER_GRID) && ROSE_BUMPER_GRID
#include <stdio.h>
/* Stream the full flattened grid for one sensor (uses the last read_one() fetch). One line per
 * sensor: "GRID <name>: v0 v1 ... vN" row-major, distance_mm where target_status==5 (valid) else -1.
 * For occlusion mapping via the live heatmap; enabled only in validation builds (-DROSE_BUMPER_GRID=1). */
static void dump_grid(int i)
{
	struct vl53l5cx_grid g;
	if (!g_running[i] || vl53l5cx_get_grid(s_dev[i], &g) != 0) {
		return;
	}
	int n = (g.resolution <= 64) ? g.resolution : 64;
	char buf[480];
	int off = snprintf(buf, sizeof(buf), "GRID %s:", s_name[i]);
	for (int z = 0; z < n && off < (int)sizeof(buf) - 8; z++) {
		int16_t v = (g.target_status[z] == 5) ? g.distance_mm[z] : -1;   /* 5 = valid */
		off += snprintf(buf + off, sizeof(buf) - off, " %d", v);
	}
	printk("%s\n", buf);
}
#endif

static void side_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (;;) {
		int16_t fr = read_one(S_FRONT), bk = read_one(S_BACK);
		int16_t lf = read_one(S_LEFT), rt = read_one(S_RIGHT);
		k_mutex_lock(&g_walls_mtx, K_FOREVER);
		g_walls.front_mm = fr; g_walls.back_mm = bk;
		g_walls.left_mm = lf;  g_walls.right_mm = rt;
		g_walls.seq++;
		k_mutex_unlock(&g_walls_mtx);
#if defined(ROSE_BUMPER_GRID) && ROSE_BUMPER_GRID
		for (int i = 0; i < N_SIDE; i++) {
			dump_grid(i);
		}
		/* the derived wall distances the controller will actually use (rows0-1 valid average) */
		printk("WALL: front=%d back=%d left=%d right=%d\n", fr, bk, lf, rt);
#endif
		k_msleep(30);   /* ~30 Hz aggregate; sensors range at ~15 Hz (8x8) */
	}
}

int side_tof_init(void)
{
	g_bus = DEVICE_DT_GET(DT_BUS(DT_ALIAS(side_front)));
	if (!device_is_ready(g_bus)) {
		printk("side_tof: I2C bus not ready\n");
		return 0;
	}
	s_dev[S_BACK]  = DEVICE_DT_GET(DT_ALIAS(side_back));
	s_dev[S_FRONT] = DEVICE_DT_GET(DT_ALIAS(side_front));
	s_dev[S_RIGHT] = DEVICE_DT_GET(DT_ALIAS(side_right));
	s_dev[S_LEFT]  = DEVICE_DT_GET(DT_ALIAS(side_left));

	for (int i = 0; i < N_SIDE; i++) {
		ads_gpio_output(s_ch[i]);
	}
	ads_set_sides(0);      /* all side sensors off (down/GPIO6 preserved) */
	k_msleep(20);

	/* Readdress each sensor one at a time (persistence-aware: skip if already at target). */
	int up = 0;
	for (int i = 0; i < N_SIDE; i++) {
		ads_set_sides(bit(s_ch[i]));   /* enable only this one */
		k_msleep(100);
		if (l5_alive(s_addr[i])) {
			/* already at target (persisted from a prior boot) */
		} else if (l5_alive(VL53L5_DEFAULT)) {
			for (int a = 0; a < 5 && !l5_alive(s_addr[i]); a++) {
				l5_readdress(VL53L5_DEFAULT, s_addr[i]);
				k_msleep(5);
			}
		}
		printk("side_tof: %s (GPIO%d) -> 0x%02x %s\n", s_name[i], s_ch[i], s_addr[i],
		       l5_alive(s_addr[i]) ? "OK" : "NOT FOUND");
	}

	/* Enable all four together (LPn retains address) so they coexist for reading. */
	ads_set_sides(bit(1) | bit(2) | bit(3) | bit(4));
	k_msleep(50);

	for (int i = 0; i < N_SIDE; i++) {
		int rc = vl53l5cx_reinit(s_dev[i]);
		g_running[i] = (rc == 0);
		if (rc == 0) {
			up++;
		} else {
			printk("side_tof: %s reinit failed (%d)\n", s_name[i], rc);
		}
	}
	printk("side_tof: %d/4 side sensors up\n", up);

	if (up > 0) {
		k_thread_create(&g_side_thread, g_side_stack, K_THREAD_STACK_SIZEOF(g_side_stack),
				side_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(9), 0, K_NO_WAIT);
		k_thread_name_set(&g_side_thread, "side_tof");
	}
	return up;
}

void side_tof_get(struct side_walls *out)
{
	k_mutex_lock(&g_walls_mtx, K_FOREVER);
	*out = g_walls;
	k_mutex_unlock(&g_walls_mtx);
}
