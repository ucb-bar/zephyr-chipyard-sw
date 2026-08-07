/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * riskybird v3 SIDE ToF bring-up (Phase 1: FRONT sensor only).
 *
 * The four side ToFs are external VL53L5CX (8x8) modules on connectors J2/J9/J10/J11, all sharing
 * I2C0 at default address 0x29 with the down VL53L1X, and enabled via the ADS7128 (0x17) GPIO XSHUT
 * lines. Verified wiring: front=J2=GPIO2, back=J9=GPIO1, left=J10=GPIO4, right=J11=GPIO3,
 * down=U9=GPIO6.
 *
 * This test powers ONLY the front sensor (GPIO2 high, all others held low) so it's alone at 0x29,
 * inits it as a VL53L5CX, and prints the CENTER-zone distance (the driver returns the center zone on
 * SENSOR_CHAN_DISTANCE, which conveniently ignores the PCB-occluded edge zones). Use it to (a) prove
 * the VL53L5CX path works and (b) WALL-VERIFY facing: put a wall in front -> distance should drop.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor/vl53l5cx.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>

/* ADS7128 I2C GPIO expander (0x17) raw register access (mirrors the flight controller). */
#define ADS7128_I2C_ADDR      0x17
#define ADS7128_CMD_REG_WRITE 0x08
#define ADS7128_CMD_REG_READ  0x10
#define ADS7128_PIN_CFG       0x05
#define ADS7128_GPIO_CFG      0x07
#define ADS7128_GPO_DRIVE_CFG 0x09
#define ADS7128_GPO_VALUE     0x0B

#define FRONT_XSHUT_CH        2                 /* GPIO2 = front (J2) */
static const uint8_t HOLD_LOW_CH[] = { 1, 3, 4, 6 };   /* back(J9), right(J11), left(J10), down(U9) */

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
	uint8_t v; int rc = ads7128_rd(bus, reg, &v);
	return rc ? rc : ads7128_wr(bus, reg, (uint8_t)(v | (1U << ch)));
}
static int ads7128_clr_bit(const struct device *bus, uint8_t reg, uint8_t ch)
{
	uint8_t v; int rc = ads7128_rd(bus, reg, &v);
	return rc ? rc : ads7128_wr(bus, reg, (uint8_t)(v & ~(1U << ch)));
}
static void ads7128_gpio_output(const struct device *bus, uint8_t ch)
{
	ads7128_set_bit(bus, ADS7128_PIN_CFG, ch);       /* GPIO mode */
	ads7128_set_bit(bus, ADS7128_GPIO_CFG, ch);      /* output */
	ads7128_set_bit(bus, ADS7128_GPO_DRIVE_CFG, ch); /* push-pull */
}

/* Enable ONLY the front sensor before the VL53L5CX driver inits (prio 90). */
static int side_tof_power(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(bus)) {
		printk("side_tof: I2C0 not ready\n");
		return 0;
	}
	/* Hold every other sensor in reset (XSHUT low). */
	for (size_t i = 0; i < ARRAY_SIZE(HOLD_LOW_CH); i++) {
		ads7128_gpio_output(bus, HOLD_LOW_CH[i]);
		ads7128_clr_bit(bus, ADS7128_GPO_VALUE, HOLD_LOW_CH[i]);
	}
	/* Power-cycle the front sensor's XSHUT low->high for a clean boot. */
	ads7128_gpio_output(bus, FRONT_XSHUT_CH);
	ads7128_clr_bit(bus, ADS7128_GPO_VALUE, FRONT_XSHUT_CH);
	k_msleep(10);
	ads7128_set_bit(bus, ADS7128_GPO_VALUE, FRONT_XSHUT_CH);
	k_msleep(10);
	printk("side_tof: front (J2/GPIO2) powered, others held in reset\n");
	return 0;
}
SYS_INIT(side_tof_power, POST_KERNEL, 80);   /* before CONFIG_SENSOR_INIT_PRIORITY (90) */

int main(void)
{
	const struct device *front = DEVICE_DT_GET(DT_ALIAS(side_front));

	k_msleep(300);
	if (!device_is_ready(front)) {
		printk("side_tof: FRONT device not ready\n");
		return -1;
	}
	/* VL53L5CX defers init (uploads ~84 KB firmware) -- run it now that the sensor is powered. */
	int rc = vl53l5cx_reinit(front);
	printk("side_tof: vl53l5cx_reinit rc=%d (%s)\n", rc,
	       rc == 0 ? "front ranging (8x8)" : "init FAILED");
	if (rc != 0) {
		return -1;
	}

	printk("side_tof: reading FRONT center-zone distance -- wave a wall in front to verify facing\n");
	for (int i = 0;; i++) {
		if (sensor_sample_fetch(front) != 0) {
			printk("side_tof: fetch error\n");
			k_msleep(100);
			continue;
		}
		struct sensor_value d;
		sensor_channel_get(front, SENSOR_CHAN_DISTANCE, &d);
		/* The VL53L5CX driver returns the center-zone distance in mm directly in val1. */
		printk("side_tof: FRONT center = %d mm\n", d.val1);
		k_msleep(100);   /* ~10 Hz print */
	}
	return 0;
}
