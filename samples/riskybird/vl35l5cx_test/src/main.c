/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/vl53l5cx.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#define I2C_NODE DT_NODELABEL(i2c0)
#define TOF0_NODE DT_ALIAS(tof0)
#define TOF1_NODE DT_ALIAS(tof1)
#define TOF2_NODE DT_ALIAS(tof2)
#define TOF3_NODE DT_ALIAS(tof3)

/* ADS7128 I2C address on RiskyBird board */
#define ADS7128_I2C_ADDR 0x17

/* ADS7128 I2C command opcodes (datasheet Table 9) */
#define ADS7128_CMD_REG_WRITE 0x08 /* Single register write */
#define ADS7128_CMD_REG_READ  0x10 /* Single register read */

/* ADS7128 registers needed for GPIO output */
#define ADS7128_PIN_CFG_ADDRESS       0x05
#define ADS7128_GPIO_CFG_ADDRESS      0x07
#define ADS7128_GPO_DRIVE_CFG_ADDRESS 0x09
#define ADS7128_GPO_VALUE_ADDRESS     0x0B

/* RiskyBird: 4x VL53L5 enables are ADS7128 GPIO1..GPIO4 */
#define VL53L5_SENSOR_COUNT 4
#define VL53L5_DEFAULT_ADDR_7BIT 0x29
#define VL53L5_BASE_NEW_ADDR_7BIT 0x31 /* sensors will become 0x31..0x34 */

static const uint8_t vl53l5_en_channels[VL53L5_SENSOR_COUNT] = { 1, 2, 3, 4 };
static uint8_t ads7128_gpo_shadow;

static int ads7128_read_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t *data)
{
	uint8_t tx[2] = { ADS7128_CMD_REG_READ, reg_addr };
	int ret = i2c_write(i2c_dev, tx, sizeof(tx), ADS7128_I2C_ADDR);
	if (ret != 0) {
		return ret;
	}
	return i2c_read(i2c_dev, data, 1, ADS7128_I2C_ADDR);
}

static int ads7128_write_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t data)
{
	uint8_t buf[3] = { ADS7128_CMD_REG_WRITE, reg_addr, data };
	return i2c_write(i2c_dev, buf, sizeof(buf), ADS7128_I2C_ADDR);
}

static int ads7128_config_gpio_output_pushpull(const struct device *i2c_dev, uint8_t channel)
{
	int ret;
	uint8_t pin_cfg, gpio_cfg, drive_cfg;

	if (channel > 7) {
		return -EINVAL;
	}

	ret = ads7128_read_reg(i2c_dev, ADS7128_PIN_CFG_ADDRESS, &pin_cfg);
	if (ret != 0) {
		return ret;
	}
	pin_cfg |= (uint8_t)(1U << channel);
	ret = ads7128_write_reg(i2c_dev, ADS7128_PIN_CFG_ADDRESS, pin_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = ads7128_read_reg(i2c_dev, ADS7128_GPIO_CFG_ADDRESS, &gpio_cfg);
	if (ret != 0) {
		return ret;
	}
	gpio_cfg |= (uint8_t)(1U << channel);
	ret = ads7128_write_reg(i2c_dev, ADS7128_GPIO_CFG_ADDRESS, gpio_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = ads7128_read_reg(i2c_dev, ADS7128_GPO_DRIVE_CFG_ADDRESS, &drive_cfg);
	if (ret != 0) {
		return ret;
	}
	drive_cfg |= (uint8_t)(1U << channel);
	ret = ads7128_write_reg(i2c_dev, ADS7128_GPO_DRIVE_CFG_ADDRESS, drive_cfg);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int ads7128_write_gpio(const struct device *i2c_dev, uint8_t channel, uint8_t value)
{
	if (channel > 7) {
		return -EINVAL;
	}

	if (value != 0U) {
		ads7128_gpo_shadow |= (uint8_t)(1U << channel);
	} else {
		ads7128_gpo_shadow &= (uint8_t)~(1U << channel);
	}

	return ads7128_write_reg(i2c_dev, ADS7128_GPO_VALUE_ADDRESS, ads7128_gpo_shadow);
}

static int ads7128_force_low_set(const struct device *i2c_dev)
{
	/* Force other sensor enables low so only one device is awake at a time.
	 * This matches your original bring-up constraint.
	 */
	const uint8_t gpios_to_force_low[] = { 1, 2, 3, 4, 6 };
	int ret;

	for (size_t i = 0; i < sizeof(gpios_to_force_low); i++) {
		uint8_t ch = gpios_to_force_low[i];
		ret = ads7128_config_gpio_output_pushpull(i2c_dev, ch);
		if (ret != 0) {
			printf("ERROR: ADS7128 configure GPIO%u failed (%d)\n", ch, ret);
			return ret;
		}
	}

	/* Atomically force all GPIO outputs low (avoid per-bit RMW jitter at boot). */
	ads7128_gpo_shadow = 0;
	ret = ads7128_write_reg(i2c_dev, ADS7128_GPO_VALUE_ADDRESS, ads7128_gpo_shadow);
	if (ret != 0) {
		printf("ERROR: ADS7128 write GPO_VALUE=0x00 failed (%d)\n", ret);
		return ret;
	}

	/* Let enables settle (important after cold power-on). */
	k_msleep(20);
	return 0;
}

static int vl53l5cx_write_reg8(const struct device *i2c_dev, uint8_t addr7, uint16_t reg, uint8_t value)
{
	uint8_t tx[3];
	tx[0] = (uint8_t)(reg >> 8);
	tx[1] = (uint8_t)(reg & 0xFF);
	tx[2] = value;
	return i2c_write(i2c_dev, tx, sizeof(tx), addr7);
}

static int vl53l5cx_read_reg8(const struct device *i2c_dev, uint8_t addr7, uint16_t reg, uint8_t *out)
{
	uint8_t regbuf[2];

	regbuf[0] = (uint8_t)(reg >> 8);
	regbuf[1] = (uint8_t)(reg & 0xFF);

	return i2c_write_read(i2c_dev, addr7, regbuf, sizeof(regbuf), out, 1);
}

static int vl53l5cx_probe_id(const struct device *i2c_dev, uint8_t addr7, uint16_t *out_id)
{
	int ret;
	uint8_t dev_id;
	uint8_t rev_id;

	/* Use the same idea as ST's is_alive(): select page 0, read regs 0/1, restore page 2. */
	ret = vl53l5cx_write_reg8(i2c_dev, addr7, 0x7FFF, 0x00);
	if (ret != 0) {
		return ret;
	}

	ret = vl53l5cx_read_reg8(i2c_dev, addr7, 0x0000, &dev_id);
	if (ret != 0) {
		(void)vl53l5cx_write_reg8(i2c_dev, addr7, 0x7FFF, 0x02);
		return ret;
	}

	ret = vl53l5cx_read_reg8(i2c_dev, addr7, 0x0001, &rev_id);
	(void)vl53l5cx_write_reg8(i2c_dev, addr7, 0x7FFF, 0x02);
	if (ret != 0) {
		return ret;
	}

	if (out_id != NULL) {
		*out_id = (uint16_t)(((uint16_t)dev_id << 8) | rev_id);
	}

	/* VL53L5CX is expected to return 0xF0 0x02 (0xF002). */
	if (dev_id != 0xF0U) {
		return -ENODEV;
	}

	return 0;
}

static int vl53l5cx_readdress_bootstrap(const struct device *i2c_dev,
					uint8_t old_addr7,
					uint8_t new_addr7)
{
	int ret;

	/* Match ST ULD sequence:
	 * - write 0x7fff = 0x00
	 * - write 0x0004 = (addr8 >> 1) == new_addr7
	 * - write 0x7fff = 0x02 (restore default page)
	 *
	 * Note: On some devices/busses the address change can take effect
	 * immediately after the 0x0004 write. In that case, any further writes to
	 * the old address may NACK. Therefore we treat the final restore write as
	 * best-effort and attempt it on both old and new addresses.
	 */
	ret = vl53l5cx_write_reg8(i2c_dev, old_addr7, 0x7FFF, 0x00);
	if (ret != 0) {
		printf("bootstrap: write 0x7FFF=0x00 failed (ret=%d)\n", ret);
		return ret;
	}
	k_msleep(2);

	ret = vl53l5cx_write_reg8(i2c_dev, old_addr7, 0x0004, new_addr7);
	if (ret != 0) {
		printf("bootstrap: write 0x0004=0x%02X failed (ret=%d)\n", new_addr7, ret);
		return ret;
	}
	k_msleep(2);

	/* Best effort restore (do not fail address change if this NACKs). */
	ret = vl53l5cx_write_reg8(i2c_dev, new_addr7, 0x7FFF, 0x02);
	if (ret != 0) {
		int ret_old = vl53l5cx_write_reg8(i2c_dev, old_addr7, 0x7FFF, 0x02);
		printf("bootstrap: restore page failed (new ret=%d, old ret=%d) - continuing\n", ret, ret_old);
	}

	return 0;
}

static int enable_one_sensor(const struct device *i2c_dev, uint8_t en_channel)
{
	int ret = ads7128_force_low_set(i2c_dev);
	if (ret != 0) {
		return ret;
	}
	ret = ads7128_write_gpio(i2c_dev, en_channel, 1);
	if (ret != 0) {
		printf("ERROR: ADS7128 set GPIO%u HIGH failed (%d)\n", en_channel, ret);
		return ret;
	}
	/* Cold-boot sensors may need longer before they ACK. */
	k_msleep(100);
	return 0;
}

static int wait_for_any_ack(const struct device *i2c_dev,
			    uint8_t addr_a,
			    uint8_t addr_b,
			    uint32_t timeout_ms)
{
	int ret_a, ret_b;
	int64_t start = k_uptime_get();
	uint16_t id;

	do {
		ret_a = vl53l5cx_probe_id(i2c_dev, addr_a, &id);
		ret_b = vl53l5cx_probe_id(i2c_dev, addr_b, &id);
		if (ret_a == 0 || ret_b == 0) {
			return 0;
		}
		k_msleep(10);
	} while ((k_uptime_get() - start) < timeout_ms);

	return -ETIMEDOUT;
}

static int program_one_sensor_address(const struct device *i2c_dev,
				      uint8_t sensor_idx,
				      uint8_t new_addr7)
{
	int ret;
	uint16_t id;

	printf("\n=== Sensor %u (enable GPIO%u) target addr 0x%02X ===\n",
	       sensor_idx, vl53l5_en_channels[sensor_idx], new_addr7);

	ret = enable_one_sensor(i2c_dev, vl53l5_en_channels[sensor_idx]);
	if (ret != 0) {
		return ret;
	}

	/* Give the device some time to show up on either address after cold boot. */
	ret = wait_for_any_ack(i2c_dev, VL53L5_DEFAULT_ADDR_7BIT, new_addr7, 500);
	if (ret != 0) {
		printf("ERROR: timeout waiting for ACK at 0x%02X or 0x%02X\n",
		       VL53L5_DEFAULT_ADDR_7BIT, new_addr7);
	}

	ret = vl53l5cx_probe_id(i2c_dev, new_addr7, &id);
	printf("Probe: target  0x%02X -> %s (ret=%d id=0x%04X)\n",
	       new_addr7, (ret == 0) ? "OK" : "NO", ret, (ret == 0) ? id : 0);
	if (ret == 0) {
		printf("Already at 0x%02X, skipping reprogram.\n", new_addr7);
		return 0;
	}

	/* Otherwise, must be at default address while only this sensor is enabled. */
	ret = vl53l5cx_probe_id(i2c_dev, VL53L5_DEFAULT_ADDR_7BIT, &id);
	printf("Probe: default 0x%02X -> %s (ret=%d id=0x%04X)\n",
	       VL53L5_DEFAULT_ADDR_7BIT, (ret == 0) ? "OK" : "NO", ret, (ret == 0) ? id : 0);
	if (ret != 0) {
		printf("ERROR: not responding at default 0x%02X and not at 0x%02X\n",
		       VL53L5_DEFAULT_ADDR_7BIT, new_addr7);
		return -ENODEV;
	}

	printf("Reprogramming 0x%02X -> 0x%02X...\n", VL53L5_DEFAULT_ADDR_7BIT, new_addr7);
	for (int attempt = 1; attempt <= 5; attempt++) {
		ret = vl53l5cx_readdress_bootstrap(i2c_dev, VL53L5_DEFAULT_ADDR_7BIT, new_addr7);
		if (ret == 0) {
			break;
		}
		printf("bootstrap attempt %d/5 failed (ret=%d), retrying...\n", attempt, ret);
		k_msleep(50);
	}
	if (ret != 0) {
		printf("ERROR: bootstrap readdress failed (ret=%d)\n", ret);
		return ret;
	}
	k_msleep(10);

	ret = vl53l5cx_probe_id(i2c_dev, new_addr7, &id);
	printf("Probe after: target  0x%02X -> %s (ret=%d id=0x%04X)\n",
	       new_addr7, (ret == 0) ? "OK" : "NO", ret, (ret == 0) ? id : 0);

	return 0;
}

int main(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
	const struct device *tof_dev[VL53L5_SENSOR_COUNT] = {
		DEVICE_DT_GET(TOF0_NODE),
		DEVICE_DT_GET(TOF1_NODE),
		DEVICE_DT_GET(TOF2_NODE),
		DEVICE_DT_GET(TOF3_NODE),
	};
	int ret;
	struct sensor_value dist;

	printf("VL53L5CX 4-sensor integration test (ADS7128 enable, address program)\n");

	if (!device_is_ready(i2c_dev)) {
		printf("ERROR: I2C device not ready\n");
		return 1;
	}

	/* Ensure GPIOs are configured and all sensors are off initially. */
	ret = ads7128_force_low_set(i2c_dev);
	if (ret != 0) {
		return 1;
	}

	for (uint8_t i = 0; i < VL53L5_SENSOR_COUNT; i++) {
		if (!device_is_ready(tof_dev[i])) {
			printf("ERROR: ToF device tof%u not ready (check overlay aliases)\n", i);
			return 1;
		}
	}

	/* Step 1: Program addresses 0x31..0x34, one sensor at a time. */
	for (uint8_t i = 0; i < VL53L5_SENSOR_COUNT; i++) {
		uint8_t new_addr7 = (uint8_t)(VL53L5_BASE_NEW_ADDR_7BIT + i);
		ret = program_one_sensor_address(i2c_dev, i, new_addr7);
		if (ret != 0) {
			printf("ERROR: address program failed for sensor %u (ret=%d)\n", i, ret);
			return 1;
		}
	}

	/* Step 2: Initialize each sensor at its final address, one at a time. */
	for (uint8_t i = 0; i < VL53L5_SENSOR_COUNT; i++) {
		ret = enable_one_sensor(i2c_dev, vl53l5_en_channels[i]);
		if (ret != 0) {
			return 1;
		}

		printf("Initializing tof%u...\n", i);
		ret = vl53l5cx_reinit(tof_dev[i]);
		if (ret != 0) {
			printf("ERROR: vl53l5cx_reinit(tof%u) failed (%d)\n", i, ret);
			return 1;
		}
	}

	while (1) {
		for (uint8_t i = 0; i < VL53L5_SENSOR_COUNT; i++) {
			ret = enable_one_sensor(i2c_dev, vl53l5_en_channels[i]);
			if (ret != 0) {
				return 1;
			}

			ret = sensor_sample_fetch(tof_dev[i]);
			if (ret != 0) {
				printf("ERROR: tof%u sample_fetch failed (%d)\n", i, ret);
				/* If power gating drops firmware/config, try reinit once. */
				ret = vl53l5cx_reinit(tof_dev[i]);
				if (ret != 0) {
					printf("ERROR: tof%u reinit retry failed (%d)\n", i, ret);
					continue;
				}
				ret = sensor_sample_fetch(tof_dev[i]);
				if (ret != 0) {
					printf("ERROR: tof%u sample_fetch failed after reinit (%d)\n", i, ret);
					continue;
				}
			}

			ret = sensor_channel_get(tof_dev[i], SENSOR_CHAN_DISTANCE, &dist);
			if (ret != 0) {
				printf("ERROR: tof%u channel_get failed (%d)\n", i, ret);
				continue;
			}

			printf("tof%u center: %d mm\n", i, dist.val1);

			struct vl53l5cx_grid grid;
			ret = vl53l5cx_get_grid(tof_dev[i], &grid);
			if (ret == 0) {
				uint8_t side = (grid.resolution == 64U) ? 8U : 4U;
				printf("tof%u grid (%ux%u) row0:", i, side, side);
				for (uint8_t k = 0; k < side; k++) {
					printf(" %d", grid.distance_mm[k]);
				}
				printf("\n");
			}
		}

		k_msleep(500);
	}
}
