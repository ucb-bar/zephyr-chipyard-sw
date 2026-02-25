/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Enable ToF at 0x29 via XSHUT (ADS7128 GPIO 6), then run VL53L0X distance loop.
 * No address reassignment.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <errno.h>
#include <stdio.h>

#define I2C_NODE            DT_NODELABEL(i2c0)

#define ADS7128_ADDR        0x17
#define ADS7128_REG_GPIO_CFG   0x20
#define ADS7128_REG_GPIO_DATA  0x21
#define TOF_XSHUT_GPIO  6

static int ads7128_write_reg(const struct device *i2c_dev,
			     uint8_t reg, uint8_t value)
{
	uint8_t buf[2] = { reg, value };
	return i2c_write(i2c_dev, buf, sizeof(buf), ADS7128_ADDR);
}

static int ads7128_read_reg(const struct device *i2c_dev,
			    uint8_t reg, uint8_t *value)
{
	int ret = i2c_write(i2c_dev, &reg, 1, ADS7128_ADDR);
	if (ret < 0) return ret;
	return i2c_read(i2c_dev, value, 1, ADS7128_ADDR);
}

static int ads7128_xshut_on(const struct device *i2c_dev)
{
	int ret;
	uint8_t data;
	ret = ads7128_read_reg(i2c_dev, ADS7128_REG_GPIO_DATA, &data);
	if (ret < 0) return ret;
	data |= BIT(TOF_XSHUT_GPIO);
	return ads7128_write_reg(i2c_dev, ADS7128_REG_GPIO_DATA, data);
}

/*
 * Enable ToF XSHUT (GPIO 6 high) before VL53L0X driver inits.
 */
static int enable_tof_xshut_init(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

	for (int i = 0; i < 25; i++) {
		if (device_is_ready(i2c_dev)) break;
		k_msleep(1);
	}
	if (!device_is_ready(i2c_dev)) return -ENODEV;

	if (ads7128_write_reg(i2c_dev, ADS7128_REG_GPIO_CFG, BIT(TOF_XSHUT_GPIO)) < 0)
		return -EIO;
	if (ads7128_write_reg(i2c_dev, ADS7128_REG_GPIO_DATA, 0) < 0)
		return -EIO;
	k_msleep(10);
	if (ads7128_xshut_on(i2c_dev) < 0) return -EIO;
	/* VL53L0X needs time after XSHUT before StaticInit (avoid -12 timeout). */
	k_msleep(2000);
	return 0;
}

SYS_INIT(enable_tof_xshut_init, POST_KERNEL, 51);

int main(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
	const struct device *vl53l0x;
	struct sensor_value distance;
	int ret;

	if (!device_is_ready(i2c_dev)) {
		printf("I2C not ready.\n");
		return 1;
	}

	vl53l0x = DEVICE_DT_GET_ONE(st_vl53l0x);
	if (!device_is_ready(vl53l0x)) {
		printf("VL53L0X not ready (ToF at 0x29). Driver init failed at boot.\n");
		printf("Add CONFIG_LOG=y and CONFIG_VL53L0X_LOG_LEVEL_DBG=y in prj.conf to see why.\n");
		return 1;
	}

	printf("VL53L0X at 0x29: distance loop\n\n");
	while (1) {
		ret = sensor_sample_fetch(vl53l0x);
		if (ret < 0) {
			printf("sample_fetch failed (%d)\n", ret);
			k_msleep(1000);
			continue;
		}
		ret = sensor_channel_get(vl53l0x, SENSOR_CHAN_DISTANCE, &distance);
		if (ret < 0) {
			printf("channel_get failed (%d)\n", ret);
		} else {
			printf("distance = %lld mm\n", sensor_value_to_milli(&distance));
		}
		k_msleep(1000);
	}
	return 0;
}
