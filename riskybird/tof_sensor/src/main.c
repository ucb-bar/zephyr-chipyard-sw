/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	const struct device *const i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	const struct device *const vl53l0x_dev = DEVICE_DT_GET_ONE(st_vl53l0x);
	struct sensor_value distance;
	int ret;

	if (!device_is_ready(i2c0)) {
		printk("I2C0: device not ready\n");
	} else {
		/* Quick probe: does anything ACK at 0x29? */
		ret = i2c_write(i2c0, NULL, 0, 0x29);
		printk("I2C0: probe 0x29 %s (ret=%d)\n", (ret == 0) ? "ACK" : "NACK", ret);
	}

	if (!device_is_ready(vl53l0x_dev)) {
		printk("VL53L0X: device not ready\n");
		return 0;
	}

	printk("VL53L0X: device ready, starting measurements\n");

	while (1) {
		ret = sensor_sample_fetch(vl53l0x_dev);
		if (ret < 0) {
			printk("VL53L0X: sensor_sample_fetch failed (%d)\n", ret);
			k_msleep(1000);
			continue;
		}

		ret = sensor_channel_get(vl53l0x_dev, SENSOR_CHAN_DISTANCE, &distance);
		if (ret < 0) {
			printk("VL53L0X: sensor_channel_get failed (%d)\n", ret);
		} else {
			printk("VL53L0X: distance = %lld mm\n",
			       sensor_value_to_milli(&distance));
		}

		k_msleep(1000);
	}

	return 0;
}