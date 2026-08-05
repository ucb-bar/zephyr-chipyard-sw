/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>

/* Get device tree nodes for accelerometer and gyroscope
 * The device tree should define aliases:
 *   aliases {
 *     bmi088-accel = &bmi088_accel;
 *     bmi088-gyro = &bmi088_gyro;
 *   };
 */
#define BMI088_ACCEL_NODE DT_ALIAS(bmi088_accel)
#define BMI088_GYRO_NODE DT_ALIAS(bmi088_gyro)

#if !DT_NODE_EXISTS(BMI088_ACCEL_NODE)
#error "BMI088 accelerometer alias 'bmi088-accel' not defined in device tree"
#endif

#if !DT_NODE_EXISTS(BMI088_GYRO_NODE)
#error "BMI088 gyroscope alias 'bmi088-gyro' not defined in device tree"
#endif

static const struct device *accel_dev = DEVICE_DT_GET(BMI088_ACCEL_NODE);
static const struct device *gyro_dev = DEVICE_DT_GET(BMI088_GYRO_NODE);

int main(void)
{
	struct sensor_value accel[3], gyro[3], temp;
	int ret;

	printf("BMI088 Sensor Test Application\n");
	printf("==============================\n\n");

	/* Check if accelerometer device is ready */
	if (!device_is_ready(accel_dev)) {
		printf("Error: Accelerometer device is not ready\n");
		return 1;
	}
	printf("Accelerometer device ready: %s\n", accel_dev->name);

	/* Check if gyroscope device is ready */
	if (!device_is_ready(gyro_dev)) {
		printf("Error: Gyroscope device is not ready\n");
		return 1;
	}
	printf("Gyroscope device ready: %s\n\n", gyro_dev->name);

	/* Main loop - read sensor data periodically */
	while (1) {
		/* Read accelerometer data */
		ret = sensor_sample_fetch(accel_dev);
		if (ret < 0) {
			printf("Error: Failed to fetch accelerometer sample: %d\n", ret);
		} else {
			ret = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
			if (ret < 0) {
				printf("Error: Failed to get accelerometer data: %d\n", ret);
			} else {
				printf("Accelerometer - X: %.6f m/s^2, Y: %.6f m/s^2, Z: %.6f m/s^2\n",
				       sensor_value_to_double(&accel[0]),
				       sensor_value_to_double(&accel[1]),
				       sensor_value_to_double(&accel[2]));
			}
		}

		/* Read gyroscope data */
		ret = sensor_sample_fetch(gyro_dev);
		if (ret < 0) {
			printf("Error: Failed to fetch gyroscope sample: %d\n", ret);
		} else {
			ret = sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
			if (ret < 0) {
				printf("Error: Failed to get gyroscope data: %d\n", ret);
			} else {
				printf("Gyroscope     - X: %.6f rad/s, Y: %.6f rad/s, Z: %.6f rad/s\n",
				       sensor_value_to_double(&gyro[0]),
				       sensor_value_to_double(&gyro[1]),
				       sensor_value_to_double(&gyro[2]));
			}
		}

		/* Read temperature from accelerometer */
		ret = sensor_channel_get(accel_dev, SENSOR_CHAN_DIE_TEMP, &temp);
		if (ret < 0) {
			printf("Error: Failed to get temperature: %d\n", ret);
		} else {
			printf("Temperature    - %.2f °C\n", sensor_value_to_double(&temp));
		}

		printf("---\n");

		/* Wait before next reading */
		k_msleep(10);
	}

	return 0;
}

