/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "attitude_estimator.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdio.h>

LOG_MODULE_REGISTER(attitude_estimator, CONFIG_LOG_DEFAULT_LEVEL);

/* Get device tree nodes for accelerometer and gyroscope */
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

/* Configuration */
#ifndef CONFIG_ATTITUDE_ESTIMATOR_FREQUENCY
#define CONFIG_ATTITUDE_ESTIMATOR_FREQUENCY 200
#endif

#ifndef CONFIG_ATTITUDE_ESTIMATOR_STACK_SIZE
#define CONFIG_ATTITUDE_ESTIMATOR_STACK_SIZE 2048
#endif

#define ESTIMATOR_FREQUENCY CONFIG_ATTITUDE_ESTIMATOR_FREQUENCY
#define ESTIMATOR_PERIOD_MS (1000 / ESTIMATOR_FREQUENCY)
#define ESTIMATOR_PERIOD_US ((1000000LL) / ESTIMATOR_FREQUENCY)
#define COMPLEMENTARY_FILTER_ALPHA 0.98f  /* Gyro weight (0-1), higher = trust gyro more */

/* Thread and synchronization */
static struct k_thread estimator_thread;
static K_THREAD_STACK_DEFINE(estimator_stack, CONFIG_ATTITUDE_ESTIMATOR_STACK_SIZE);
static volatile bool estimator_running = false;
static struct k_mutex attitude_mutex;

/* Current attitude estimate */
static struct attitude current_attitude = {0.0f, 0.0f, 0.0f};

/**
 * @brief Complementary filter for attitude estimation
 *
 * This combines accelerometer (for roll/pitch) and gyroscope (for all axes)
 * to estimate the drone's attitude.
 *
 * @param accel Accelerometer data (m/s^2)
 * @param gyro Gyroscope data (rad/s)
 * @param dt Time step in seconds
 * @param attitude Output attitude estimate
 */
static void complementary_filter(const float accel[3], const float gyro[3],
				 float dt, struct attitude *attitude)
{
	/* Calculate roll and pitch from accelerometer (assuming gravity is dominant) */
	float accel_roll = atan2f(accel[1], accel[2]);
	float accel_pitch = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));

	/* Integrate gyroscope to get attitude change */
	float gyro_roll = attitude->roll + gyro[0] * dt;
	float gyro_pitch = attitude->pitch + gyro[1] * dt;
	float gyro_yaw = attitude->yaw + gyro[2] * dt;

	/* Complementary filter: blend accelerometer and gyroscope */
	attitude->roll = COMPLEMENTARY_FILTER_ALPHA * gyro_roll +
			 (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;
	attitude->pitch = COMPLEMENTARY_FILTER_ALPHA * gyro_pitch +
			  (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch;
	attitude->yaw = gyro_yaw; /* Yaw from gyro only (no magnetometer) */
}

/**
 * @brief Attitude estimator thread
 */
static void attitude_estimator_thread(void *arg1, void *arg2, void *arg3)
{
	struct sensor_value accel[3], gyro[3];
	int ret;
	float accel_f[3], gyro_f[3];
	int64_t last_time = k_uptime_get();
	int64_t current_time;
	float dt;

	LOG_INF("Attitude estimator thread started");

	while (estimator_running) {
		current_time = k_uptime_get();
		dt = (current_time - last_time) / 1000.0f;  /* Convert to seconds */
		last_time = current_time;

		/* Read accelerometer */
		ret = sensor_sample_fetch(accel_dev);
		if (ret < 0) {
			LOG_ERR("Failed to fetch accelerometer sample: %d", ret);
			k_msleep(ESTIMATOR_PERIOD_MS);
			continue;
		}

		ret = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (ret < 0) {
			LOG_ERR("Failed to get accelerometer data: %d", ret);
			k_msleep(ESTIMATOR_PERIOD_MS);
			continue;
		}

		/* Read gyroscope */
		ret = sensor_sample_fetch(gyro_dev);
		if (ret < 0) {
			LOG_ERR("Failed to fetch gyroscope sample: %d", ret);
			k_msleep(ESTIMATOR_PERIOD_MS);
			continue;
		}

		ret = sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		if (ret < 0) {
			LOG_ERR("Failed to get gyroscope data: %d", ret);
			k_msleep(ESTIMATOR_PERIOD_MS);
			continue;
		}

		/* Convert sensor values to float */
		accel_f[0] = sensor_value_to_double(&accel[0]);
		accel_f[1] = sensor_value_to_double(&accel[1]);
		accel_f[2] = sensor_value_to_double(&accel[2]);

		gyro_f[0] = sensor_value_to_double(&gyro[0]);
		gyro_f[1] = sensor_value_to_double(&gyro[1]);
		gyro_f[2] = sensor_value_to_double(&gyro[2]);

		/* Update attitude estimate */
		k_mutex_lock(&attitude_mutex, K_FOREVER);
		complementary_filter(accel_f, gyro_f, dt, &current_attitude);
		k_mutex_unlock(&attitude_mutex);

		/* Sleep until next iteration - use microseconds for better precision */
		if (ESTIMATOR_PERIOD_MS > 0) {
			k_msleep(ESTIMATOR_PERIOD_MS);
		} else {
			k_usleep(ESTIMATOR_PERIOD_US);
		}
	}

	LOG_INF("Attitude estimator thread stopped");
}

int attitude_estimator_init(void)
{
	int ret;

	/* Check if accelerometer device is ready */
	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Accelerometer device is not ready");
		return -ENODEV;
	}

	/* Check if gyroscope device is ready */
	if (!device_is_ready(gyro_dev)) {
		LOG_ERR("Gyroscope device is not ready");
		return -ENODEV;
	}

	/* Initialize mutex */
	k_mutex_init(&attitude_mutex);

	LOG_INF("Attitude estimator initialized (frequency: %d Hz)", ESTIMATOR_FREQUENCY);

	return 0;
}

int attitude_estimator_get(struct attitude *attitude)
{
	if (attitude == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&attitude_mutex, K_FOREVER);
	*attitude = current_attitude;
	k_mutex_unlock(&attitude_mutex);

	return 0;
}

int attitude_estimator_start(void)
{
	if (estimator_running) {
		return -EALREADY;
	}

	estimator_running = true;

	k_thread_create(&estimator_thread, estimator_stack,
			K_THREAD_STACK_SIZEOF(estimator_stack),
			attitude_estimator_thread,
			NULL, NULL, NULL,
			K_PRIO_COOP(7), 0, K_NO_WAIT);

	k_thread_name_set(&estimator_thread, "attitude_estimator");

	LOG_INF("Attitude estimator started");

	return 0;
}

int attitude_estimator_stop(void)
{
	if (!estimator_running) {
		return -EALREADY;
	}

	estimator_running = false;
	k_thread_join(&estimator_thread, K_FOREVER);

	LOG_INF("Attitude estimator stopped");

	return 0;
}

