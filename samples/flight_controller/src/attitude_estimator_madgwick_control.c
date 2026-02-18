/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "attitude_estimator.h"
#include "madgwick_wrapper.h"

LOG_MODULE_REGISTER(attitude_estimator, CONFIG_LOG_DEFAULT_LEVEL);

static struct attitude att_state = {0};
static struct k_mutex att_mutex;

static bool running = false;
static bool initialized = false;


#define ATT_THREAD_STACK 2048
#define ATT_THREAD_PRIO  5

K_THREAD_STACK_DEFINE(att_stack, ATT_THREAD_STACK);
static struct k_thread att_thread;

static const struct device *imu;

#define ATT_UPDATE_HZ 200
#define ATT_UPDATE_MS (1000 / ATT_UPDATE_HZ)

static void quaternion_to_rpy(float qw, float qx, float qy, float qz,
                              float *roll, float *pitch, float *yaw)
{
    *roll  = atan2f(2.0f*(qw*qx + qy*qz),
                    1.0f - 2.0f*(qx*qx + qy*qy));
    *pitch = asinf(2.0f*(qw*qy - qz*qx));
    *yaw   = atan2f(2.0f*(qw*qz + qx*qy),
                    1.0f - 2.0f*(qy*qy + qz*qz));
}

static void attitude_thread_fn(void *p1, void *p2, void *p3)
{
    imu = DEVICE_DT_GET_ANY(invensense_mpu6050);
    if (!imu || !device_is_ready(imu)) {
        LOG_ERR("IMU not found or not ready");
        running = false;
        return;
    }

    LOG_INF("Attitude estimator started at %d Hz", ATT_UPDATE_HZ);

    uint64_t last_log = 0;

    while (running) {

        sensor_sample_fetch(imu);

        struct sensor_value ax, ay, az;
        struct sensor_value gx, gy, gz;

        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &ax);
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ay);
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &az);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_X,  &gx);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y,  &gy);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z,  &gz);

        float ax_f = sensor_value_to_double(&ax);
        float ay_f = sensor_value_to_double(&ay);
        float az_f = sensor_value_to_double(&az);
        float gx_f = sensor_value_to_double(&gx);
        float gy_f = sensor_value_to_double(&gy);
        float gz_f = sensor_value_to_double(&gz);

        float qw, qx, qy, qz;

        float dt = 1.0f / ATT_UPDATE_HZ;

        run_madgwick_sample(gx_f, gy_f, gz_f,
                            ax_f, ay_f, az_f,
                            0, 0, 0,       
                            dt,
                            &qw, &qx, &qy, &qz);

        float roll, pitch, yaw;
        quaternion_to_rpy(qw, qx, qy, qz, &roll, &pitch, &yaw);

        /* Save into shared state */
        k_mutex_lock(&att_mutex, K_FOREVER);
        att_state.roll  = roll;
        att_state.pitch = pitch;
        att_state.yaw   = yaw;
        k_mutex_unlock(&att_mutex);

        /* Log at 10 Hz */
        uint64_t now = k_uptime_get();
        if (now - last_log > 100) {
            last_log = now;
            LOG_INF("RPY: %.2f %.2f %.2f", roll, pitch, yaw);
        }

        k_msleep(ATT_UPDATE_MS);
    }

    LOG_INF("Attitude estimator stopped.");
}


int attitude_estimator_init(void)
{
    if (initialized)
        return 0;

    k_mutex_init(&att_mutex);
    initialized = true;
    return 0;
}

int attitude_estimator_start(void)
{
    if (!initialized)
        return -EINVAL;

    if (running)
        return 0;

    running = true;

    k_thread_create(&att_thread,
                    att_stack,
                    K_THREAD_STACK_SIZEOF(att_stack),
                    attitude_thread_fn,
                    NULL, NULL, NULL,
                    ATT_THREAD_PRIO,
                    0,
                    K_NO_WAIT);

    k_thread_name_set(&att_thread, "attitude_estimator");
    return 0;
}

int attitude_estimator_stop(void)
{
    if (!running)
        return 0;

    running = false;
    return 0;
}

int attitude_estimator_get(struct attitude *out)
{
    if (!initialized || !out)
        return -EINVAL;

    k_mutex_lock(&att_mutex, K_FOREVER);
    *out = att_state;
    k_mutex_unlock(&att_mutex);

    return 0;
}
