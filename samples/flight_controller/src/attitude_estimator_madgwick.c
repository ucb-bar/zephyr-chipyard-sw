#include "attitude_estimator.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include <stdio.h>

#include "madgwick_wrapper.h"

LOG_MODULE_REGISTER(attitude_estimator, CONFIG_LOG_DEFAULT_LEVEL);

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

#ifndef CONFIG_ATTITUDE_ESTIMATOR_FREQUENCY
#define CONFIG_ATTITUDE_ESTIMATOR_FREQUENCY 200
#endif

#ifndef CONFIG_ATTITUDE_ESTIMATOR_STACK_SIZE
#define CONFIG_ATTITUDE_ESTIMATOR_STACK_SIZE 2048
#endif

#define ESTIMATOR_FREQUENCY CONFIG_ATTITUDE_ESTIMATOR_FREQUENCY
#define ESTIMATOR_PERIOD_MS (1000 / ESTIMATOR_FREQUENCY)
#define ESTIMATOR_PERIOD_US ((1000000LL) / ESTIMATOR_FREQUENCY)
#define COMPLEMENTARY_FILTER_ALPHA 0.98f  

static struct k_thread estimator_thread;
static K_THREAD_STACK_DEFINE(estimator_stack, CONFIG_ATTITUDE_ESTIMATOR_STACK_SIZE);
static volatile bool estimator_running = false;
static struct k_mutex attitude_mutex;

static struct attitude current_attitude = {0.0f, 0.0f, 0.0f};

static void attitude_estimator_thread(void *arg1, void *arg2, void *arg3)
{
    struct sensor_value accel[3], gyro[3];
    int ret;
    float accel_f[3], gyro_f[3];
    int64_t last_time = k_uptime_get();
    int64_t current_time;
    float dt;
    int log_counter = 0;  

    LOG_INF("Attitude estimator thread started (Madgwick)");

    while (estimator_running) {
        current_time = k_uptime_get();
        dt = (current_time - last_time) / 1000.0f;  
        last_time = current_time;

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

        accel_f[0] = sensor_value_to_double(&accel[0]);
        accel_f[1] = sensor_value_to_double(&accel[1]);
        accel_f[2] = sensor_value_to_double(&accel[2]);

        gyro_f[0] = sensor_value_to_double(&gyro[0]);
        gyro_f[1] = sensor_value_to_double(&gyro[1]);
        gyro_f[2] = sensor_value_to_double(&gyro[2]);

        float qw, qx, qy, qz;
        run_madgwick_sample(
            gyro_f[0], gyro_f[1], gyro_f[2],
            accel_f[0], accel_f[1], accel_f[2],
            0.0f, 0.0f, 0.0f,   
            dt,
            &qw, &qx, &qy, &qz
        );

        float roll  = atan2f(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)) * 180.0f / M_PI;
        float pitch = asinf(2*(qw*qy - qz*qx)) * 180.0f / M_PI;
        float yaw   = atan2f(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)) * 180.0f / M_PI;

        k_mutex_lock(&attitude_mutex, K_FOREVER);
        current_attitude.roll  = roll;
        current_attitude.pitch = pitch;
        current_attitude.yaw   = yaw;
        k_mutex_unlock(&attitude_mutex);

        log_counter++;
        if (log_counter % 100 == 0) {  
            LOG_INF("Madgwick Q: qw=%.6f qx=%.6f qy=%.6f qz=%.6f", qw, qx, qy, qz);
            LOG_INF("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
        }

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
    if (!device_is_ready(accel_dev)) {
        LOG_ERR("Accelerometer device is not ready");
        return -ENODEV;
    }
    if (!device_is_ready(gyro_dev)) {
        LOG_ERR("Gyroscope device is not ready");
        return -ENODEV;
    }

    k_mutex_init(&attitude_mutex);
    LOG_INF("Attitude estimator initialized (frequency: %d Hz)", ESTIMATOR_FREQUENCY);
    return 0;
}

int attitude_estimator_start(void)
{
    if (estimator_running) return -EALREADY;

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
    if (!estimator_running) return -EALREADY;

    estimator_running = false;
    k_thread_join(&estimator_thread, K_FOREVER);

    LOG_INF("Attitude estimator stopped");
    return 0;
}

int attitude_estimator_get(struct attitude *attitude)
{
    if (!attitude) return -EINVAL;

    k_mutex_lock(&attitude_mutex, K_FOREVER);
    *attitude = current_attitude;
    k_mutex_unlock(&attitude_mutex);

    return 0;
}

// create control task to launch in main file
// creates shared memory for state and controller can read from that to get position
// set position to 0 for now since madgwick is giving attitude
// feed magwick into mpc model and put position and velocity? as 0 for now