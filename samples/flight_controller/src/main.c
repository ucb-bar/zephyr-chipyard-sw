#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "attitude_estimator.h"
#include "control_task.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
    int ret;
    struct attitude att;

    LOG_INF("Flight Controller Starting...");

    /* --- Initialize attitude estimator --- */
    ret = attitude_estimator_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize attitude estimator: %d", ret);
        return ret;
    }

    /* --- Start attitude estimator thread --- */
    ret = attitude_estimator_start();
    if (ret < 0) {
        LOG_ERR("Failed to start attitude estimator: %d", ret);
        return ret;
    }

    /* --- Start control task (TinyMPC runs inside this) --- */
    control_task_start();

    LOG_INF("Flight Controller Initialized");

    /* --- Optional monitoring loop --- */
    while (1) {
        if (attitude_estimator_get(&att) == 0) {
            LOG_INF("Attitude [deg] r=%.2f p=%.2f y=%.2f",
                    att.roll, att.pitch, att.yaw);
        }
        k_msleep(100);
    }

    return 0;
}
