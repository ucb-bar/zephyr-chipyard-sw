/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "attitude_estimator.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
	int ret;
	struct attitude attitude;

	LOG_INF("Flight Controller Starting...");

	/* Initialize attitude estimator */
	ret = attitude_estimator_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize attitude estimator: %d", ret);
		return ret;
	}

	/* Start attitude estimator task */
	ret = attitude_estimator_start();
	if (ret < 0) {
		LOG_ERR("Failed to start attitude estimator: %d", ret);
		return ret;
	}

	LOG_INF("Flight Controller Initialized");

	/* Main loop - can be used for other tasks or monitoring */
	while (1) {
		/* Get current attitude estimate */
		ret = attitude_estimator_get(&attitude);
		if (ret == 0) {
			LOG_INF("Attitude - Roll: %.3f, Pitch: %.3f, Yaw: %.3f (rad)",
				attitude.roll, attitude.pitch, attitude.yaw);
		}

		/* Sleep for a bit - main loop runs at lower frequency */
		k_msleep(100);
	}

	return 0;
}

