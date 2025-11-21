/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <zephyr/kernel.h>

/* Attitude estimation result */
struct attitude {
	float roll;   /* Roll angle in radians */
	float pitch;  /* Pitch angle in radians */
	float yaw;    /* Yaw angle in radians */
};

/**
 * @brief Initialize the attitude estimator
 *
 * @return 0 on success, negative error code on failure
 */
int attitude_estimator_init(void);

/**
 * @brief Get the current attitude estimate
 *
 * @param attitude Pointer to attitude structure to fill
 * @return 0 on success, negative error code on failure
 */
int attitude_estimator_get(struct attitude *attitude);

/**
 * @brief Start the attitude estimator task
 *
 * @return 0 on success, negative error code on failure
 */
int attitude_estimator_start(void);

/**
 * @brief Stop the attitude estimator task
 *
 * @return 0 on success, negative error code on failure
 */
int attitude_estimator_stop(void);

#endif /* ATTITUDE_ESTIMATOR_H */

