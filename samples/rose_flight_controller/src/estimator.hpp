/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pluggable state-estimator interface for the RoSE flight controller.
 *
 * All estimators consume the same sensor set (6-axis IMU + optical-flow velocity +
 * downward ToF height) and produce the 12-DoF state TinyMPC expects:
 *   [x, y, z, r1, r2, r3, vx, vy, vz, wx, wy, wz]   (Rodrigues attitude).
 *
 * Concrete filters implement IStateEstimator; the active one is selected at build time
 * (see estimator_factory.cpp, -DROSE_USE_EKF). This keeps estimators MODULAR: new filters
 * drop in behind this interface without touching main.cpp. Current implementations:
 *   - ComplementaryEstimator (estimator_complementary.*): quaternion Mahony attitude +
 *     fixed-gain dead-reckoning translation.
 *   - EkfEstimator (estimator_ekf.*): quaternion Mahony attitude + per-axis Kalman
 *     filters for translation (flow -> velocity, ToF -> altitude); default.
 */

#ifndef ROSE_ESTIMATOR_HPP
#define ROSE_ESTIMATOR_HPP

#define EST_NSTATES 12

struct IStateEstimator {
	/* Initialize at a known takeoff pose (level, at rest). */
	virtual void init(float x0, float y0, float z0) = 0;

	/* One estimator step.
	 *   accel:  body-frame specific force (m/s^2)   [ax, ay, az]
	 *   gyro:   body-frame angular rate   (rad/s)   [gx, gy, gz]
	 *   flow:   body-frame horizontal velocity (m/s)[vx, vy]
	 *   height: downward ToF height above ground (m)
	 *   dt:     control period (s)                                                     */
	virtual void update(const float accel[3], const float gyro[3], const float flow[2],
			    float height, float dt) = 0;

	/* Fill the 12-DoF TinyMPC state from the current estimate. */
	virtual void get_state(float state[EST_NSTATES]) const = 0;

	/* Human-readable filter name (for the boot log). */
	virtual const char *name() const = 0;

	virtual ~IStateEstimator() {}
};

/* The build-time-selected estimator singleton (-DROSE_USE_EKF=1 -> EKF, else
 * complementary). Defined in estimator_factory.cpp. */
IStateEstimator &active_estimator();

#endif /* ROSE_ESTIMATOR_HPP */
