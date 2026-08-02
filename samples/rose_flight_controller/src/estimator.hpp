/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Full-state estimator for the RoSE flight controller. Fuses the sensors a real
 * Crazyflie carries — a 6-axis IMU (accelerometer specific force + rate gyro) and a
 * Flow deck (optical-flow horizontal velocity + downward ToF height) — into the 12-DoF
 * state TinyMPC expects:  [x, y, z, r1, r2, r3, vx, vy, vz, wx, wy, wz]  (Rodrigues
 * attitude).
 *
 * Method:
 *   - attitude: a QUATERNION Mahony complementary filter. The gyro is integrated on the
 *     quaternion manifold (singularity-free, valid at large angles); the accelerometer
 *     nudges roll/pitch toward gravity, but that correction is GATED by |accel|: during
 *     accelerated flight the accelerometer is gravity + linear acceleration, so trusting
 *     it would inject false tilt and the controller would oscillate/flip. Yaw has no
 *     reference (no magnetometer) -> gyro only, drifts.
 *   - horizontal velocity: accel integration fused with the optical-flow measurement.
 *   - altitude (z, vz): accel integration corrected by the downward ToF height.
 *   - x/y position + yaw: pure integration (no absolute reference -> drift).
 */

#ifndef ROSE_ESTIMATOR_HPP
#define ROSE_ESTIMATOR_HPP

#define EST_NSTATES 12

struct StateEstimator {
	/* attitude quaternion (body->world), w,x,y,z */
	float qw, qx, qy, qz;
	/* world position + velocity */
	float x, y, z;
	float vx, vy, vz;
	/* last body rates, rad/s */
	float gx, gy, gz;

	/* tuning */
	float mahony_kp;  /* accelerometer -> attitude correction gain (gravity trim)     */
	float flow_gain;  /* optical-flow weight for horizontal velocity (0..1)           */
	float z_gain;     /* ToF height -> z position observer gain (0..1)                 */
	float vz_gain;    /* ToF height residual -> vz observer gain                       */

	/* Initialize at a known takeoff pose (level, at rest). */
	void init(float x0, float y0, float z0);

	/* One estimator step.
	 *   accel:  body-frame specific force (m/s^2)   [ax, ay, az]
	 *   gyro:   body-frame angular rate   (rad/s)   [gx, gy, gz]
	 *   flow:   body-frame horizontal velocity (m/s)[vx, vy]
	 *   height: downward ToF height above ground (m)
	 *   dt:     control period (s)                                                     */
	void update(const float accel[3], const float gyro[3], const float flow[2],
		    float height, float dt);

	/* Fill the 12-DoF TinyMPC state from the current estimate. */
	void get_state(float state[EST_NSTATES]) const;
};

#endif /* ROSE_ESTIMATOR_HPP */
