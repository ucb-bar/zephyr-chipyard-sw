/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Full-state estimator for the RoSE flight controller. Fuses the sensors a real
 * Crazyflie carries — a 6-axis IMU (accelerometer specific force + rate gyro) and a
 * downward optical-flow sensor (body-frame horizontal velocity) — into the 12-DoF state
 * TinyMPC expects:  [x, y, z, r1, r2, r3, vx, vy, vz, wx, wy, wz]  (Rodrigues attitude).
 *
 * Method (a complementary filter + dead reckoning, extending samples/flight_controller's
 * attitude_estimator.c):
 *   - attitude: gyro integration corrected toward the accelerometer gravity vector
 *     (roll/pitch); yaw from gyro only (no magnetometer -> yaw drifts).
 *   - horizontal velocity: accel integration fused with the optical-flow measurement
 *     (flow bounds the velocity error; without it accel integration runs away).
 *   - altitude (z, vz): accel integration corrected by the downward ToF height (a
 *     fixed-gain observer), so altitude is observable and the hover holds.
 *   - x/y position + yaw: pure integration. There is NO absolute horizontal-position or
 *     heading reference, so these drift over time — expected ("no global pose feedback").
 */

#ifndef ROSE_ESTIMATOR_HPP
#define ROSE_ESTIMATOR_HPP

#define EST_NSTATES 12

struct StateEstimator {
	/* estimated state */
	float roll, pitch, yaw;   /* attitude, rad          */
	float x, y, z;            /* world position, m       */
	float vx, vy, vz;         /* world velocity, m/s     */
	float gx, gy, gz;         /* last body rates, rad/s  */

	/* tuning */
	float alpha;      /* complementary-filter gyro weight (0..1), higher = trust gyro */
	float flow_gain;  /* optical-flow weight for horizontal velocity (0..1)           */
	float z_gain;     /* ToF height -> z position observer gain (0..1)                 */
	float vz_gain;    /* ToF height residual -> vz observer gain (per step)           */

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
