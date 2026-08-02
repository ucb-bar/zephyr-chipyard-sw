/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 */

#include "estimator.hpp"
#include <math.h>

#define GRAVITY 9.81f

void StateEstimator::init(float x0, float y0, float z0)
{
	roll = pitch = yaw = 0.0f;
	x = x0; y = y0; z = z0;
	vx = vy = vz = 0.0f;
	gx = gy = gz = 0.0f;
	alpha = 0.98f;       /* mostly gyro; accelerometer only trims roll/pitch */
	flow_gain = 0.85f;   /* trust optical flow for horizontal velocity       */
	z_gain = 0.3f;       /* ToF height -> z position observer gain           */
	vz_gain = 0.2f;      /* ToF residual -> vz observer gain (alpha-beta)    */
}

void StateEstimator::update(const float accel[3], const float gyro[3],
			    const float flow[2], float height, float dt)
{
	/* 1. Attitude — complementary filter (cf. samples/flight_controller). The
	 *    accelerometer gives an absolute (drift-free) roll/pitch from gravity; the
	 *    gyro gives smooth high-rate integration. Yaw is gyro-only (no magnetometer). */
	float accel_roll  = atan2f(accel[1], accel[2]);
	float accel_pitch = atan2f(-accel[0],
				   sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
	float gyro_roll  = roll  + gyro[0] * dt;
	float gyro_pitch = pitch + gyro[1] * dt;
	float gyro_yaw   = yaw   + gyro[2] * dt;
	roll  = alpha * gyro_roll  + (1.0f - alpha) * accel_roll;
	pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
	yaw   = gyro_yaw;
	gx = gyro[0]; gy = gyro[1]; gz = gyro[2];

	/* 2. Body->world rotation from Euler (Z-Y-X: yaw, pitch, roll). */
	float cr = cosf(roll),  sr = sinf(roll);
	float cp = cosf(pitch), sp = sinf(pitch);
	float cy = cosf(yaw),   sy = sinf(yaw);
	float R00 = cy * cp, R01 = cy * sp * sr - sy * cr, R02 = cy * sp * cr + sy * sr;
	float R10 = sy * cp, R11 = sy * sp * sr + cy * cr, R12 = sy * sp * cr - cy * sr;
	float R20 = -sp,     R21 = cp * sr,                R22 = cp * cr;

	/* 3. World linear acceleration: invert the specific-force measurement,
	 *    a_world = R * f_body + g,  g = (0,0,-9.81). At hover this is ~0. */
	float ax_w = R00 * accel[0] + R01 * accel[1] + R02 * accel[2];
	float ay_w = R10 * accel[0] + R11 * accel[1] + R12 * accel[2];
	float az_w = R20 * accel[0] + R21 * accel[1] + R22 * accel[2] - GRAVITY;

	/* 4. Velocity: integrate acceleration. */
	vx += ax_w * dt;
	vy += ay_w * dt;
	vz += az_w * dt;

	/* 5. Optical-flow fusion for horizontal velocity (bounds accel drift). Flow is a
	 *    body-frame horizontal velocity; rotate its xy into the world and blend. */
	float vfx_w = R00 * flow[0] + R01 * flow[1];
	float vfy_w = R10 * flow[0] + R11 * flow[1];
	vx = (1.0f - flow_gain) * vx + flow_gain * vfx_w;
	vy = (1.0f - flow_gain) * vy + flow_gain * vfy_w;

	/* 6. Position: integrate velocity. x/y have no absolute reference (drift); z is
	 *    corrected below by the ToF height. */
	x += vx * dt;
	y += vy * dt;
	z += vz * dt;

	/* 7. Altitude observer: correct z (and vz) toward the downward ToF height. This
	 *    makes altitude observable so the hover holds; without it dead-reckoned z locks
	 *    in the climb-transient error. (x/y stay dead-reckoned -> lateral drift.) */
	float rz = height - z;
	z  += z_gain * rz;
	vz += vz_gain * rz;
}

void StateEstimator::get_state(float state[EST_NSTATES]) const
{
	/* Rodrigues params from the Euler estimate, via the body quaternion:
	 * r = q_xyz / qw  (same convention the env/TinyMPC use). */
	float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
	float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
	float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);
	float qw = cr * cp * cy + sr * sp * sy;
	float qx = sr * cp * cy - cr * sp * sy;
	float qy = cr * sp * cy + sr * cp * sy;
	float qz = cr * cp * sy - sr * sp * cy;
	if (fabsf(qw) < 1e-9f) {
		qw = (qw >= 0.0f) ? 1e-9f : -1e-9f;
	}

	state[0] = x;       state[1] = y;       state[2] = z;
	state[3] = qx / qw; state[4] = qy / qw; state[5] = qz / qw;
	state[6] = vx;      state[7] = vy;      state[8] = vz;
	state[9] = gx;      state[10] = gy;     state[11] = gz;
}
