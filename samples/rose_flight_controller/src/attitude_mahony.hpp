/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared quaternion Mahony attitude filter, reused by every estimator implementation so
 * the (well-behaved) attitude path isn't duplicated. Integrates the gyro on the quaternion
 * manifold and trims roll/pitch toward the accelerometer ONLY when |accel| ~ g, so linear
 * acceleration doesn't inject false tilt. Yaw has no absolute reference (gyro only).
 */

#ifndef ROSE_ATTITUDE_MAHONY_HPP
#define ROSE_ATTITUDE_MAHONY_HPP

#include <math.h>

struct MahonyAttitude {
	float qw, qx, qy, qz;   /* body->world quaternion */
	float kp;               /* gravity-trim gain (gated by |accel|) */

	void init()
	{
		qw = 1.0f; qx = qy = qz = 0.0f;
		kp = 1.0f;
	}

	/* Advance the quaternion with body rate + gated gravity trim. */
	void update(const float accel[3], const float gyro[3], float dt)
	{
		float wx = gyro[0], wy = gyro[1], wz = gyro[2];
		float amag = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
		if (amag > 1e-3f) {
			float gate = 1.0f - fabsf(amag - 9.81f) / (0.5f * 9.81f);
			if (gate > 0.0f) {
				float ax = accel[0]/amag, ay = accel[1]/amag, az = accel[2]/amag;
				/* predicted world +z ("up") expressed in body = R^T*(0,0,1) */
				float vux = 2.0f * (qx*qz - qw*qy);
				float vuy = 2.0f * (qy*qz + qw*qx);
				float vuz = 1.0f - 2.0f * (qx*qx + qy*qy);
				float ex = ay*vuz - az*vuy;
				float ey = az*vux - ax*vuz;
				float ez = ax*vuy - ay*vux;
				float k = kp * gate;
				wx += k*ex; wy += k*ey; wz += k*ez;
			}
		}
		float dqw = -0.5f * (qx*wx + qy*wy + qz*wz);
		float dqx =  0.5f * (qw*wx + qy*wz - qz*wy);
		float dqy =  0.5f * (qw*wy - qx*wz + qz*wx);
		float dqz =  0.5f * (qw*wz + qx*wy - qy*wx);
		qw += dqw*dt; qx += dqx*dt; qy += dqy*dt; qz += dqz*dt;
		float n = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
		if (n > 1e-9f) { qw /= n; qx /= n; qy /= n; qz /= n; }
	}

	/* Body->world rotation matrix (row-major R[0..8]). */
	void rot(float R[9]) const
	{
		R[0] = 1 - 2*(qy*qy + qz*qz); R[1] = 2*(qx*qy - qw*qz);     R[2] = 2*(qx*qz + qw*qy);
		R[3] = 2*(qx*qy + qw*qz);     R[4] = 1 - 2*(qx*qx + qz*qz); R[5] = 2*(qy*qz - qw*qx);
		R[6] = 2*(qx*qz - qw*qy);     R[7] = 2*(qy*qz + qw*qx);     R[8] = 1 - 2*(qx*qx + qy*qy);
	}

	/* Rodrigues params r = q_xyz / qw (matches the env/TinyMPC convention). */
	void rodrigues(float r[3]) const
	{
		float w = qw;
		if (fabsf(w) < 1e-9f) w = (w >= 0.0f) ? 1e-9f : -1e-9f;
		r[0] = qx / w; r[1] = qy / w; r[2] = qz / w;
	}
};

#endif /* ROSE_ATTITUDE_MAHONY_HPP */
