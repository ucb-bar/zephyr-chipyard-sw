/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 */

#include "estimator.hpp"
#include <math.h>

#define GRAVITY 9.81f

void StateEstimator::init(float x0, float y0, float z0)
{
	qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
	x = x0; y = y0; z = z0;
	vx = vy = vz = 0.0f;
	gx = gy = gz = 0.0f;
	mahony_kp = 1.0f;    /* gravity trim strength (gated by |accel| below)  */
	flow_gain = 0.85f;   /* trust optical flow for horizontal velocity       */
	z_gain = 0.3f;       /* ToF height -> z position observer gain           */
	vz_gain = 0.2f;      /* ToF residual -> vz observer gain                 */
}

void StateEstimator::update(const float accel[3], const float gyro[3],
			    const float flow[2], float height, float dt)
{
	gx = gyro[0]; gy = gyro[1]; gz = gyro[2];

	/* 1. Attitude: quaternion Mahony complementary filter.
	 *    (a) gravity-trim correction from the accelerometer, GATED by |accel|: when the
	 *        measured specific force is not ~g the body is accelerating, so the accel is
	 *        not gravity and we must not trust it (else false tilt -> oscillation/flip). */
	float wx = gx, wy = gy, wz = gz;
	float amag = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	if (amag > 1e-3f) {
		/* trust accel only near |accel| ~ g; fade to zero as it deviates */
		float gate = 1.0f - fabsf(amag - GRAVITY) / (0.5f * GRAVITY);
		if (gate > 0.0f) {
			float ax = accel[0] / amag, ay = accel[1] / amag, az = accel[2] / amag;
			/* predicted "up" (world +z) expressed in body = R^T * (0,0,1) */
			float vux = 2.0f * (qx * qz - qw * qy);
			float vuy = 2.0f * (qy * qz + qw * qx);
			float vuz = 1.0f - 2.0f * (qx * qx + qy * qy);
			/* error = measured_up x predicted_up  (rotation that aligns them) */
			float ex = ay * vuz - az * vuy;
			float ey = az * vux - ax * vuz;
			float ez = ax * vuy - ay * vux;
			float k = mahony_kp * gate;
			wx += k * ex; wy += k * ey; wz += k * ez;
		}
	}

	/* (b) integrate the quaternion with the (corrected) body rate. */
	float dqw = -0.5f * (qx * wx + qy * wy + qz * wz);
	float dqx =  0.5f * (qw * wx + qy * wz - qz * wy);
	float dqy =  0.5f * (qw * wy - qx * wz + qz * wx);
	float dqz =  0.5f * (qw * wz + qx * wy - qy * wx);
	qw += dqw * dt; qx += dqx * dt; qy += dqy * dt; qz += dqz * dt;
	float qn = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
	if (qn > 1e-9f) { qw /= qn; qx /= qn; qy /= qn; qz /= qn; }

	/* 2. Body->world rotation from the quaternion. */
	float R00 = 1 - 2 * (qy * qy + qz * qz);
	float R01 = 2 * (qx * qy - qw * qz);
	float R02 = 2 * (qx * qz + qw * qy);
	float R10 = 2 * (qx * qy + qw * qz);
	float R11 = 1 - 2 * (qx * qx + qz * qz);
	float R12 = 2 * (qy * qz - qw * qx);
	float R20 = 2 * (qx * qz - qw * qy);
	float R21 = 2 * (qy * qz + qw * qx);
	float R22 = 1 - 2 * (qx * qx + qy * qy);

	/* 3. World linear acceleration: a_world = R * f_body + g,  g=(0,0,-9.81). */
	float ax_w = R00 * accel[0] + R01 * accel[1] + R02 * accel[2];
	float ay_w = R10 * accel[0] + R11 * accel[1] + R12 * accel[2];
	float az_w = R20 * accel[0] + R21 * accel[1] + R22 * accel[2] - GRAVITY;

	/* 4. Velocity: integrate acceleration. */
	vx += ax_w * dt;
	vy += ay_w * dt;
	vz += az_w * dt;

	/* 5. Optical-flow fusion for horizontal velocity (bounds accel drift). */
	float vfx_w = R00 * flow[0] + R01 * flow[1];
	float vfy_w = R10 * flow[0] + R11 * flow[1];
	vx = (1.0f - flow_gain) * vx + flow_gain * vfx_w;
	vy = (1.0f - flow_gain) * vy + flow_gain * vfy_w;

	/* 6. Position integration; z corrected by ToF below, x/y dead-reckoned (drift). */
	x += vx * dt;
	y += vy * dt;
	z += vz * dt;

	/* 7. Altitude observer: correct z (and vz) toward the ToF height. */
	float rz = height - z;
	z  += z_gain * rz;
	vz += vz_gain * rz;
}

void StateEstimator::get_state(float state[EST_NSTATES]) const
{
	/* Rodrigues params r = q_xyz / qw (same convention as the env/TinyMPC). */
	float w = qw;
	if (fabsf(w) < 1e-9f) {
		w = (w >= 0.0f) ? 1e-9f : -1e-9f;
	}
	state[0] = x;      state[1] = y;      state[2] = z;
	state[3] = qx / w; state[4] = qy / w; state[5] = qz / w;
	state[6] = vx;     state[7] = vy;     state[8] = vz;
	state[9] = gx;     state[10] = gy;    state[11] = gz;
}
