/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 */

#include "estimator_ekf.hpp"

#define GRAVITY 9.81f

void EkfEstimator::init(float x0, float y0, float z0)
{
	att.init();
	/* process accel-noise ~2 m/s^2 (model/attitude error); tune for stability */
	kx.init(x0, 2.0f);
	ky.init(y0, 2.0f);
	kz.init(z0, 2.0f);
	gx = gy = gz = 0.0f;
	r_flow = 4e-4f;   /* trust optical flow strongly for velocity (~0.02 m/s std) */
	r_tof  = 1e-4f;   /* trust ToF height strongly (~0.01 m std) */
}

void EkfEstimator::update(const float accel[3], const float gyro[3],
			  const float flow[2], float height, float dt)
{
	gx = gyro[0]; gy = gyro[1]; gz = gyro[2];
	att.update(accel, gyro, dt);
	float R[9]; att.rot(R);

	/* world acceleration = R f_body + g */
	float ax_w = R[0]*accel[0] + R[1]*accel[1] + R[2]*accel[2];
	float ay_w = R[3]*accel[0] + R[4]*accel[1] + R[5]*accel[2];
	float az_w = R[6]*accel[0] + R[7]*accel[1] + R[8]*accel[2] - GRAVITY;

	/* predict */
	kx.predict(ax_w, dt);
	ky.predict(ay_w, dt);
	kz.predict(az_w, dt);

	/* update: horizontal velocity from optical flow (body->world) */
	float vfx = R[0]*flow[0] + R[1]*flow[1];
	float vfy = R[3]*flow[0] + R[4]*flow[1];
	kx.update_vel(vfx, r_flow);
	ky.update_vel(vfy, r_flow);

	/* update: altitude from downward ToF (position measurement) */
	kz.update_pos(height, r_tof);
}

void EkfEstimator::get_state(float state[EST_NSTATES]) const
{
	float r[3]; att.rodrigues(r);
	/* world-frame angular velocity (R * body-rate) to match the ground-truth env /
	 * TinyMPC convention (the loop is stable with world rates). */
	float R[9]; att.rot(R);
	float wxw = R[0]*gx + R[1]*gy + R[2]*gz;
	float wyw = R[3]*gx + R[4]*gy + R[5]*gz;
	float wzw = R[6]*gx + R[7]*gy + R[8]*gz;
	state[0] = kx.p; state[1] = ky.p; state[2] = kz.p;
	state[3] = r[0]; state[4] = r[1]; state[5] = r[2];
	state[6] = kx.v; state[7] = ky.v; state[8] = kz.v;
	state[9] = wxw;  state[10] = wyw; state[11] = wzw;
}
