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
	awx = awy = awz = 0.0f; dt_last = 0.0f;
	r_flow = 4e-4f;      /* trust optical flow strongly for velocity (~0.02 m/s std) */
	r_tof  = 1e-4f;      /* trust ToF height strongly (~0.01 m std) */
	delay_steps = 1.0f;  /* control acts 1 step later -> predict 1 step ahead */
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
	awx = ax_w; awy = ay_w; awz = az_w; dt_last = dt;

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
	/* Model-based delay compensation: the control acts delay_steps later, so report the
	 * state predicted that far ahead using the estimator's own model (kinematics + last
	 * acceleration / body rate). One knob = the known delay; not per-channel tuning. */
	float h = delay_steps * dt_last;

	/* attitude: integrate the quaternion forward by the body rate over h */
	float pw = att.qw, px = att.qx, py = att.qy, pz = att.qz;
	float dpw = -0.5f*(px*gx + py*gy + pz*gz);
	float dpx =  0.5f*(pw*gx + py*gz - pz*gy);
	float dpy =  0.5f*(pw*gy - px*gz + pz*gx);
	float dpz =  0.5f*(pw*gz + px*gy - py*gx);
	pw += dpw*h; px += dpx*h; py += dpy*h; pz += dpz*h;
	float qn = sqrtf(pw*pw + px*px + py*py + pz*pz);
	if (qn > 1e-9f) { pw/=qn; px/=qn; py/=qn; pz/=qn; }
	float qwv = (fabsf(pw) < 1e-9f) ? (pw >= 0.0f ? 1e-9f : -1e-9f) : pw;

	/* world-frame angular velocity (R * body-rate), matching the ground-truth convention */
	float R[9]; att.rot(R);
	float wxw = R[0]*gx + R[1]*gy + R[2]*gz;
	float wyw = R[3]*gx + R[4]*gy + R[5]*gz;
	float wzw = R[6]*gx + R[7]*gy + R[8]*gz;

	/* translation: predict pos += vel*h + 0.5 a h^2, vel += a*h */
	state[0] = kx.p + kx.v*h + 0.5f*awx*h*h;
	state[1] = ky.p + ky.v*h + 0.5f*awy*h*h;
	state[2] = kz.p + kz.v*h + 0.5f*awz*h*h;
	state[3] = px/qwv; state[4] = py/qwv; state[5] = pz/qwv;
	state[6] = kx.v + awx*h; state[7] = ky.v + awy*h; state[8] = kz.v + awz*h;
	state[9] = wxw;  state[10] = wyw; state[11] = wzw;
}
