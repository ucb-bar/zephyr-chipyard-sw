/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 */

#include "estimator_ekf.hpp"

#define GRAVITY 9.81f

void EkfEstimator::init(float x0, float y0, float z0)
{
	att.init();
	/* process accel-noise ~2 m/s^2 (model/attitude error); accel-bias random walk ~0.05
	 * m/s^2/sqrt(s) with an initial 1-sigma bias uncertainty of 0.5 m/s^2 (covers the L1/L2
	 * injected biases). The bias state converges within a few seconds and removes the steady
	 * altitude offset / horizontal drift a fixed-gain filter leaves under a biased IMU. */
	kx.init(x0, 2.0f, 0.05f, 0.25f);
	ky.init(y0, 2.0f, 0.05f, 0.25f);
	kz.init(z0, 2.0f, 0.05f, 0.25f);
	gx = gy = gz = 0.0f;
	awx = awy = awz = 0.0f; dt_last = 0.0f;
	r_flow = 9e-4f;      /* optical-flow velocity variance ~ (0.03 m/s)^2, matched to the
	                      * aggressive-noise flow std so the filter doesn't over-trust noisy
	                      * flow (stress plan section 4, item 4). */
	r_tof  = 1e-4f;      /* trust ToF height strongly (~0.01 m std) */
	/* chi-square (NIS) outlier gates (item 3): reject a flow sample whose normalized
	 * innovation^2 exceeds ~9 (a ~0.5 m/s flow outlier at hover is tens of sigma -> rejected,
	 * while ordinary 0.03 m/s noise passes). ToF gate is looser (25) since altitude is only
	 * weakly re-observed and a wrongly-rejected ToF would let the height drift. */
	flow_gate = 9.0f;
	tof_gate  = 25.0f;
	r_wall = 4e-3f;      /* wall-derived horizontal position ~ (0.06 m)^2 (multizone min + a
	                      * modest margin for wall non-perpendicularity); anchors x/y position. */
	delay_steps = 1.0f;  /* control acts 1 step later -> predict 1 step ahead */
}

void EkfEstimator::update(const float accel[3], const float gyro[3],
			  const float flow[2], bool flow_valid, float height, bool tof_valid, float dt)
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

	/* update: horizontal velocity from optical flow (body->world). On a flow dropout the
	 * sample is stale, so skip the velocity correction and let the KF run predict-only (its
	 * covariance grows, so the next valid sample is weighted more) -- mirrors the ToF path. */
	if (flow_valid) {
		float vfx = R[0]*flow[0] + R[1]*flow[1];
		float vfy = R[3]*flow[0] + R[4]*flow[1];
		kx.update_vel(vfx, r_flow, flow_gate);
		ky.update_vel(vfy, r_flow, flow_gate);
	}

	/* update: altitude from downward ToF (position measurement) -- only when a fresh
	 * low-rate ToF sample is available. Between samples the z-axis KF runs predict-only
	 * (accel), and its covariance grows so the next ToF correction is weighted more. */
	if (tof_valid) {
		kz.update_pos(height, r_tof, tof_gate);
	}
}

void EkfEstimator::fuse_walls(float d_front, float d_back, float d_left, float d_right,
			      float rmax)
{
	/* Only fuse a pair when BOTH facing walls are within range (a valid two-sided reference).
	 * (d_right - d_left)/2 is the lateral position relative to the corridor center; likewise
	 * (d_back - d_front)/2 for longitudinal. These are POSITION measurements (H=[1,0,0]) that
	 * anchor kx/ky, which flow (velocity only) cannot -- removing horizontal drift and making
	 * the position observable for waypoint control. The corridor center is the estimator's
	 * origin (init pose), matching how the vehicle is placed at the corridor midline. */
	const float lim = 0.95f * rmax;
	if (d_left < lim && d_right < lim) {
		ky.update_pos(0.5f * (d_right - d_left), r_wall, 0.0f);
	}
	if (d_front < lim && d_back < lim) {
		kx.update_pos(0.5f * (d_back - d_front), r_wall, 0.0f);
	}
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

	/* translation: predict pos += vel*h + 0.5 a h^2, vel += a*h, using the BIAS-CORRECTED
	 * world acceleration (a - ba) consistent with the KF predict, so the lead prediction is
	 * not re-corrupted by the estimated accel bias. */
	float cax = awx - kx.ba, cay = awy - ky.ba, caz = awz - kz.ba;
	state[0] = kx.p + kx.v*h + 0.5f*cax*h*h;
	state[1] = ky.p + ky.v*h + 0.5f*cay*h*h;
	state[2] = kz.p + kz.v*h + 0.5f*caz*h*h;
	state[3] = px/qwv; state[4] = py/qwv; state[5] = pz/qwv;
	state[6] = kx.v + cax*h; state[7] = ky.v + cay*h; state[8] = kz.v + caz*h;
	state[9] = wxw;  state[10] = wyw; state[11] = wzw;
}
