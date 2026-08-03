/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 */

#include "estimator_complementary.hpp"

#define GRAVITY 9.81f

void ComplementaryEstimator::init(float x0, float y0, float z0)
{
	att.init();
	x = x0; y = y0; z = z0;
	vx = vy = vz = 0.0f;
	gx = gy = gz = 0.0f;
	awx = awy = awz = 0.0f; dt_last = 0.0f;
	flow_gain = 1.0f;    /* trust optical flow fully for horizontal velocity (zero lag) */
	z_gain = 0.5f;       /* trust ToF strongly for altitude */
	vz_gain = 0.3f;
	lead = 1.0f;         /* translation lead: compensate the 1-step actuation delay */
	lead_att = 0.5f;     /* attitude lead: smaller (predicting the noisy gyro forward a full
	                      * step over-amplifies the fast attitude loop) */
}

void ComplementaryEstimator::update(const float accel[3], const float gyro[3],
				    const float flow[2], float height, bool tof_valid, float dt)
{
	gx = gyro[0]; gy = gyro[1]; gz = gyro[2];
	att.update(accel, gyro, dt);
	float R[9]; att.rot(R);

	/* world acceleration = R f_body + g */
	float ax_w = R[0]*accel[0] + R[1]*accel[1] + R[2]*accel[2];
	float ay_w = R[3]*accel[0] + R[4]*accel[1] + R[5]*accel[2];
	float az_w = R[6]*accel[0] + R[7]*accel[1] + R[8]*accel[2] - GRAVITY;
	awx = ax_w; awy = ay_w; awz = az_w; dt_last = dt;

	vx += ax_w*dt; vy += ay_w*dt; vz += az_w*dt;

	/* optical-flow fusion for horizontal velocity */
	float vfx = R[0]*flow[0] + R[1]*flow[1];
	float vfy = R[3]*flow[0] + R[4]*flow[1];
	vx = (1.0f - flow_gain)*vx + flow_gain*vfx;
	vy = (1.0f - flow_gain)*vy + flow_gain*vfy;

	x += vx*dt; y += vy*dt; z += vz*dt;

	/* ToF altitude observer -- only on steps with a fresh (low-rate) ToF sample. Between
	 * samples z/vz dead-reckon on the accelerometer (fast layer); the ToF corrects the
	 * accumulated drift when it arrives (slow layer). */
	if (tof_valid) {
		float rz = height - z;
		z  += z_gain*rz;
		vz += vz_gain*rz;
	}
}

void ComplementaryEstimator::get_state(float state[EST_NSTATES]) const
{
	float h = lead * dt_last;
	/* Attitude lead (smaller than translation): predict the quaternion forward by the body
	 * rate over lead_att*dt, then Rodrigues. Compensates the fast attitude loop's latency
	 * without the full-step over-amplification that destabilizes it. */
	float ha = lead_att * dt_last;
	float pw = att.qw, px = att.qx, py = att.qy, pz = att.qz;
	float dpw = -0.5f*(px*gx + py*gy + pz*gz);
	float dpx =  0.5f*(pw*gx + py*gz - pz*gy);
	float dpy =  0.5f*(pw*gy - px*gz + pz*gx);
	float dpz =  0.5f*(pw*gz + px*gy - py*gx);
	pw += dpw*ha; px += dpx*ha; py += dpy*ha; pz += dpz*ha;
	float qn = sqrtf(pw*pw + px*px + py*py + pz*pz);
	if (qn > 1e-9f) { pw/=qn; px/=qn; py/=qn; pz/=qn; }
	float qwv = (fabsf(pw) < 1e-9f) ? (pw >= 0.0f ? 1e-9f : -1e-9f) : pw;
	float r[3] = { px/qwv, py/qwv, pz/qwv };
	/* Translation lead: feed TinyMPC the predicted next state (pos += vel*dt,
	 * vel += a_world*dt). Restores the phase margin the estimator latency erodes. */
	state[0] = x + vx*h;  state[1] = y + vy*h;  state[2] = z + vz*h;
	state[3] = r[0];      state[4] = r[1];      state[5] = r[2];
	state[6] = vx + awx*h; state[7] = vy + awy*h; state[8] = vz + awz*h;
	/* Body-frame rate gyro (raw, low-noise). */
	state[9] = gx;  state[10] = gy; state[11] = gz;
}
