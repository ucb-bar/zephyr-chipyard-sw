/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 */

#include "estimator_ekf.hpp"

#define GRAVITY 9.81f

/* Optical-flow measurement variance (tunable). The flow is intrinsically noisy and ~3x noisier on
 * the PMW3901's y (left) axis than x (fwd), so trust y less. And v = angular_flow * height, so the
 * velocity noise (variance ~ height^2) grows with altitude -> scale r_flow up with height. Raising
 * these smooths est-v (the KF's proper noise knob) at the cost of a little lag. */
#ifndef R_FLOW_X
#define R_FLOW_X    4.0e-3f   /* fwd  ~ (0.063 m/s)^2 base */
#endif
#ifndef R_FLOW_Y
#define R_FLOW_Y    2.5e-2f   /* left ~ (0.16 m/s)^2 base -- y is markedly noisier */
#endif
#ifndef FLOW_HREF_M
#define FLOW_HREF_M 0.5f      /* flow-velocity variance doubles by this height */
#endif
/* Diagnostic: fuse the optical-flow velocity into the horizontal KF (1) or NOT (0). With
 * ROSE_FLOW_FUSE=0, horizontal velocity is pure accel dead-reckoning (+ on-ground ZUPT) -- the
 * flow-less "dead reckoning" baseline, to isolate whether flow feedback is driving the runaway. */
#ifndef ROSE_FLOW_FUSE
#define ROSE_FLOW_FUSE 1
#endif

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
	r_flow_x = R_FLOW_X; r_flow_y = R_FLOW_Y; flow_href = FLOW_HREF_M;  /* per-axis, height-scaled */
	r_zupt = 1e-3f;      /* on-ground zero-velocity pseudo-measurement ~ (0.03 m/s)^2: when there is
	                      * no flow but the ToF says we are grounded, the vehicle is at rest, so
	                      * measure v=0. Anchors vx/vy (and makes the x/y accel bias observable)
	                      * instead of integrating accel into an unbounded velocity drift. */
	zupt_height = 0.05f; /* "on the ground" band: at or below this ToF height, at rest -> ZUPT.
	                      * Kept well under any hover setpoint so it never fires in flight. */
	r_tof  = 1e-4f;      /* trust ToF height strongly (~0.01 m std) */
	/* Flow velocity outlier gate DISABLED (0). A fixed chi-square gate can't distinguish "bad flow"
	 * from "good flow vs a diverged state": once the accel-predicted velocity drifts past the gate
	 * (e.g. noisy flow amplified at large ToF height is rejected -> predict-only), EVERY subsequent
	 * good flow sample (v~0) has a huge residual vs the diverged velocity and is ALSO rejected, so
	 * velocity runs away unbounded (observed on the bench: est-v -> 152 m/s while flow read ~0/ok).
	 * Instead the flow input is clamped to a physical bound (main.cpp) and the velocity state is
	 * clamped below, so the flow is ALWAYS allowed to pull velocity back. ToF gate kept (position). */
	flow_gate = 0.0f;
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

	/* Down-ToF gives a SLANT range; convert to VERTICAL height with THIS estimator's FRESH attitude:
	 * cos(tilt) = R[8] (world-z projection of the body-down beam), exact -- no cached/Gibbs approx.
	 * Past ~87 deg off vertical (R[8] <= 0.05) the beam no longer measures the floor below, so treat
	 * the sample as unavailable. This tilt/height fusion is the ToF measurement model, so it lives
	 * here (not as a main.cpp preprocessing step); main.cpp passes the raw slant. */
	float ctilt = R[8];
	bool  tof_vert = tof_valid && ctilt > 0.05f;
	float h_vert = height * ctilt;

	/* update: horizontal velocity from optical flow (body->world). If there is no valid flow
	 * sample, fall back to one of two behaviors:
	 *   - ON THE GROUND (ToF says grounded): the vehicle is at rest, so apply a zero-velocity
	 *     update (ZUPT) -- measure v=0. Without it the x/y velocity has NO observation on the
	 *     bench (flow is gated by low height / near-field SQUAL), so the KF integrates accel
	 *     bias + attitude gravity-leak into an unbounded velocity drift. The ZUPT anchors vx/vy
	 *     and, through the pos/vel/bias covariance, makes the x/y accel bias observable.
	 *   - IN THE AIR (flow dropout at altitude): the sample is merely stale and the vehicle may
	 *     genuinely be translating, so run velocity predict-only (covariance grows -> the next
	 *     valid flow sample is weighted more). We must NOT force zero here. */
	if (flow_valid && ROSE_FLOW_FUSE) {   /* ROSE_FLOW_FUSE=0 -> skip flow, dead-reckon + on-ground ZUPT */
		float vfx = R[0]*flow[0] + R[1]*flow[1];
		float vfy = R[3]*flow[0] + R[4]*flow[1];
		/* Height-scale the measurement variance (v = flow*height -> noise variance ~ height^2), and
		 * trust the noisier y axis less. This is the KF's proper smoothing knob (no separate LPF). */
		float hs = 1.0f + (h_vert * h_vert) / (flow_href * flow_href);
		kx.update_vel(vfx, r_flow_x * hs, flow_gate);
		ky.update_vel(vfy, r_flow_y * hs, flow_gate);
	} else if (tof_vert && h_vert < zupt_height) {
		kx.update_vel(0.0f, r_zupt, 0.0f);
		ky.update_vel(0.0f, r_zupt, 0.0f);
	}

	/* update: altitude from downward ToF (position measurement) -- only when a fresh
	 * low-rate ToF sample is available. Between samples the z-axis KF runs predict-only
	 * (accel), and its covariance grows so the next ToF correction is weighted more. */
	if (tof_vert) {
		kz.update_pos(h_vert, r_tof, tof_gate);
	}

	/* Hard backstop against non-physical horizontal-velocity runaway. With the flow gate off the
	 * flow normally pulls velocity back, but a flow dropout (predict-only) can still integrate accel
	 * without bound -- clamp the velocity state to a generous physical max the vehicle cannot exceed,
	 * so a garbage estimate can never reach the controller (which turns it into a huge tilt command). */
	const float VMAX = 5.0f;
	if (kx.v >  VMAX) { kx.v =  VMAX; } else if (kx.v < -VMAX) { kx.v = -VMAX; }
	if (ky.v >  VMAX) { ky.v =  VMAX; } else if (ky.v < -VMAX) { ky.v = -VMAX; }
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

	/* translation: predict pos += vel*h + 0.5 a h^2, vel += a*h, using the BIAS-CORRECTED
	 * world acceleration (a - ba) consistent with the KF predict, so the lead prediction is
	 * not re-corrupted by the estimated accel bias. */
	float cax = awx - kx.ba, cay = awy - ky.ba, caz = awz - kz.ba;
	state[0] = kx.p + kx.v*h + 0.5f*cax*h*h;
	state[1] = ky.p + ky.v*h + 0.5f*cay*h*h;
	state[2] = kz.p + kz.v*h + 0.5f*caz*h*h;
	state[3] = px/qwv; state[4] = py/qwv; state[5] = pz/qwv;
	state[6] = kx.v + cax*h; state[7] = ky.v + cay*h; state[8] = kz.v + caz*h;
	/* BODY-frame angular rates (raw gyro). The PID controller's rate loop and the documented
	 * [9..11] ~ body-rate convention expect BODY rates here; emitting world-frame R*gyro instead
	 * rotates the fast rate-loop feedback by the (drifting, mag-less) yaw angle -> roll/pitch
	 * cross-coupling -> the takeoff flip. Matches the complementary filter, which flew. */
	state[9] = gx;  state[10] = gy; state[11] = gz;
}
