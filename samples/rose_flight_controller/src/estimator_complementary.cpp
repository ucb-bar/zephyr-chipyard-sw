/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 */

#include "estimator_complementary.hpp"

#define GRAVITY 9.81f
#define ZUPT_HEIGHT_M 0.05f   /* on-ground band (ToF height) for the zero-velocity update */
#ifndef ROSE_FLOW_FUSE
#define ROSE_FLOW_FUSE 1      /* 0 = don't fuse optical flow -> pure dead-reckoning + on-ground ZUPT */
#endif
/* Per-axis flow trust (blend weight): 1.0 = replace velocity with flow every sample (zero lag but
 * passes all flow noise through); lower = heavier low-pass toward the accel dead-reckoned velocity
 * (smoother, laggier). The PMW3901's y (left) axis reads garbage on directional/low-texture floors
 * (measured ~7x worse than x, railing to +/-3 m/s while nearly stationary), so de-trust y on its own
 * -DFLOW_GAIN_Y=<0..1> instead of following it 1:1 into a velocity-watchdog trip / divergent bank. */
#ifndef FLOW_GAIN_X
#define FLOW_GAIN_X 1.0f
#endif
#ifndef FLOW_GAIN_Y
#define FLOW_GAIN_Y 1.0f
#endif

/* ---- Barometer (BMP388) altitude fusion (ROSE_BARO) -------------------------------------------
 * The down-ToF is the absolute-to-floor altitude anchor when valid, but it drops out under large
 * tilt (R[8] <= 0.05 -> tof_vert=false) and STEP-JUMPS over obstacles/desks. The barometer supplies
 * a smooth, tilt-immune RELATIVE altitude (baro_rel, referenced to the arm point in main.cpp) to
 * coast through both. Strategy (complementary blend): while the ToF agrees with the current estimate
 * (within BARO_TOF_STEP_M) it stays the low-frequency truth AND slowly (re)calibrates the baro bias;
 * when the ToF is invalid (tilt) or disagrees (obstacle jump / beam glitch) we reject it and lean on
 * the baro absolute estimate (baro_rel + baro_bias) so altitude never dead-reckons on accel alone.
 * OFF by default (ROSE_BARO=0) -> identical to the ToF-only filter. */
#ifndef ROSE_BARO
#define ROSE_BARO 0
#endif
#ifndef BARO_Z_GAIN
#define BARO_Z_GAIN 0.10f    /* z pull toward the baro estimate while coasting (ToF gap/jump) */
#endif
#ifndef BARO_VZ_GAIN
#define BARO_VZ_GAIN 0.05f   /* vz pull from the baro innovation (small: baro is noisy as a rate) */
#endif
#ifndef BARO_BIAS_GAIN
#define BARO_BIAS_GAIN 0.01f /* slow tracking of baro_bias to the ToF-anchored floor (drift cal) */
#endif
#ifndef BARO_TOF_STEP_M
#define BARO_TOF_STEP_M 0.15f/* |ToF - estimate| above this => reject ToF (obstacle/glitch), lean on baro */
#endif

/* ---- IMU lever-arm (off-CoM) compensation ----------------------------------------------------
 * The IMU sits at r (body frame) from the CoM, so it reads a_imu = a_com + alpha x r + omega x
 * (omega x r). Left uncompensated, those rotational terms integrate into velocity/position as fake
 * translation -- a rotation-correlated drift that is WORST on takeoff (the velocity integrator that
 * otherwise trims it hasn't wound up yet). r for riskybird v3 (PCB): IMU is 16 mm LEFT of the CoM
 * -> IMU_OFFSET_Y = -0.016. Default 0 (opt-in); set via -DIMU_OFFSET_* . */
#ifndef IMU_OFFSET_X
#define IMU_OFFSET_X 0.0f      /* body +x = forward */
#endif
#ifndef IMU_OFFSET_Y
#define IMU_OFFSET_Y 0.0f      /* body +y = right; PCB value -0.016 (16 mm left) */
#endif
#ifndef IMU_OFFSET_Z
#define IMU_OFFSET_Z 0.0f      /* body +z = up */
#endif
#ifndef IMU_ALPHA_TAU
#define IMU_ALPHA_TAU 0.02f    /* LP time-constant (s) for the gyro-derivative (angular accel) */
#endif

void ComplementaryEstimator::init(float x0, float y0, float z0)
{
	att.init();
	w_prev[0] = w_prev[1] = w_prev[2] = 0.0f;
	alpha_f[0] = alpha_f[1] = alpha_f[2] = 0.0f;
	have_wprev = false;
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
	baro_bias = 0.0f; baro_have = false;   /* baro reference locks on the first trusted ToF */
}

void ComplementaryEstimator::update(const float accel[3], const float gyro[3],
				    const float flow[2], bool flow_valid, float height, bool tof_valid,
				    float baro_rel, bool baro_valid, float dt)
{
	gx = gyro[0]; gy = gyro[1]; gz = gyro[2];

	/* IMU lever-arm compensation: a_com = a_imu - (alpha x r + omega x (omega x r)). alpha =
	 * LP-filtered d(omega)/dt. Skips cleanly (a == accel) when the offset is 0. */
	float a[3] = { accel[0], accel[1], accel[2] };
	{
		const float rx = IMU_OFFSET_X, ry = IMU_OFFSET_Y, rz = IMU_OFFSET_Z;
		if ((rx != 0.0f || ry != 0.0f || rz != 0.0f) && dt > 1e-6f) {
			if (have_wprev) {
				const float k = dt / (IMU_ALPHA_TAU + dt);
				alpha_f[0] += k * ((gx - w_prev[0]) / dt - alpha_f[0]);
				alpha_f[1] += k * ((gy - w_prev[1]) / dt - alpha_f[1]);
				alpha_f[2] += k * ((gz - w_prev[2]) / dt - alpha_f[2]);
			}
			w_prev[0] = gx; w_prev[1] = gy; w_prev[2] = gz; have_wprev = true;
			const float alx = alpha_f[0], aly = alpha_f[1], alz = alpha_f[2];
			/* alpha x r */
			const float axr0 = aly*rz - alz*ry, axr1 = alz*rx - alx*rz, axr2 = alx*ry - aly*rx;
			/* omega x r, then omega x (omega x r) */
			const float wr0 = gy*rz - gz*ry, wr1 = gz*rx - gx*rz, wr2 = gx*ry - gy*rx;
			const float wwr0 = gy*wr2 - gz*wr1, wwr1 = gz*wr0 - gx*wr2, wwr2 = gx*wr1 - gy*wr0;
			a[0] -= (axr0 + wwr0);
			a[1] -= (axr1 + wwr1);
			a[2] -= (axr2 + wwr2);
		}
	}

	att.update(a, gyro, dt);
	float R[9]; att.rot(R);
	/* Down-ToF SLANT range -> VERTICAL height via the fresh attitude: cos(tilt) = R[8] (exact). Past
	 * ~87 deg off vertical (R[8] <= 0.05) the beam isn't measuring the floor below -> unavailable. */
	float ctilt = R[8];
	bool  tof_vert = tof_valid && ctilt > 0.05f;
	float h_vert = height * ctilt;

	/* world acceleration = R f_body + g  (using the lever-arm-corrected body accel) */
	float ax_w = R[0]*a[0] + R[1]*a[1] + R[2]*a[2];
	float ay_w = R[3]*a[0] + R[4]*a[1] + R[5]*a[2];
	float az_w = R[6]*a[0] + R[7]*a[1] + R[8]*a[2] - GRAVITY;
	awx = ax_w; awy = ay_w; awz = az_w; dt_last = dt;

	vx += ax_w*dt; vy += ay_w*dt; vz += az_w*dt;

	/* optical-flow fusion for horizontal velocity -- only when the sample is fresh. On a
	 * dropout the flow is stale, so keep the accel dead-reckoned velocity (predict-only)
	 * rather than snapping to a stale measurement. */
	if (flow_valid && ROSE_FLOW_FUSE) {   /* ROSE_FLOW_FUSE=0 -> skip flow, dead-reckon + on-ground ZUPT */
		float vfx = R[0]*flow[0] + R[1]*flow[1];
		float vfy = R[3]*flow[0] + R[4]*flow[1];
		vx = (1.0f - FLOW_GAIN_X)*vx + FLOW_GAIN_X*vfx;
		vy = (1.0f - FLOW_GAIN_Y)*vy + FLOW_GAIN_Y*vfy;   /* y de-trusted (noisy PMW3901 left axis) */
	} else if (tof_vert && h_vert < ZUPT_HEIGHT_M) {
		/* On the ground with no valid flow, the vehicle is at rest -> zero-velocity update (ZUPT):
		 * measure v=0 instead of dead-reckoning the accelerometer, which otherwise ramps to ~1 m/s
		 * within seconds on the bench (nothing else observes horizontal velocity at rest, and the
		 * down-ToF reading ~0 keeps flow gated). Full trust (like flow_gain=1) since v=0 is exact at
		 * rest. Gated on-ground ONLY, so an in-air flow dropout still dead-reckons (may be moving). */
		vx = 0.0f;
		vy = 0.0f;
	}

	/* Physical-velocity backstop (mirrors estimator_ekf.cpp): a flow dropout dead-reckons vx/vy on
	 * the accelerometer without bound; clamp so a garbage velocity can't reach the controller. */
	const float VMAX = 5.0f;
	if (vx >  VMAX) { vx =  VMAX; } else if (vx < -VMAX) { vx = -VMAX; }
	if (vy >  VMAX) { vy =  VMAX; } else if (vy < -VMAX) { vy = -VMAX; }

	x += vx*dt; y += vy*dt; z += vz*dt;

	/* ToF altitude observer -- only on steps with a fresh (low-rate) ToF sample. Between
	 * samples z/vz dead-reckon on the accelerometer (fast layer); the ToF corrects the
	 * accumulated drift when it arrives (slow layer). With ROSE_BARO, the barometer fills
	 * the gaps where the ToF is invalid (tilt) or a step-jump (obstacle) is rejected. */
#if ROSE_BARO
	if (baro_valid) {
		float baro_z = baro_rel + baro_bias;   /* baro's floor-referenced absolute altitude */
		bool  tof_ok = false;
		if (tof_vert) {
			float rz = h_vert - z;
			/* Gate the ToF against the baro-propagated estimate: a step larger than
			 * BARO_TOF_STEP_M is an obstacle/desk edge or beam glitch, not true altitude ->
			 * reject it and coast on the smooth baro. Before the reference is locked
			 * (baro_have==false) trust the ToF unconditionally to seed baro_bias. */
			if (!baro_have || fabsf(rz) < BARO_TOF_STEP_M) {
				z  += z_gain*rz;
				vz += vz_gain*rz;
				tof_ok = true;
				if (!baro_have) { baro_bias = z - baro_rel; baro_have = true; }
				else            { baro_bias += BARO_BIAS_GAIN*((z - baro_rel) - baro_bias); }
			}
		}
		if (!tof_ok && baro_have) {
			float rzb = baro_z - z;   /* lean on baro through the ToF gap / rejected jump */
			z  += BARO_Z_GAIN*rzb;
			vz += BARO_VZ_GAIN*rzb;
		}
	} else if (tof_vert) {
		float rz = h_vert - z;
		z  += z_gain*rz;
		vz += vz_gain*rz;
	}
#else
	(void)baro_rel; (void)baro_valid;   /* barometer disabled -> ToF-only (unchanged behavior) */
	if (tof_vert) {
		float rz = h_vert - z;
		z  += z_gain*rz;
		vz += vz_gain*rz;
	}
#endif
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
