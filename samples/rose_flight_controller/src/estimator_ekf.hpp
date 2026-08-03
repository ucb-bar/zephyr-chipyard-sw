/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * EKF-style estimator: quaternion Mahony attitude (shared) + a per-axis Kalman filter for
 * translation. Each axis is a 2-state [position, velocity] KF driven by the world-frame
 * accelerometer (predict) and corrected by a measurement (update):
 *   - x, y: measure world-frame VELOCITY from optical flow  -> velocity tracks flow
 *           tightly (the hover goal: keep velocity ~0), position dead-reckons (drift OK).
 *   - z:    measure POSITION from the downward ToF          -> altitude + climb rate
 *           observed via the position/velocity cross-covariance (lower latency + smoother
 *           vz than the fixed-gain complementary filter, which improves the loop's
 *           stability margin).
 *
 * "Extended" because the linear per-axis KFs are driven by the accelerometer rotated into
 * the world frame through the nonlinear attitude estimate.
 */

#ifndef ROSE_ESTIMATOR_EKF_HPP
#define ROSE_ESTIMATOR_EKF_HPP

#include "estimator.hpp"
#include "attitude_mahony.hpp"

/* 3-state [pos, vel, accel_bias] Kalman filter for one translational axis.
 *
 * The accel-bias state `ba` is the estimator-hardening fix (stress plan section 4, item 1):
 * a constant accelerometer bias, fed through predict, otherwise shows up as a persistent
 * measurement innovation that a 2-state filter absorbs into a *position* offset -> with the
 * no-integrator controller that becomes a steady altitude offset (the dominant L1/L2 noise
 * failure). With a bias state the filter attributes that persistent innovation to `ba`,
 * estimates it (observable via ToF position for z, optical-flow velocity for x/y), and
 * subtracts it in predict, so pos/vel are unbiased. The Kalman gain sets the sign, so this
 * is self-correcting (unlike a hand-tuned integral trim). Reduces to the old Kf2 behavior if
 * sigma_ba2 -> 0 and P22 -> 0.
 */
struct Kf3 {
	float p, v, ba;                            /* state: position, velocity, accel bias */
	float P00, P01, P02, P11, P12, P22;        /* covariance (symmetric 3x3, upper) */
	float sigma_a2;                            /* accel process-noise variance */
	float sigma_ba2;                           /* accel-bias random-walk variance */

	void init(float p0, float sigma_a, float sigma_ba, float p22_0)
	{
		p = p0; v = 0.0f; ba = 0.0f;
		P00 = 1e-2f; P11 = 1e-2f; P22 = p22_0;
		P01 = P02 = P12 = 0.0f;
		sigma_a2 = sigma_a * sigma_a;
		sigma_ba2 = sigma_ba * sigma_ba;
	}

	/* Predict with measured world-frame acceleration a_meas; the true accel used for the
	 * kinematics is (a_meas - ba). F = [[1,dt,-dt^2/2],[0,1,-dt],[0,0,1]]. */
	void predict(float a_meas, float dt)
	{
		float h2 = 0.5f * dt * dt;
		float a = a_meas - ba;
		p += v * dt + h2 * a;
		v += a * dt;
		/* ba unchanged (random-walk mean) */

		/* A = F * P */
		float a00 = P00 + dt * P01 - h2 * P02;
		float a01 = P01 + dt * P11 - h2 * P12;
		float a02 = P02 + dt * P12 - h2 * P22;
		float a10 = P01 - dt * P02;
		float a11 = P11 - dt * P12;
		float a12 = P12 - dt * P22;
		float a20 = P02, a21 = P12, a22 = P22;
		/* P' = A * F^T */
		float np00 = a00 + a01 * dt - a02 * h2;
		float np01 = a01 - a02 * dt;
		float np02 = a02;
		float np11 = a11 - a12 * dt;
		float np12 = a12;
		float np22 = a22;
		(void)a10; (void)a20; (void)a21;   /* symmetric: upper triangle suffices */
		/* process noise: accel white noise into (p,v); bias random walk into ba */
		float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
		np00 += sigma_a2 * dt4 * 0.25f;
		np01 += sigma_a2 * dt3 * 0.5f;
		np11 += sigma_a2 * dt2;
		np22 += sigma_ba2 * dt2;
		P00 = np00; P01 = np01; P02 = np02; P11 = np11; P12 = np12; P22 = np22;
	}

	/* Normalized innovation squared for a scalar measurement -> chi-square outlier gate. A
	 * measurement is rejected (predict-only this step, covariance keeps growing) when
	 * resid^2 / S exceeds `gate` (gate <= 0 disables). Rejects flow/ToF spikes that would
	 * otherwise corrupt velocity/altitude (stress plan section 4, item 3). */
	static bool gated(float resid, float S, float gate)
	{
		return gate > 0.0f && resid * resid > gate * S;
	}

	/* Correct with a position measurement (H = [1,0,0], variance r); chi-square gated. */
	bool update_pos(float meas, float r, float gate)
	{
		float S = P00 + r;
		float resid = meas - p;
		if (gated(resid, S, gate)) {
			return false;
		}
		float K0 = P00 / S, K1 = P01 / S, K2 = P02 / S;
		p += K0 * resid; v += K1 * resid; ba += K2 * resid;
		/* P = (I - K H) P, H picks row 0 -> subtract K * [P00,P01,P02] */
		float np00 = P00 - K0 * P00;
		float np01 = P01 - K0 * P01;
		float np02 = P02 - K0 * P02;
		float np11 = P11 - K1 * P01;
		float np12 = P12 - K1 * P02;
		float np22 = P22 - K2 * P02;
		P00 = np00; P01 = np01; P02 = np02; P11 = np11; P12 = np12; P22 = np22;
		return true;
	}

	/* Correct with a velocity measurement (H = [0,1,0], variance r); chi-square gated. */
	bool update_vel(float meas, float r, float gate)
	{
		float S = P11 + r;
		float resid = meas - v;
		if (gated(resid, S, gate)) {
			return false;
		}
		float K0 = P01 / S, K1 = P11 / S, K2 = P12 / S;
		p += K0 * resid; v += K1 * resid; ba += K2 * resid;
		/* P = (I - K H) P, H picks row 1 -> subtract K * [P01,P11,P12] */
		float np00 = P00 - K0 * P01;
		float np01 = P01 - K0 * P11;
		float np02 = P02 - K0 * P12;
		float np11 = P11 - K1 * P11;
		float np12 = P12 - K1 * P12;
		float np22 = P22 - K2 * P12;
		P00 = np00; P01 = np01; P02 = np02; P11 = np11; P12 = np12; P22 = np22;
		return true;
	}
};

class EkfEstimator : public IStateEstimator {
public:
	void init(float x0, float y0, float z0) override;
	void update(const float accel[3], const float gyro[3], const float flow[2],
		    bool flow_valid, float height, bool tof_valid, float dt) override;
	void get_state(float state[EST_NSTATES]) const override;
	const char *name() const override { return "EKF (Mahony + per-axis Kalman)"; }

private:
	MahonyAttitude att;
	Kf3 kx, ky, kz;
	float gx, gy, gz;
	float awx, awy, awz;   /* last world-frame acceleration (for delay prediction) */
	float dt_last;
	float r_flow;          /* optical-flow velocity measurement variance */
	float r_tof;           /* ToF height measurement variance */
	float flow_gate;       /* chi-square gate (NIS) for flow velocity updates */
	float tof_gate;        /* chi-square gate (NIS) for ToF position updates */
	float delay_steps;     /* known actuation delay, in control steps (model-based
	                        * delay compensation; NOT a tuned per-channel gain) */
};

#endif /* ROSE_ESTIMATOR_EKF_HPP */
