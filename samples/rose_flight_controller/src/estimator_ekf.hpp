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

/* 2-state [pos, vel] Kalman filter for one translational axis. */
struct Kf2 {
	float p, v;             /* state */
	float P00, P01, P11;    /* covariance (symmetric) */
	float sigma_a2;         /* accel process-noise variance (m/s^2)^2 */

	void init(float p0, float sigma_a)
	{
		p = p0; v = 0.0f;
		P00 = 1e-2f; P01 = 0.0f; P11 = 1e-2f;
		sigma_a2 = sigma_a * sigma_a;
	}

	/* Predict with world-frame acceleration a over dt (constant-accel model). */
	void predict(float a, float dt)
	{
		p += v * dt + 0.5f * a * dt * dt;
		v += a * dt;
		float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
		float q00 = sigma_a2 * dt4 * 0.25f;
		float q01 = sigma_a2 * dt3 * 0.5f;
		float q11 = sigma_a2 * dt2;
		float np00 = P00 + 2.0f * dt * P01 + dt2 * P11 + q00;
		float np01 = P01 + dt * P11 + q01;
		float np11 = P11 + q11;
		P00 = np00; P01 = np01; P11 = np11;
	}

	/* Correct with a position measurement (variance r). */
	void update_pos(float meas, float r)
	{
		float S = P00 + r;
		float K0 = P00 / S, K1 = P01 / S;
		float resid = meas - p;
		p += K0 * resid; v += K1 * resid;
		float np00 = P00 - K0 * P00;
		float np01 = P01 - K0 * P01;
		float np11 = P11 - K1 * P01;
		P00 = np00; P01 = np01; P11 = np11;
	}

	/* Correct with a velocity measurement (variance r). */
	void update_vel(float meas, float r)
	{
		float S = P11 + r;
		float K0 = P01 / S, K1 = P11 / S;
		float resid = meas - v;
		p += K0 * resid; v += K1 * resid;
		float np00 = P00 - K0 * P01;
		float np01 = P01 - K0 * P11;
		float np11 = P11 - K1 * P11;
		P00 = np00; P01 = np01; P11 = np11;
	}
};

class EkfEstimator : public IStateEstimator {
public:
	void init(float x0, float y0, float z0) override;
	void update(const float accel[3], const float gyro[3], const float flow[2],
		    float height, float dt) override;
	void get_state(float state[EST_NSTATES]) const override;
	const char *name() const override { return "EKF (Mahony + per-axis Kalman)"; }

private:
	MahonyAttitude att;
	Kf2 kx, ky, kz;
	float gx, gy, gz;
	float awx, awy, awz;   /* last world-frame acceleration (for delay prediction) */
	float dt_last;
	float r_flow;          /* optical-flow velocity measurement variance */
	float r_tof;           /* ToF height measurement variance */
	float delay_steps;     /* known actuation delay, in control steps (model-based
	                        * delay compensation; NOT a tuned per-channel gain) */
};

#endif /* ROSE_ESTIMATOR_EKF_HPP */
