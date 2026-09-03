/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Complementary/dead-reckoning estimator: quaternion Mahony attitude + fixed-gain
 * translation (accel integration, optical-flow velocity blend, ToF altitude observer).
 * Simple and cheap; the EKF variant tracks velocity/altitude with lower latency.
 */

#ifndef ROSE_ESTIMATOR_COMPLEMENTARY_HPP
#define ROSE_ESTIMATOR_COMPLEMENTARY_HPP

#include "estimator.hpp"
#include "attitude_mahony.hpp"

class ComplementaryEstimator : public IStateEstimator {
public:
	void init(float x0, float y0, float z0) override;
	void update(const float accel[3], const float gyro[3], const float flow[2],
		    bool flow_valid, float height, bool tof_valid,
		    float baro_rel, bool baro_valid, float dt) override;
	void get_state(float state[EST_NSTATES]) const override;
	const char *name() const override { return "complementary (Mahony + fixed-gain)"; }

private:
	MahonyAttitude att;
	float x, y, z;
	float vx, vy, vz;
	float gx, gy, gz;
	float awx, awy, awz;   /* last world-frame acceleration (for lead prediction) */
	float dt_last;
	float flow_gain, z_gain, vz_gain, lead, lead_att;
	float baro_bias;       /* floor-referenced altitude minus baro_rel: baro_z = baro_rel + baro_bias */
	bool  baro_have;       /* baro_bias locked to a trusted ToF reference (ROSE_BARO) */
	/* IMU lever-arm (off-CoM) compensation: previous body rates + LP-filtered angular accel. */
	float w_prev[3];
	float alpha_f[3];
	bool  have_wprev;
};

#endif /* ROSE_ESTIMATOR_COMPLEMENTARY_HPP */
