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
		    float height, bool tof_valid, float dt) override;
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
};

#endif /* ROSE_ESTIMATOR_COMPLEMENTARY_HPP */
