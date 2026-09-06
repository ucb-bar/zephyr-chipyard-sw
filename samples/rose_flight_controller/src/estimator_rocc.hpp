/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoCC (FcRoCC custom-0) state estimator drop-in. The estimator runs entirely on
 * the hardened FC RoCC accelerator in Q24.24 integer (no FPU, no Saturn V) --
 * fc_rocc_estimate_q() streams the sensor inputs in, RUNs the EST program, and
 * pops the 12-DoF state. Selected by -DROSE_USE_ROCC=1 in estimator_factory.cpp.
 *
 * Only the float<->Q24.24 boundary conversion is scalar (soft-float on this
 * misa.F=0 core -- a handful of ops per tick; the heavy estimator math is on the
 * RoCC). The RoCC's regfile persists the computed state, which the RoCC controller
 * (controller_rocc.hpp) reads directly on its own RUN -- so est.update() MUST run
 * before ctrl.compute() each tick (it does: main calls estimator then controller).
 */
#ifndef ROSE_ESTIMATOR_ROCC_HPP
#define ROSE_ESTIMATOR_ROCC_HPP

#include "estimator.hpp"
#include <stdint.h>
extern "C" {
#include "fc_rocc_fw.h"
void fcr_boot(void);
void fc_rocc_estimate_q(const int64_t in_q[10], int flow_valid, int tof_valid,
			int64_t state_out_q[12]);
}

class RoccEstimator : public IStateEstimator {
public:
	void init(float x0, float y0, float z0) override {
		(void)x0; (void)y0; (void)z0;   /* initial pose comes from the FCR_INIT boot image */
		static bool booted = false;      /* boot the RoCC regfile once (idempotent guard) */
		if (!booted) { fcr_boot(); booted = true; }
		for (int i = 0; i < EST_NSTATES; i++) state_[i] = 0.0f;
	}

	void update(const float accel[3], const float gyro[3], const float flow[2],
		    bool flow_valid, float height, bool tof_valid,
		    float baro_rel, bool baro_valid, float dt) override {
		(void)baro_rel; (void)baro_valid;   /* RoCC estimator is ToF-only complementary */
		const float Q24 = 16777216.0f;      /* 2^FCR_QA, FCR_QA=24 : all inputs are Q24.24 */
		int64_t in_q[10] = {
			(int64_t)(accel[0] * Q24), (int64_t)(accel[1] * Q24), (int64_t)(accel[2] * Q24),
			(int64_t)(gyro[0]  * Q24), (int64_t)(gyro[1]  * Q24), (int64_t)(gyro[2]  * Q24),
			(int64_t)(flow[0]  * Q24), (int64_t)(flow[1]  * Q24),
			(int64_t)(height   * Q24), (int64_t)(dt       * Q24),
		};
		int64_t st_q[12];
		fc_rocc_estimate_q(in_q, flow_valid ? 1 : 0, tof_valid ? 1 : 0, st_q);
		/* per-field output scales: pos Q_POS, att QA, vel Q_VEL, rates QA */
		const float iPOS = 1.0f / (float)((uint32_t)1 << FCR_Q_POS);
		const float iA   = 1.0f / (float)((uint32_t)1 << FCR_QA);
		const float iVEL = 1.0f / (float)((uint32_t)1 << FCR_Q_VEL);
		state_[0] = (float)st_q[0] * iPOS; state_[1] = (float)st_q[1] * iPOS; state_[2] = (float)st_q[2] * iPOS;
		state_[3] = (float)st_q[3] * iA;   state_[4] = (float)st_q[4] * iA;   state_[5] = (float)st_q[5] * iA;
		state_[6] = (float)st_q[6] * iVEL; state_[7] = (float)st_q[7] * iVEL; state_[8] = (float)st_q[8] * iVEL;
		state_[9] = (float)st_q[9] * iA;   state_[10] = (float)st_q[10] * iA; state_[11] = (float)st_q[11] * iA;
	}

	void get_state(float state[EST_NSTATES]) const override {
		for (int i = 0; i < EST_NSTATES; i++) state[i] = state_[i];
	}

	const char *name() const override { return "RoCC(Q24.24 FcRoCC) complementary"; }

private:
	float state_[EST_NSTATES];
};

#endif /* ROSE_ESTIMATOR_ROCC_HPP */
