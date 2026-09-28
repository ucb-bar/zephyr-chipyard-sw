/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoCC (FcRoCC custom-0) controller drop-in. The hierarchical PID runs on the
 * hardened FC RoCC in Q24.24 integer (no FPU, no Saturn V): fc_rocc_control_q()
 * streams the setpoint in, RUNs the CTRL program (which reads the 12-DoF state the
 * estimator RUN left in the RoCC regfile), and pops the 4 normalized-thrust duties.
 * Selected by -DROSE_USE_ROCC=1 in controller_factory.cpp.
 *
 * The `state` arg is intentionally UNUSED: the RoCC controller consumes the
 * estimator's state directly from the RoCC regfile (persisted by estimate_q's RUN
 * earlier in the same tick), so RoccEstimator::update() must run first each tick.
 * Only the float<->Q24.24 setpoint/duty boundary is scalar (soft-float, a few
 * ops/tick). RoCC RUN is a single blocking custom-0 instruction (atomic -- no
 * mid-op preemption), so the RoCC FC issues no Saturn V ops (avoids vstate #1).
 */
#ifndef ROSE_CONTROLLER_ROCC_HPP
#define ROSE_CONTROLLER_ROCC_HPP

#include "controller.hpp"
#include <stdint.h>
extern "C" {
#include "fc_rocc_fw.h"
void fcr_boot(void);
void fc_rocc_control_q(int64_t desH_q, int64_t desV1_q, int64_t desV2_q,
		       int64_t yawT_q, int64_t dt_q, int64_t u_out_q[4]);
}

class RoccController : public IController {
public:
	void init() override {
		/* Idempotent guard: fcr_boot() is normally done in RoccEstimator::init()
		 * (called first); repeat here in case only the controller is RoCC. */
		static bool booted = false;
		if (!booted) { fcr_boot(); booted = true; }
	}

	void compute(const float state[CTRL_NSTATES], const float setpoint[CTRL_NSTATES],
		     float u_out[CTRL_NACTIONS], float dt) override {
		(void)state;   /* RoCC controller reads state from its own regfile (estimate_q's RUN) */
		const float Q24 = 16777216.0f;   /* 2^24 : setpoint + dt are Q24.24 */
		/* setpoint map (matches the fp16/PID kernels): [2]=desH, [6]=desV1, [7]=desV2, [5]=yawT */
		int64_t u_q[4];
		fc_rocc_control_q((int64_t)(setpoint[2] * Q24), (int64_t)(setpoint[6] * Q24),
				  (int64_t)(setpoint[7] * Q24), (int64_t)(setpoint[5] * Q24),
				  (int64_t)(dt * Q24), u_q);
		const float iQ24 = 1.0f / Q24;   /* u_out_q is Q24.24 (duty - 0.583); send_control adds 0.583 + clamps */
		for (int i = 0; i < CTRL_NACTIONS; i++) u_out[i] = (float)u_q[i] * iQ24;
	}

	const char *name() const override { return "RoCC(Q24.24 FcRoCC) hierarchical PID"; }
};

#endif /* ROSE_CONTROLLER_ROCC_HPP */
