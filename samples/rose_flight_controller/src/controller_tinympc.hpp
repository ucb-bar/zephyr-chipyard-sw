/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * TinyMPC controller: constrained model-predictive control (the original/default control law).
 * Wraps the TinyMPC solver behind IController; solver caches + workspace live in the .cpp.
 */
#ifndef ROSE_CONTROLLER_TINYMPC_HPP
#define ROSE_CONTROLLER_TINYMPC_HPP

#include "controller.hpp"

struct TinympcController : IController {
	void init() override;
	void compute(const float state[CTRL_NSTATES], const float setpoint[CTRL_NSTATES],
		     float u_out[CTRL_NACTIONS], float dt) override;
	const char *name() const override { return "TinyMPC (constrained MPC)"; }
};

#endif /* ROSE_CONTROLLER_TINYMPC_HPP */
