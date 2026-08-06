/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hierarchical (cascaded) PID controller -- the alternate control law, ported from the original
 * riskybird FreeRTOS firmware (CobbledSteel/riskybird-firmware, main/i2c_simple_main.cpp).
 * Four nested first-order loops: altitude -> horizontal velocity -> attitude -> body rate,
 * then a physical mixer maps [thrust, roll/pitch/yaw moment] to 4 motor duties. Stateless
 * (pure P/PD cascade, no integrators), so compute() is a pure function of state + setpoint.
 */
#ifndef ROSE_CONTROLLER_PID_HPP
#define ROSE_CONTROLLER_PID_HPP

#include "controller.hpp"

struct HierarchicalPidController : IController {
	void init() override {}   /* fixed gains -- nothing to set up */
	void compute(const float state[CTRL_NSTATES], const float setpoint[CTRL_NSTATES],
		     float u_out[CTRL_NACTIONS], float dt) override;
	const char *name() const override { return "hierarchical PID (cascaded)"; }
};

#endif /* ROSE_CONTROLLER_PID_HPP */
