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
	/* Clear the integrators / held state so EACH flight (and each soft-RESET) starts fresh. Without
	 * this the altitude integral (auto hover-thrust) and velocity integral (drift trim) carry over
	 * between flights when the on-ground reset doesn't fire -> the next flight over-thrusts (altitude
	 * overshoot -> height watchdog) and over-tilts (growing drift). */
	void init() override {
		pos_ref_1 = pos_ref_2 = 0.0f;
		alt_int = 0.0f;
		vel_int_1 = vel_int_2 = 0.0f;
		desRoll_prev = desPitch_prev = 0.0f;
	}
	void compute(const float state[CTRL_NSTATES], const float setpoint[CTRL_NSTATES],
		     float u_out[CTRL_NACTIONS], float dt) override;
	const char *name() const override { return "hierarchical PID (cascaded)"; }

private:
	float pos_ref_1 = 0.0f, pos_ref_2 = 0.0f;         /* ROSE_POS_LOOP dead-reckoned position ref */
	float alt_int = 0.0f;                             /* altitude integral (KI_HEIGHT) -> hover FF */
	float vel_int_1 = 0.0f, vel_int_2 = 0.0f;         /* velocity integral (KI_VEL) -> drift trim */
	float desRoll_prev = 0.0f, desPitch_prev = 0.0f;  /* tilt slew-limiter state */
};

#endif /* ROSE_CONTROLLER_PID_HPP */
