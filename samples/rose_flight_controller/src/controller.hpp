/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pluggable CONTROLLER interface for the RoSE flight controller -- the control-law analog of
 * IStateEstimator. Every controller maps the 12-DoF state (+ hover setpoint) to 4 normalized
 * motor thrusts; the active one is selected at build time (controller_factory.cpp,
 * -DROSE_USE_PID). New control laws drop in behind this interface without touching main.cpp.
 * Implementations:
 *   - TinympcController (controller_tinympc.*): constrained TinyMPC (model-predictive); default.
 *   - HierarchicalPidController (controller_pid.*): cascaded altitude/attitude/rate first-order
 *     control, ported from the original riskybird FreeRTOS firmware.
 */
#ifndef ROSE_CONTROLLER_HPP
#define ROSE_CONTROLLER_HPP

#define CTRL_NSTATES  12
#define CTRL_NACTIONS 4

struct IController {
	/* One-time setup (solver caches / gains). Called after the estimator is initialized. */
	virtual void init() = 0;

	/* One control step. state/setpoint are the 12-DoF vector
	 *   [x, y, z, r1, r2, r3, vx, vy, vz, wx, wy, wz]   (Rodrigues attitude; for small angles
	 *   r1/r2/r3 ~ roll/pitch/yaw). x/y are unobservable from flow -- regulate velocity.
	 * u_out: 4 normalized motor thrusts in the ~[-0.583, 0.417] convention (u + 0.583 = per-motor
	 * duty in [0,1]), consumed by send_control() (which clamps and applies the bench cap). */
	virtual void compute(const float state[CTRL_NSTATES], const float setpoint[CTRL_NSTATES],
			     float u_out[CTRL_NACTIONS], float dt) = 0;

	/* Human-readable name (boot log). */
	virtual const char *name() const = 0;

	virtual ~IController() {}
};

/* Build-time-selected controller singleton (-DROSE_USE_PID=1 -> hierarchical PID, else TinyMPC).
 * Defined in controller_factory.cpp. */
IController &active_controller();

#endif /* ROSE_CONTROLLER_HPP */
