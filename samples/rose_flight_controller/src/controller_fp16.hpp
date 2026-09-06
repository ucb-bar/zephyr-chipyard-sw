/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * fp16 (Zvfh) + fp32-vector hierarchical PID controller: the cascaded altitude/velocity/attitude/
 * rate loops with the instantaneous gains, 4x4 mixer, cos() polynomial and force->duty (incl.
 * vfsqrt.v) on the fp16 vector unit, and the PID integrators kept in fp32. Saturation clamps guard
 * the fp16 mixer/force path (no overflow->NaN). Matches the fp32 PID off-board (u RMS ~5e-4 of the
 * normalized-thrust range). Drops in behind IController; select with -DROSE_USE_FP16 in the factory.
 *
 * Thin wrapper over the shared C kernels (fp16_fc_kernels.*). STAGED for the FC co-residency tree
 * (which owns controller_factory.cpp / IController); add the factory branch shown in the header of
 * this file when merging there.
 *
 *   // controller_factory.cpp:
 *   #if ROSE_USE_FP16
 *   #include "controller_fp16.hpp"
 *   static Fp16Controller g_controller;
 *   #elif ROSE_USE_PID
 *   ...
 */
#ifndef ROSE_CONTROLLER_FP16_HPP
#define ROSE_CONTROLLER_FP16_HPP

#include "controller.hpp"
extern "C" {
#include "fp16_fc_kernels.h"
}

class Fp16Controller : public IController {
public:
	void init() override { kfc_init(&k_, 0.0f, 0.0f, 0.0f); /* only the PID integrators are used */ }

	void compute(const float state[CTRL_NSTATES], const float setpoint[CTRL_NSTATES],
		     float u_out[CTRL_NACTIONS], float dt) override {
		kfc_control(&k_, state, setpoint, u_out, dt);
	}

	const char *name() const override { return "fp16(Zvfh)+fp32 hierarchical PID"; }

private:
	kfc_state k_;   /* carries alt_int / vel_int / slew memory (the PID accumulators) */
};

#endif /* ROSE_CONTROLLER_FP16_HPP */
