/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * fp16 (Zvfh) + fp32-vector state estimator: the complementary filter (Mahony attitude +
 * dead-reckoning translation) with its instantaneous algebra on the fp16 vector unit and its
 * drift-sensitive accumulators in fp32 vector. Numerically matches the fp32 complementary filter
 * (validated off-board: pos RMS 0.4 mm, att 0.05 deg vs the fp32 golden) at a fraction of the
 * soft-float cost. Drops in behind IStateEstimator; selected by -DROSE_USE_FP16 in the factory.
 *
 * Implemented as a thin wrapper over the shared C kernels (fp16_fc_kernels.*), so the identical
 * code is validated in the off-board Spike/pybullet harness and deployed here.
 */
#ifndef ROSE_ESTIMATOR_FP16_HPP
#define ROSE_ESTIMATOR_FP16_HPP

#include "estimator.hpp"
#include <zephyr/kernel.h>   /* irq_lock/irq_unlock */
extern "C" {
#include "fp16_fc_kernels.h"
}

class Fp16Estimator : public IStateEstimator {
public:
	void init(float x0, float y0, float z0) override {
		kfc_init(&k_, x0, y0, z0);
		for (int i = 0; i < EST_NSTATES; i++) state_[i] = 0.0f;
	}

	/* The kernel fuses update + get_state in one pass (it needs the fresh attitude for the ToF
	 * tilt correction and the lead prediction), so update() runs it and caches the 12-DoF state;
	 * get_state() returns the cache. (ToF-only complementary, matching the ROSE_BARO=0 default.) */
	void update(const float accel[3], const float gyro[3], const float flow[2],
		    bool flow_valid, float height, bool tof_valid,
		    float baro_rel, bool baro_valid, float dt) override {
		(void)baro_rel; (void)baro_valid;   /* fp16 kernel is ToF-only complementary */
		/* vstate-hazard mitigation #1: the fp16 kernel issues Saturn V ops from the
		 * high-prio control loop; an ISR (timer/i2c) preempting mid-vector-op mis-resumes
		 * the strided vle/vse (bench: mcause-7 wild store in ve_alpha_inc). irq_lock the
		 * ~us V compute so no interrupt lands mid-op. Bounded jitter (a few us at 35 MHz). */
		unsigned int _vk = irq_lock();
		kfc_estimate(&k_, accel, gyro, flow, flow_valid ? 1 : 0, height,
			     tof_valid ? 1 : 0, dt, state_);
		irq_unlock(_vk);
	}

	void get_state(float state[EST_NSTATES]) const override {
		for (int i = 0; i < EST_NSTATES; i++) state[i] = state_[i];
	}

	const char *name() const override { return "fp16(Zvfh)+int-accum complementary"; }

private:
	kfc_state k_;
	float state_[EST_NSTATES];
};

#endif /* ROSE_ESTIMATOR_FP16_HPP */
