/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Build-time selection of the active state estimator. Default is the EKF; build with
 * -DROSE_USE_EKF=0 to fall back to the complementary filter. Adding a new filter is just
 * a new IStateEstimator subclass plus a branch here.
 */

#include "estimator.hpp"
#include "estimator_complementary.hpp"
#include "estimator_ekf.hpp"

#ifndef ROSE_USE_EKF
#define ROSE_USE_EKF 1
#endif
#ifndef ROSE_USE_FP16
#define ROSE_USE_FP16 0
#endif
#ifndef ROSE_USE_ROCC
#define ROSE_USE_ROCC 0
#endif

#if ROSE_USE_ROCC
#include "estimator_rocc.hpp"
#elif ROSE_USE_FP16
#include "estimator_fp16.hpp"
#endif

/* File-scope (not function-local) static: avoids __cxa_guard_* which the minimal libcpp
 * config (CONFIG_REQUIRES_FULL_LIBCPP=n) does not provide. Precedence:
 * -DROSE_USE_ROCC=1 -> the FcRoCC (custom-0, Q24.24 integer) estimator, no FPU/no Saturn V;
 * -DROSE_USE_FP16=1 -> the fp16(Zvfh)+int-accum estimator on the Saturn vector unit;
 * else ROSE_USE_EKF / complementary. */
#if ROSE_USE_ROCC
static RoccEstimator g_estimator;
#elif ROSE_USE_FP16
static Fp16Estimator g_estimator;
#elif ROSE_USE_EKF
static EkfEstimator g_estimator;
#else
static ComplementaryEstimator g_estimator;
#endif

IStateEstimator &active_estimator()
{
	return g_estimator;
}
