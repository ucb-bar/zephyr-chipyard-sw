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

/* File-scope (not function-local) static: avoids __cxa_guard_* which the minimal libcpp
 * config (CONFIG_REQUIRES_FULL_LIBCPP=n) does not provide. */
#if ROSE_USE_EKF
static EkfEstimator g_estimator;
#else
static ComplementaryEstimator g_estimator;
#endif

IStateEstimator &active_estimator()
{
	return g_estimator;
}
