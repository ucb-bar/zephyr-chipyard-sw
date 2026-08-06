/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Build-time selection of the active controller. Default is TinyMPC (constrained MPC); build with
 * -DROSE_USE_PID=1 to use the hierarchical PID cascade instead. Adding a new control law is just
 * a new IController subclass plus a branch here (mirrors estimator_factory.cpp).
 */

#include "controller.hpp"
#include "controller_tinympc.hpp"
#include "controller_pid.hpp"

#ifndef ROSE_USE_PID
#define ROSE_USE_PID 0
#endif

/* File-scope (not function-local) static: avoids __cxa_guard_* which the minimal libcpp
 * config (CONFIG_REQUIRES_FULL_LIBCPP=n) does not provide. */
#if ROSE_USE_PID
static HierarchicalPidController g_controller;
#else
static TinympcController g_controller;
#endif

IController &active_controller()
{
	return g_controller;
}
