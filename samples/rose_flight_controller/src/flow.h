/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * PMW3901 optical-flow front-end for the flight controller. Reads the sensor's motion burst on a
 * dedicated background thread (blocking SPI off the control loop, like side_tof / the down-ToF),
 * applies the validated body-frame remap, gates on SQUAL, and caches the latest body-frame ANGULAR
 * flow rate (rad/s). The control loop reads the cache non-blocking via flow_get() and multiplies by
 * the ToF height to get body-frame velocity (m/s) for the estimator's flow update.
 *
 * Axis remap (validated on HW via tools/live_flow_plot.py): the sensor is mounted so
 *   drone +x (forward) = -deltaX,   drone +y (left) = +deltaY.
 */
#ifndef ROSE_FLOW_H
#define ROSE_FLOW_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the PMW3901 (SPI2, software CS) and start the background reader thread.
 * Returns 0 if the sensor answered (chip ID ok), <0 otherwise (flow then stays invalid). */
int flow_init(void);

/* Non-blocking snapshot of the latest flow sample.
 *   ang_x, ang_y : body-frame optical-flow ANGULAR rate (rad/s), remapped to the drone frame.
 *                  Body velocity (m/s) = ang * height_m.  (+x fwd, +y left)
 *   squal        : surface quality of the sample (0..~128; higher = more trustworthy)
 *   valid        : true iff the sample is fresh AND squal >= the floor (caller should also
 *                  require a valid ToF height before using it as velocity).
 */
void flow_get(float *ang_x, float *ang_y, int *squal, bool *valid);

#ifdef __cplusplus
}
#endif

#endif /* ROSE_FLOW_H */
