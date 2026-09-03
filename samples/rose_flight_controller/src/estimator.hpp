/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pluggable state-estimator interface for the RoSE flight controller.
 *
 * All estimators consume the same sensor set (6-axis IMU + optical-flow velocity +
 * downward ToF height) and produce the 12-DoF state TinyMPC expects:
 *   [x, y, z, r1, r2, r3, vx, vy, vz, wx, wy, wz]   (Rodrigues attitude).
 *
 * Concrete filters implement IStateEstimator; the active one is selected at build time
 * (see estimator_factory.cpp, -DROSE_USE_EKF). This keeps estimators MODULAR: new filters
 * drop in behind this interface without touching main.cpp. Current implementations:
 *   - ComplementaryEstimator (estimator_complementary.*): quaternion Mahony attitude +
 *     fixed-gain dead-reckoning translation.
 *   - EkfEstimator (estimator_ekf.*): quaternion Mahony attitude + per-axis Kalman
 *     filters for translation (flow -> velocity, ToF -> altitude); default.
 */

#ifndef ROSE_ESTIMATOR_HPP
#define ROSE_ESTIMATOR_HPP

#define EST_NSTATES 12

struct IStateEstimator {
	/* Initialize at a known takeoff pose (level, at rest). */
	virtual void init(float x0, float y0, float z0) = 0;

	/* One estimator step. The IMU + optical flow are sampled every call (fast); the ToF
	 * height is LOW-RATE (~20-40 ms) so it is only fused when a fresh sample is available.
	 *   accel:      body-frame specific force (m/s^2)   [ax, ay, az]
	 *   gyro:       body-frame angular rate   (rad/s)   [gx, gy, gz]
	 *   flow:       body-frame horizontal velocity (m/s)[vx, vy]; used only if flow_valid
	 *   flow_valid: false on a flow dropout (stale/no sample) -> run velocity predict-only
	 *   height:     downward ToF SLANT range (m); used only if tof_valid (tilt-corrected inside)
	 *   tof_valid:  true on steps where a fresh ToF sample arrived
	 *   baro_rel:   barometer altitude relative to the arm reference (m); used only if baro_valid.
	 *               OPTIONAL (ROSE_BARO): a smooth, tilt-immune altitude that fills ToF dropouts
	 *               (large tilt -> beam off-vertical) and coasts through ToF step-jumps (obstacle/
	 *               desk edge). Pass 0/false to ignore it -> the filter is unchanged (backward compat).
	 *   baro_valid: true when a fresh/valid barometer sample is available
	 *   dt:         control period (s)                                                  */
	virtual void update(const float accel[3], const float gyro[3], const float flow[2],
			    bool flow_valid, float height, bool tof_valid,
			    float baro_rel, bool baro_valid, float dt) = 0;

	/* Optional: fuse horizontal wall distances (4x multizone ToF, nearest-wall per direction)
	 * for obstacle-relative position. In a corridor, (d_right - d_left)/2 is an exact lateral
	 * position relative to the corridor center, and (d_back - d_front)/2 the longitudinal one --
	 * position observability the flow-only filter lacks (flow gives velocity; position drifts).
	 * Each pair is fused only when BOTH facing walls are within range (rmax). Distances are the
	 * nearest wall in metres per body direction; rmax is the sensor's max range. Default no-op
	 * (filters that don't use walls, or builds without the sensors, ignore it).            */
	virtual void fuse_walls(float d_front, float d_back, float d_left, float d_right,
				float rmax) { (void)d_front; (void)d_back; (void)d_left;
				(void)d_right; (void)rmax; }

	/* Fill the 12-DoF TinyMPC state from the current estimate. */
	virtual void get_state(float state[EST_NSTATES]) const = 0;

	/* Human-readable filter name (for the boot log). */
	virtual const char *name() const = 0;

	virtual ~IStateEstimator() {}
};

/* The build-time-selected estimator singleton (-DROSE_USE_EKF=1 -> EKF, else
 * complementary). Defined in estimator_factory.cpp. */
IStateEstimator &active_estimator();

#endif /* ROSE_ESTIMATOR_HPP */
