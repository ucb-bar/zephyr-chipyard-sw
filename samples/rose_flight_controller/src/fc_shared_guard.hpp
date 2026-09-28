/* SPDX-License-Identifier: Apache-2.0
 * Guard algorithm extracted from main.cpp without changing threshold order or
 * persistence. Caller supplies uptime and battery voltage. The firmware keeps
 * this object across soft resets, as it did the original function-local statics.
 * SAFE_* and battery feature macros must be defined by the build/caller.
 */
#ifndef ROSE_FC_SHARED_GUARD_HPP
#define ROSE_FC_SHARED_GUARD_HPP
#include "fc_shared_math.h"
#include <stddef.h>
struct FcEnvelopeGuard {
    float g_guard_tilt_rad=0, g_guard_est_rad=0, g_guard_amag=0;
    uint32_t g_guard_acc_skips=0;
    float lp=9.80665f, yaw_lp=0, vel_lp[3]={};
    int64_t lp_prev=0, yaw_lp_t=0;
const char *accel_envelope(const float *accel, int64_t now_ms, int *need_ms, float *meas, const char **unit)
{
	const float ax = accel[0], ay = accel[1], az = accel[2];
	const float horiz2 = ax * ax + ay * ay;
	const float total2 = horiz2 + az * az;
	const float amag = sqrtf(total2);

	g_guard_amag = amag;

	/* Free fall first: it is the most urgent and the only one tilt can never see. A drone dropped
	 * flat stays flat the whole way down. Tested on a low-passed |a| when SAFE_FREEFALL_LP_TAU_MS
	 * is set, so motor vibration averages out (see SAFE_FREEFALL_DEBOUNCE_MS). */
	float ff_amag = amag;
#if SAFE_FREEFALL_LP_TAU_MS > 0
	{
		const int64_t now = now_ms;
		const float dt_ms = lp_prev ? (float)(now - lp_prev) : 0.0f;

		lp_prev = now;
		lp += (1.0f - expf(-dt_ms / (float)(SAFE_FREEFALL_LP_TAU_MS))) * (amag - lp);
		ff_amag = lp;
	}
#endif
	if (ff_amag < (SAFE_FREEFALL_MPS2)) {
		*need_ms = SAFE_FREEFALL_DEBOUNCE_MS;
		*meas = ff_amag; *unit = "m/s2 |a| (free-fall test)";
		return "free-fall";
	}
	if (total2 > (SAFE_IMPACT_MPS2) * (SAFE_IMPACT_MPS2)) {
		*need_ms = SAFE_SHOCK_DEBOUNCE_MS;
		*meas = amag; *unit = "m/s2 |a|";
		return "impact";
	}
	/* Direction is attitude only while the magnitude is plausibly gravity. Outside the band the
	 * two tests above own the case and this one stays quiet rather than guessing. */
	if (amag >= (SAFE_ACC_TRUST_LO_MPS2) && amag <= (SAFE_ACC_TRUST_HI_MPS2)) {
		/* tilt from vertical, straight out of gravity: sin(theta) = |horizontal| / |a|. */
		const float tilt = asinf(sqrtf(horiz2 / total2));

		g_guard_tilt_rad = tilt;
		if (tilt > (SAFE_ACC_TILT_RAD)) {
			*need_ms = SAFE_DEBOUNCE_MS;
			*meas = tilt * (180.0f / 3.14159265f); *unit = "deg accel-tilt";
			return "accel-tilt";
		}
	} else {
		/* Out of the trust band: publish "unknown, assume the worst" rather than leaving the
		 * last good value in place. The arm gate reads this, and a stale level reading is the
		 * one way a cross-check can silently permit what it was added to forbid. */
		g_guard_tilt_rad = 3.14159265f;
		g_guard_acc_skips++;
	}
	return NULL;
}
const char *evaluate(const float *state, const float *gyro, const float *accel,
				    int64_t now_ms, float battery_v, int *need_ms, float *meas, const char **unit)
{
	const char *why;
	float tilt;

	/* Filtered every call, before any early return, so the average never skips samples. */
	{
		const int64_t t = now_ms;
		const float dt_ms = yaw_lp_t ? (float)(t - yaw_lp_t) : 0.0f;
		const float a = (SAFE_YAW_LP_TAU_MS > 0) ? 1.0f - expf(-dt_ms / (float)(SAFE_YAW_LP_TAU_MS)) : 1.0f;

		yaw_lp += a * (gyro[2] - yaw_lp);
		yaw_lp_t = t;

		const float av = (SAFE_VEL_LP_TAU_MS > 0) ? 1.0f - expf(-dt_ms / (float)(SAFE_VEL_LP_TAU_MS)) : 1.0f;

		for (int i = 0; i < 3; i++) {
			vel_lp[i] += av * (state[6 + i] - vel_lp[i]);
		}
	}

	*need_ms = SAFE_DEBOUNCE_MS;
	*meas = 0.0f;
	*unit = "";

	why = accel_envelope(accel, now_ms, need_ms, meas, unit);
	if (why != NULL) {
		return why;
	}
	*need_ms = SAFE_DEBOUNCE_MS;

	tilt = fc_tilt_rad(state);
	g_guard_est_rad = tilt;
	if (tilt > SAFE_MAX_TILT_RAD) {
		*meas = tilt * (180.0f / 3.14159265f); *unit = "deg est-tilt";
		return "tilt";
	}
	if (fabsf(gyro[0]) > SAFE_MAX_RATE_RADPS || fabsf(gyro[1]) > SAFE_MAX_RATE_RADPS ||
	    fabsf(gyro[2]) > SAFE_MAX_RATE_RADPS) {
		float m = fabsf(gyro[0]);

		if (fabsf(gyro[1]) > m) m = fabsf(gyro[1]);
		if (fabsf(gyro[2]) > m) m = fabsf(gyro[2]);
		*meas = m; *unit = "rad/s";
		return "rate";
	}
	if (fabsf(yaw_lp) > SAFE_MAX_YAW_RATE_RADPS) {
		*need_ms = SAFE_YAW_DEBOUNCE_MS;
		*meas = fabsf(yaw_lp); *unit = "rad/s yaw (low-passed)";
		return "yaw-spin";
	}
	if (fabsf(vel_lp[0]) > SAFE_MAX_VEL_MPS || fabsf(vel_lp[1]) > SAFE_MAX_VEL_MPS ||
	    fabsf(vel_lp[2]) > SAFE_MAX_VEL_MPS) {
		float m = fabsf(vel_lp[0]);

		if (fabsf(vel_lp[1]) > m) m = fabsf(vel_lp[1]);
		if (fabsf(vel_lp[2]) > m) m = fabsf(vel_lp[2]);
		*meas = m; *unit = "m/s";
		return "velocity";
	}
	if (state[2] > SAFE_MAX_HEIGHT_M) {   /* z = altitude (up); one-sided ceiling guard */
		*meas = state[2]; *unit = "m";
		return "height";
	}
#if ROSE_BATT_SENSE && !ROSE_BATT_REPORT_ONLY
	/* Low-voltage cutoff: only a VALID reading (>= 1.0 V) below the threshold trips -- a garbage-low
	 * read (< 1.0 V, sensor fault) is ignored so it can't false-estop mid-flight. Debounced by the
	 * caller like every other reason.
	 *
	 * That guard is sufficient against a reading that is garbage-LOW and nothing else. It does NOT
	 * protect against a plausible wrong reading: a mis-scaled or wrong-channel result that lands in
	 * [1.0, 3.2) V trips this exactly as a flat pack would, and from a remote console the two are
	 * indistinguishable. That is why a mode whose scaling is unverified sets ROSE_BATT_REPORT_ONLY
	 * and compiles this test out entirely rather than relying on the >= 1.0 V floor. */
	if (battery_v >= 1.0f && battery_v < BATT_CUTOFF_V) {
		*meas = battery_v; *unit = "V";
		return "battery";
	}
#endif
	return NULL;
}
};
#endif
