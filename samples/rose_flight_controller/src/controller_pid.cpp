/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hierarchical PID controller, ported from the original riskybird FreeRTOS firmware
 * (CobbledSteel/riskybird-firmware, main/i2c_simple_main.cpp). Gains, mixer, and the
 * force->duty curve are reproduced verbatim; only the interface (state vector in, u_out in the
 * shared [-0.583, 0.417] convention out) is adapted so it slots behind IController and reuses
 * send_control()'s clamp/cap/timeout unchanged.
 *
 * State/setpoint layout (matches TinyMPC / the estimator):
 *   [x, y, z, r1, r2, r3, vx, vy, vz, wx, wy, wz]  (Rodrigues attitude; small-angle r1/r2/r3
 *   ~ roll/pitch/yaw, w* ~ body rates). Horizontal position (x,y) is not regulated -- like the
 *   original firmware (and TinyMPC here) we regulate horizontal VELOCITY to the setpoint.
 */
#include "controller_pid.hpp"

#include <math.h>

/* --- gains (verbatim from the original firmware) --- */
static const float gravity            = 9.81f;

/* altitude loop */
static const float natFreq_height     = 2.0f;
static const float dampingRatio_height = 0.7f;

/* horizontal velocity loop */
static const float timeConst_horizVel = 0.5f;

/* attitude (angle) loop */
static const float tau_roll           = 0.10f;
static const float tau_pitch          = 0.10f;   /* = tau_roll */
static const float tau_yaw            = 0.25f;

/* body-rate loop */
static const float tau_rollRate       = 0.025f;
static const float tau_pitchRate      = 0.025f;  /* = tau_rollRate */
static const float tau_yawRate        = 0.05f;

/* rigid-body params used by the mixer (as used in the original firmware's mixing block) */
/* Vehicle mass (kg) -- the altitude-loop thrust feedforward. The PD altitude loop has no
 * integrator, so this MUST match the real weight or it can't reach hover thrust. Default is the
 * Crazyflie (32 g); riskybird v3 is heavier -> override with -DPID_MASS_KG=<kg>. */
#ifndef PID_MASS_KG
#define PID_MASS_KG 0.032f
#endif
static const float MASS               = PID_MASS_KG;        /* kg */
static const float J[3]               = {16e-6f, 16e-6f, 29e-6f}; /* diag inertia (kg*m^2) */

/* mixer geometry: arm length l, drag/thrust ratio k */
static const float l_arm              = 33e-3f;
static const float k_drag             = 0.01f;

/* --- force (Newtons) -> normalized motor duty [0,1] ---------------------------------------------
 * Replaces the old riskybird hand-measured chain (propConstant F=k*w^2 + a linear w->pwm fit,
 * which had hover landing ~63% duty). We now use Bitcraze's PUBLISHED thrust stand data for the
 * stock 7x16 coreless motor, which is the motor/prop this board uses:
 *
 *   thrust[g] = A*duty^2 + B*duty      (duty in [0,1]; A,B least-squares fit to their 2015
 *                                       45-35 table, RMS 0.38 g, max 0.82 g over 0..94%)
 *   https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/pwm-to-thrust/
 *
 * The 47-17 prop (what we spin) delivers ~15% more thrust than the 45-35 at the same drive
 * (Bitcraze published only this relative figure, no full 47-17 polynomial), so we scale the
 * curve by PROP_4717_GAIN. forceToVoltage() inverts thrust(duty) via the quadratic formula.
 *
 * IMPORTANT (fixed 2026-08-06): the Bitcraze table is TOTAL thrust of all 4 motors (whole
 * Crazyflie on a scale), max ~58 g total at 94% -- NOT per-motor. But the mixer feeds
 * forceToVoltage a PER-MOTOR force. So multiply the per-motor target by MOTORS(=4) before
 * inverting the total curve. Without this the curve over-predicts thrust ~4x and the controller
 * commands ~1/4 the duty needed -> won't take off. (Sanity: a stock 27 g Crazyflie now hovers at
 * ~48% duty, matching reality; the old firmware's Forster propConstant 2e-8 was per-motor and
 * gave ~58%.) A bench thrust-stand cal for THIS airframe would still be more accurate. */
static const float THRUST_A        = 26.919633f;   /* g per duty^2 (45-35 TOTAL fit) */
static const float THRUST_B        = 35.754861f;   /* g per duty   (45-35 TOTAL fit) */
static const float PROP_4717_GAIN  = 1.15f;        /* 47-17 vs 45-35 thrust ratio */
static const float GRAMS_PER_NEWTON = 101.9368f;   /* 1 / 9.80665e-3 */
static const float MOTORS           = 4.0f;        /* curve is total-thrust -> scale per-motor x4 */

static float forceToVoltage(float forceNewtons)
{
	if (forceNewtons <= 0.0f) {
		return 0.0f;
	}
	/* per-motor force (g), scaled x4 to the TOTAL curve and de-scaled to the 45-35 base */
	float t = (forceNewtons * GRAMS_PER_NEWTON * MOTORS) / PROP_4717_GAIN;
	/* solve THRUST_A*duty^2 + THRUST_B*duty - t = 0 for duty >= 0 */
	float disc = THRUST_B * THRUST_B + 4.0f * THRUST_A * t;
	if (disc < 0.0f) {
		disc = 0.0f;
	}
	float duty = (-THRUST_B + sqrtf(disc)) / (2.0f * THRUST_A);
	if (duty < 0.0f) {
		duty = 0.0f;
	} else if (duty > 1.0f) {
		duty = 1.0f;
	}
	return duty;
}

void HierarchicalPidController::compute(const float state[CTRL_NSTATES],
				       const float setpoint[CTRL_NSTATES],
				       float u_out[CTRL_NACTIONS], float dt)
{
	(void)dt;   /* pure P/PD cascade -- no integrator state, so no dt dependence */

	/* unpack state (our layout) */
	const float estRoll   = state[3];
	const float estPitch  = state[4];
	const float estYaw    = state[5];
	const float estVel_1  = state[6];   /* vx */
	const float estVel_2  = state[7];   /* vy */
	const float estVel_3  = state[8];   /* vz */
	const float gyroX     = state[9];
	const float gyroY     = state[10];
	const float gyroZ     = state[11];
	const float estHeight = state[2];

	/* setpoints (original firmware regulated vel->0, height->desHeight, yaw->0; we take those
	 * from the setpoint vector so hover reduces to the original behaviour). */
	const float desHeight = setpoint[2];
	const float desVel_1  = setpoint[6];
	const float desVel_2  = setpoint[7];
	const float yaw_tgt   = setpoint[5];

	/* (1) altitude loop -> normalized vertical acceleration command */
	const float desAcc3 = -2.0f * dampingRatio_height * natFreq_height * estVel_3
			      - natFreq_height * natFreq_height * (estHeight - desHeight);
	const float desNormalizedAcceleration =
		(gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));

	/* (2) horizontal velocity loop -> desired accelerations -> desired tilt */
	const float desAcc1 = -(1.0f / timeConst_horizVel) * (estVel_1 - desVel_1);
	const float desAcc2 = -(1.0f / timeConst_horizVel) * (estVel_2 - desVel_2);
	const float desRoll  = -desAcc2 / gravity;
	const float desPitch =  desAcc1 / gravity;

	/* (3) attitude loop -> desired body rates */
	const float roll_tgt  = desRoll;
	const float pitch_tgt = desPitch;
	const float rollRate_tgt  = (-1.0f / tau_roll)  * (estRoll  - roll_tgt);
	const float pitchRate_tgt = (-1.0f / tau_pitch) * (estPitch - pitch_tgt);
	const float yawRate_tgt   = (-1.0f / tau_yaw)   * (estYaw   - yaw_tgt);

	/* (4) body-rate loop -> angular-acceleration commands */
	const float rollRate_cmd  = (-1.0f / tau_rollRate)  * (gyroX - rollRate_tgt);
	const float pitchRate_cmd = (-1.0f / tau_pitchRate) * (gyroY - pitchRate_tgt);
	const float yawRate_cmd   = (-1.0f / tau_yawRate)   * (gyroZ - yawRate_tgt);

	/* (5) mixing: [thrust, moments] -> per-motor force -> duty */
	float u[4] = {desNormalizedAcceleration, rollRate_cmd, pitchRate_cmd, yawRate_cmd};
	u[0] = u[0] * MASS;                 /* normalized accel -> force */
	for (int i = 0; i < 3; i++) {
		u[i + 1] = u[i + 1] * J[i];    /* angular accel -> moment */
	}

	const float M[4][4] = {
		{0.25f,  0.25f / l_arm, -0.25f / l_arm,  0.25f / k_drag},
		{0.25f, -0.25f / l_arm, -0.25f / l_arm, -0.25f / k_drag},
		{0.25f, -0.25f / l_arm,  0.25f / l_arm,  0.25f / k_drag},
		{0.25f,  0.25f / l_arm,  0.25f / l_arm, -0.25f / k_drag},
	};
	float ctrl[4] = {0, 0, 0, 0};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ctrl[i] += M[i][j] * u[j];
		}
	}

	/* Per-motor force -> duty [0,1]. The ctrl[] permutation + trim factors are the original
	 * firmware's motor MAPPING, which carries over verbatim: riskybird v3 copies the old
	 * schematic's motor wiring, so motor<->corner assignment is unchanged. (The absolute
	 * force->duty scale in forceToVoltage below is a separate concern -- see note there.) */
	float duty[4];
	duty[0] = forceToVoltage(0.9f * ctrl[1]);
	duty[1] = forceToVoltage(0.9f * ctrl[2]);
	duty[2] = forceToVoltage(0.9f * ctrl[3] * 0.87f);
	duty[3] = forceToVoltage(0.9f * ctrl[0] * 0.87f);

	/* Emit in the shared normalized-thrust convention (send_control adds 0.583 back to recover
	 * duty, then clamps + applies the bench MOTOR_MAX_DUTY cap / timeout). */
	for (int i = 0; i < CTRL_NACTIONS; i++) {
		u_out[i] = duty[i] - 0.583f;
	}
}
