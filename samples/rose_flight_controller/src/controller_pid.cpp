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
#include <stdint.h>

/* --- gains (verbatim from the original firmware) --- */
static const float gravity            = 9.81f;

/* altitude loop */
static const float natFreq_height     = 2.0f;
static const float dampingRatio_height = 0.7f;
/* Altitude INTEGRAL term (makes the height loop a PID). The gravity feed-forward goes through an
 * uncalibrated force->duty curve, so a pure P/D loop droops (FF too low) or overshoots (FF too high),
 * and battery sag drifts the hover point during a flight -- no single PID_MASS_KG fits. The integrator
 * auto-cancels that steady bias (it winds up to whatever thrust hovers *now*), so we stop hand-tuning
 * mass. Reset on the ground (no pre-takeoff windup) and hard-clamped (bounded authority -> can't wind
 * up into a climb-away). Set KI_HEIGHT=0 to disable and fall back to the pure P/D loop. */
#ifndef KI_HEIGHT
#define KI_HEIGHT 1.5f            /* integral gain (m/s^2 per m*s) */
#endif
#ifndef ALT_INT_MAX
#define ALT_INT_MAX 3.0f          /* anti-windup clamp: max |integral| accel contribution (m/s^2) */
#endif
#ifndef ALT_INT_GROUND_M
#define ALT_INT_GROUND_M 0.05f    /* est height below this = on the ground -> hold integrator at 0 */
#endif
/* Horizontal velocity INTEGRAL (makes the velocity loop a PI). The proportional loop always leaves a
 * steady-state error against a constant disturbance -- a mount/CG tilt makes the drone hover banked and
 * drift at a fixed rate that P can't null. The I term winds up to cancel it (auto roll/pitch trim).
 * Only worth running because the gyro-cal'd flow now reports that drift reliably. Ground-reset (no
 * pre-takeoff windup) + clamped (bounded extra bank). KI_VEL=0 disables -> pure P velocity loop. */
#ifndef KI_VEL
#define KI_VEL      0.8f          /* velocity integral gain (m/s^2 per (m/s)*s) */
#endif
#ifndef VEL_INT_MAX
#define VEL_INT_MAX 2.0f          /* anti-windup clamp: max |vel integral| accel (m/s^2 ~ 12 deg bank) */
#endif

/* horizontal velocity loop. Gain = 1/VEL_TC: smaller VEL_TC -> stronger velocity correction (bigger
 * tilt per m/s of velocity error). Cranked up it makes a flow sign/frame error unmistakable (a wrong
 * sign flies away HARD instead of ambiguously drifting; a right sign holds position visibly tighter). */
#ifndef VEL_TC
#define VEL_TC 0.5f
#endif
static const float timeConst_horizVel = VEL_TC;
/* Ceiling on the velocity-loop tilt command (rad). The loop turns velocity error into a desired
 * bank; a noisy/large velocity spike * a hot gain can demand an absurd tilt (the gain-5 limit cycle
 * commanded ~44 deg), which the attitude loop chases -> violent thrash. Cap it so the velocity loop
 * can never ask for more than a sane bank; residual authority is bounded, oscillation can't build. */
#ifndef VEL_TILT_MAX
#define VEL_TILT_MAX 0.26f   /* ~15 deg */
#endif
/* Slew-rate limit on the velocity-loop tilt command (rad/s). Caps how FAST the loop can change the
 * commanded bank, which caps the body roll/pitch RATE it induces -- and body rate is exactly what
 * leaks through the (imperfect) flow gyro-compensation into vy, driving the roll<->flow oscillation.
 * "Gentler corrections" in the literal rate sense: a vy spike now ramps the bank instead of snapping
 * it, so the drone never rotates fast enough to corrupt its own flow. 0 or large = no slew limit. */
#ifndef TILT_SLEW_RADPS
#define TILT_SLEW_RADPS 1.0f
#endif

/* attitude (angle) loop -- larger tau = softer/slower (override via -DTAU_ROLL=<s>) */
#ifndef TAU_ROLL
#define TAU_ROLL 0.10f
#endif
static const float tau_roll           = TAU_ROLL;
static const float tau_pitch          = TAU_ROLL;   /* = tau_roll */
static const float tau_yaw            = 0.25f;

/* body-rate (inner) loop -- larger tau = softer, less oscillation (override via -DTAU_ROLLRATE=<s>) */
#ifndef TAU_ROLLRATE
#define TAU_ROLLRATE 0.025f
#endif
static const float tau_rollRate       = TAU_ROLLRATE;
static const float tau_pitchRate      = TAU_ROLLRATE;  /* = tau_rollRate */
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
 * This airframe (riskybird v3) runs the Bitcraze THRUST-UPGRADE combo: 7x20 mm motors + HQ
 * Ultralight 51MMX2 props. Bitcraze rates that at ~+5 g/motor (~+20 g TOTAL) over the stock
 * 7x16 + 45-35, i.e. ~1.33x the 45-35 total-thrust curve (no full polynomial published), so we
 * scale by PROP_GAIN. APPROXIMATE (modeled as a uniform gain from the published max-thrust
 * delta) -- a bench thrust-stand cal for THIS airframe is still more accurate. (Was 1.15 =
 * 47-17 vs 45-35 on the OG drone.) forceToVoltage() inverts thrust(duty) via the quadratic.
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
static const float PROP_GAIN       = 1.33f;        /* 7x20 + 51MMX2 vs 45-35 (see note above) */
static const float GRAMS_PER_NEWTON = 101.9368f;   /* 1 / 9.80665e-3 */
static const float MOTORS           = 4.0f;        /* curve is total-thrust -> scale per-motor x4 */

static float forceToVoltage(float forceNewtons)
{
	if (forceNewtons <= 0.0f) {
		return 0.0f;
	}
	/* per-motor force (g), scaled x4 to the TOTAL curve and de-scaled to the 45-35 base */
	float t = (forceNewtons * GRAMS_PER_NEWTON * MOTORS) / PROP_GAIN;
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

/* --- side-ToF wall repulsion ("bumper") -------------------------------------------------------
 * When the ROSE_BUMPER build is active, main.cpp feeds the latest side-wall distances via
 * pid_set_walls() each control tick. compute() turns any wall closer than WALL_REPULSE_MM into a
 * bounded velocity command AWAY from it, ADDED to the horizontal-velocity setpoint -- so the
 * existing vel->tilt cascade produces a lean away from the wall. Authority is intentionally small
 * for first flights (WALL_MAX_REP_VEL). Signs are per-axis flippable (WALL_SIGN_X/Y) so a facing
 * that turns out mapped backwards on the bench can be corrected with a rebuild, not a rewire. */
#ifndef WALL_REPULSE_MM
#define WALL_REPULSE_MM   350      /* start pushing when a wall is closer than this (mm) */
#endif
#ifndef WALL_MIN_MM
#define WALL_MIN_MM       90       /* full-authority distance (mm); anything closer clamps to full */
#endif
#ifndef WALL_MAX_REP_VEL
#define WALL_MAX_REP_VEL  0.35f    /* max commanded repulsion velocity (m/s), per axis */
#endif
#ifndef WALL_SIGN_X
#define WALL_SIGN_X       1.0f     /* flip to -1.0f if front/back push the wrong way */
#endif
#ifndef WALL_SIGN_Y
#define WALL_SIGN_Y       1.0f     /* flip to -1.0f if left/right push the wrong way */
#endif

/* Attitude trim (rad) added to the desired roll/pitch, to cancel the deterministic hover drift that
 * the controller CANNOT observe: with no optical flow there is no horizontal-velocity feedback, so a
 * small attitude/thrust bias makes the drone hover tilted and drift at a fixed rate. +ROLL_TRIM_RAD
 * banks right (-y) to cancel a LEFT (+y) drift; +PITCH_TRIM_RAD noses forward (+x). Flip the sign if
 * the drift grows. Tune per airframe via -DROLL_TRIM_RAD=<rad>. */
#ifndef ROLL_TRIM_RAD
#define ROLL_TRIM_RAD 0.0f
#endif
#ifndef PITCH_TRIM_RAD
#define PITCH_TRIM_RAD 0.0f
#endif
/* Horizontal velocity loop: 1 = regulate velocity -> tilt (needs a REAL velocity, i.e. optical flow);
 * 0 = DISABLE it (desRoll=desPitch=0) -> hold LEVEL attitude + altitude only, no velocity/position
 * hold. Pure dead-reckoning velocity is physically blind to bank translation and drifts, so closing
 * the loop on it chases a phantom and runs the drone away. ROSE_VEL_LOOP=0 is the stable-drift
 * baseline: it drifts horizontally but stays oriented. */
#ifndef ROSE_VEL_LOOP
#define ROSE_VEL_LOOP 1
#endif
/* POSITION-HOLD outer loop (opt-in, OFF by default). Wraps the velocity loop: latch a horizontal
 * position reference at takeoff, then command a velocity SETPOINT back toward it -- turning "hold zero
 * velocity" into "return to a spot", which cuts the residual dead-reckoning drift the velocity-only loop
 * leaves (~1 m/s wander). Enable with -DROSE_POS_LOOP=1. See the block in compute() for the full
 * rationale + the dead-reckoning-drift caveat. KP_POS is the position->velocity gain (1/s) and
 * POS_VEL_MAX clamps the commanded return speed (m/s). */
#ifndef ROSE_POS_LOOP
#define ROSE_POS_LOOP 0
#endif
#ifndef KP_POS
#define KP_POS 1.0f          /* position->velocity gain (1/s): desVel = -KP_POS*(pos - pos_ref) */
#endif
#ifndef POS_VEL_MAX
#define POS_VEL_MAX 0.3f     /* clamp on the position-hold velocity command magnitude (m/s), per axis */
#endif
/* Attitude-loop authority scale on the roll+pitch moments. 1.0 = original gains; <1 = gentler / more
 * phase margin. The bare-drone gains went marginally unstable once the cage changed the mass/inertia
 * (roll oscillation grows to a tumble), so scale them down to buy back stability. Tune -DATT_GAIN=<f>. */
#ifndef ATT_GAIN
#define ATT_GAIN 1.0f
#endif

struct pid_walls { int16_t front, back, left, right; bool valid; };
static volatile struct pid_walls g_walls = {0, 0, 0, 0, false};

/* Called from main.cpp each control tick (only in ROSE_BUMPER builds). <=0 mm = no wall/no target. */
extern "C" void pid_set_walls(int16_t front_mm, int16_t back_mm,
			      int16_t left_mm, int16_t right_mm, bool valid)
{
	g_walls.front = front_mm;
	g_walls.back  = back_mm;
	g_walls.left  = left_mm;
	g_walls.right = right_mm;
	g_walls.valid = valid;
}

/* distance (mm) -> repulsion speed magnitude (m/s); linear ramp over [WALL_MIN_MM, WALL_REPULSE_MM] */
static float wall_push(int16_t d_mm)
{
	if (d_mm <= 0 || d_mm >= WALL_REPULSE_MM) {
		return 0.0f;   /* no valid target, or wall beyond the influence radius */
	}
	float d = (d_mm < WALL_MIN_MM) ? (float)WALL_MIN_MM : (float)d_mm;
	float frac = ((float)WALL_REPULSE_MM - d) / (float)(WALL_REPULSE_MM - WALL_MIN_MM);
	return frac * WALL_MAX_REP_VEL;   /* 0..WALL_MAX_REP_VEL */
}

void HierarchicalPidController::compute(const float state[CTRL_NSTATES],
				       const float setpoint[CTRL_NSTATES],
				       float u_out[CTRL_NACTIONS], float dt)
{
	/* dt now matters: the altitude loop carries an integral term (below). */

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
	float desVel_1        = setpoint[6];
	float desVel_2        = setpoint[7];
	const float yaw_tgt   = setpoint[5];

	/* Wall repulsion: bias the horizontal-velocity setpoint away from any close wall. Body frame
	 * is FLU (+x fwd, +y left): a front (+x) wall pushes -x, back (-x) pushes +x; a left (+y) wall
	 * pushes -y, right (-y) pushes +y. Net per axis, so opposing walls partially cancel (centering). */
	if (g_walls.valid) {
		desVel_1 += WALL_SIGN_X * (wall_push(g_walls.back)  - wall_push(g_walls.front));
		desVel_2 += WALL_SIGN_Y * (wall_push(g_walls.right) - wall_push(g_walls.left));
	}

	/* (1b) POSITION-HOLD outer loop (opt-in) -> biases the horizontal-velocity setpoint. Latch a
	 * position reference at takeoff and command a velocity back toward it:
	 *   desVel += clamp(-KP_POS * (pos - pos_ref), +/-POS_VEL_MAX)
	 * fed into the velocity loop below via the same "bias desVel" idiom as the wall bumper (setpoint[6/7]
	 * are 0 in hover, so this is the horizontal setpoint). Ground-reset: while grounded (estHeight <
	 * ALT_INT_GROUND_M) keep pos_ref = current pos, so there is NO setpoint jump at takeoff -- mirroring
	 * the altitude/velocity integrator ground-reset above.
	 *
	 * KNOWN LIMITATION: pos (state[0]/state[1]) is DEAD-RECKONED by integrating the fused/flow velocity;
	 * there is NO absolute position reference, so pos_ref and the estimate drift together and slowly. This
	 * loop REDUCES but CANNOT eliminate long-term drift -- it is exactly the "dead-reckoned position
	 * control" intended, not a GPS/anchored hold. */
#if ROSE_POS_LOOP
	const float estPos_1 = state[0];   /* x (fwd) */
	const float estPos_2 = state[1];   /* y (left) */
	if (estHeight < ALT_INT_GROUND_M) {
		pos_ref_1 = estPos_1; pos_ref_2 = estPos_2;   /* on the ground: track pos -> no jump at takeoff */
	} else {
		float posVel_1 = -KP_POS * (estPos_1 - pos_ref_1);
		float posVel_2 = -KP_POS * (estPos_2 - pos_ref_2);
		if (posVel_1 >  POS_VEL_MAX) { posVel_1 =  POS_VEL_MAX; } else if (posVel_1 < -POS_VEL_MAX) { posVel_1 = -POS_VEL_MAX; }
		if (posVel_2 >  POS_VEL_MAX) { posVel_2 =  POS_VEL_MAX; } else if (posVel_2 < -POS_VEL_MAX) { posVel_2 = -POS_VEL_MAX; }
		desVel_1 += posVel_1;
		desVel_2 += posVel_2;
	}
#endif

	/* (1) altitude loop -> normalized vertical acceleration command (P/D + integral trim). alt_int is a
	 * member (reset in init()) so a soft-RESET clears the auto-hover-thrust between flights. */
	if (estHeight < ALT_INT_GROUND_M) {
		alt_int = 0.0f;        /* on the ground: no windup before takeoff */
	} else {
		alt_int += KI_HEIGHT * (desHeight - estHeight) * dt;
		if (alt_int >  ALT_INT_MAX) { alt_int =  ALT_INT_MAX; }
		else if (alt_int < -ALT_INT_MAX) { alt_int = -ALT_INT_MAX; }
	}
	const float desAcc3 = -2.0f * dampingRatio_height * natFreq_height * estVel_3
			      - natFreq_height * natFreq_height * (estHeight - desHeight)
			      + alt_int;
	const float desNormalizedAcceleration =
		(gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));

	/* (2) horizontal velocity loop -> desired accelerations -> desired tilt (ROSE_VEL_LOOP=0 holds
	 * LEVEL instead: no velocity/position hold, so it can't chase a bad dead-reckoned velocity). */
#if ROSE_VEL_LOOP
	/* PI velocity loop: P for transients, I to cancel the constant drift P leaves as steady-state
	 * error (auto-trims the mount/CG tilt). (1/VEL_TC)*err == the old -(1/VEL_TC)*(est-des). */
	const float verr1 = desVel_1 - estVel_1;
	const float verr2 = desVel_2 - estVel_2;
	if (estHeight < ALT_INT_GROUND_M) {
		vel_int_1 = 0.0f; vel_int_2 = 0.0f;   /* on the ground: no windup before takeoff */
	} else {
		vel_int_1 += KI_VEL * verr1 * dt;
		vel_int_2 += KI_VEL * verr2 * dt;
		if (vel_int_1 >  VEL_INT_MAX) { vel_int_1 =  VEL_INT_MAX; } else if (vel_int_1 < -VEL_INT_MAX) { vel_int_1 = -VEL_INT_MAX; }
		if (vel_int_2 >  VEL_INT_MAX) { vel_int_2 =  VEL_INT_MAX; } else if (vel_int_2 < -VEL_INT_MAX) { vel_int_2 = -VEL_INT_MAX; }
	}
	const float desAcc1 = (1.0f / timeConst_horizVel) * verr1 + vel_int_1;
	const float desAcc2 = (1.0f / timeConst_horizVel) * verr2 + vel_int_2;
	float desRoll  = -desAcc2 / gravity;
	float desPitch =  desAcc1 / gravity;
	/* Cap the commanded bank so a velocity spike can't demand a violent tilt (see VEL_TILT_MAX). */
	if (desRoll  >  VEL_TILT_MAX) { desRoll  =  VEL_TILT_MAX; } else if (desRoll  < -VEL_TILT_MAX) { desRoll  = -VEL_TILT_MAX; }
	if (desPitch >  VEL_TILT_MAX) { desPitch =  VEL_TILT_MAX; } else if (desPitch < -VEL_TILT_MAX) { desPitch = -VEL_TILT_MAX; }
	/* Slew-limit the tilt command: cap its rate of change so the induced body rate stays low (keeps
	 * rotation from corrupting the flow -> breaks the roll<->flow oscillation). See TILT_SLEW_RADPS. */
	if (TILT_SLEW_RADPS > 0.0f) {
		const float dmax = TILT_SLEW_RADPS * dt;
		if (estHeight < ALT_INT_GROUND_M) { desRoll_prev = 0.0f; desPitch_prev = 0.0f; }  /* reset on ground */
		if (desRoll  > desRoll_prev  + dmax) { desRoll  = desRoll_prev  + dmax; }
		else if (desRoll  < desRoll_prev  - dmax) { desRoll  = desRoll_prev  - dmax; }
		if (desPitch > desPitch_prev + dmax) { desPitch = desPitch_prev + dmax; }
		else if (desPitch < desPitch_prev - dmax) { desPitch = desPitch_prev - dmax; }
		desRoll_prev = desRoll; desPitch_prev = desPitch;
	}
#else
	const float desRoll  = 0.0f;
	const float desPitch = 0.0f;
	(void)estVel_1; (void)estVel_2; (void)desVel_1; (void)desVel_2;
#endif

	/* (3) attitude loop -> desired body rates (+ static trim to cancel unobservable hover drift) */
	const float roll_tgt  = desRoll  + ROLL_TRIM_RAD;
	const float pitch_tgt = desPitch + PITCH_TRIM_RAD;
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
	u[1] = u[1] * J[0] * ATT_GAIN;      /* roll moment  (ATT_GAIN = attitude authority scale) */
	u[2] = u[2] * J[1] * ATT_GAIN;      /* pitch moment */
	u[3] = u[3] * J[2];                 /* yaw moment (unscaled) */

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
