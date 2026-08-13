/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE navigation controller — waypoint flight in a corridor using obstacle-relative
 * position from the 4x horizontal VL53L5CX multizone ToF (F/R/B/L).
 *
 * This extends the hover flight controller: same IMU + optical-flow + downward-ToF sensing,
 * same modular estimator + TinyMPC. The addition is the FOUR horizontal ToFs (read through the
 * reusable ucbbar,rose-tof-zone driver) fed to estimator.fuse_walls(): in a corridor the
 * difference of opposite wall distances is an EXACT lateral / longitudinal position relative to
 * the corridor center, which the flow-only filter lacks (flow gives velocity; x/y position
 * drifts). With x/y now observable, TinyMPC regulates POSITION -- so the controller tracks a
 * WAYPOINT (hold the corridor center, advance forward), not just hover.
 *
 * Threading (-DROSE_THREADED=1): a producer/consumer split into sensor-IO / estimator / control
 * / vision / keepalive threads (see the ROSE_THREADED block). Default (0) is the single loop.
 *
 * Same shared-app sim-to-real story: on real riskybird hardware the aliases bind to
 * st,vl53l5cx (4x, tested on the riskybird branch) instead of the RoSE virtual driver.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include <math.h>

#include "admm.hpp"
/* YAW-MIXING-CORRECTED model: the committed Bdyn yaw rows used signs (+,-,-,+) but the PHYSICAL
 * plant's propeller-drag yaw follows CRAZYFLIE spin (m1:+,m2:-,m3:+,m4:-)=(+,-,+,-); rotors m3,m4
 * were yaw-sign-flipped in the model, so the MPC's yaw command produced the wrong/near-zero net
 * yaw on the real plant (root cause of the on-SoC yaw failure). This regen flips Bdyn cols 2,3 on
 * the yaw rows to match the plant and recomputes the cache (host-validated: weave tracks 99%,
 * altitude held). Q[yaw-rate]=150. Drive yaw body-relative: err[5] = -yaw_rate * YAW_CMD_GAIN. */
#include "problem_data/quadrotor_yawfix_params.hpp"
#include "glob_opts.hpp"

#include "estimator.hpp"
#include <rose/rose_sensor.h>

#define NSTATES   12
#define NACTIONS  4

#define CTRL_DT      0.005f
/* Estimator init altitude + hover target. Override with -DSTART_Z=.. -DTARGET_Z=.. so the
 * on-SoC EKF can be initialized at the co-sim spawn height (avoids a huge ToF innovation
 * transient) and the controller can hold the warehouse cruise altitude (~2.0 m). */
#ifndef START_Z
#define START_Z      0.9f
#endif
#ifndef TARGET_Z
#define TARGET_Z     1.0f
#endif
/* One-time takeoff-heading calibration for the gyro-only Mahony yaw. The co-sim spawns the drone
 * at a nonzero yaw (seed 1000 ~= 1.727 rad); without seeding, est-yaw=0 disagrees with the true
 * heading, so the MPC (which tracks est-yaw) sees no yaw error and never steers. Set to the spawn
 * yaw. NOT continuous GT -- a single init seed, then the gyro carries it closed-loop. */
#ifndef START_YAW
#define START_YAW    0.0f
#endif
/* Body-relative yaw command gain: yaw-angle offset commanded per unit of policy yaw-rate.
 * err[5] = -yaw_rate * YAW_CMD_GAIN, re-zeroed each tick (bounded, drift-immune). */
#ifndef YAW_CMD_GAIN
#define YAW_CMD_GAIN 0.5f
#endif
/* Turn-before-go forward gating: fwd *= max(0, 1 - |yr|/YR_MAX)^2. YR_MAX = the policy yr scale
 * (rad/s) at which forward speed is fully cut (host-tuned winner: 0.8). */
#ifndef YR_MAX
#define YR_MAX 0.8f
#endif
/* Minimum fraction of commanded forward speed kept by turn-before-go (prevents a hover deadlock
 * when the real vision model's |yr| baseline stays high even when aligned). */
/* Alignment gate now only lightly throttles: the co-sim confirmed the drone stays DEAD-CENTERED
 * (drift <1 m) -- the earlier TBG_FLOOR=0.5 + narrow gate chronically halved forward speed (vy~0.42
 * m/s) as the (benign) yaw oscillated, ending 1.48 m short of gate 2. Keep the gate for gross-
 * misalignment safety only: high floor + wide full-engage angle. */
#ifndef TBG_FLOOR
#define TBG_FLOOR 0.35f
#endif
#ifndef ALIGN_FULL
#define ALIGN_FULL 0.44f     /* 25deg: strong throttle on misalignment => self-centering; full speed only when aligned */
#endif
/* Cruise scale + cap on the model's forward command so effective vx reaches ~1.2-1.5 m/s (the
 * no-gate run sustained ~1.5 flyably). Centering is handled by the alignment gate above. */
#ifndef CRUISE_SCALE
#define CRUISE_SCALE 1.5f
#endif
#ifndef VX_MAX
#define VX_MAX 1.6f
#endif
/* Clamp on the body-relative yaw-angle offset err[5] (rad) so a large transient policy yr can't
 * saturate the rotors and steal z/attitude headroom. */
#ifndef YAW_ERR_MAX
#define YAW_ERR_MAX  0.30f
#endif
#ifndef CTRL_ITERS
#define CTRL_ITERS   6000
#endif
#define TOF_MAX_RANGE 4.0f

/* Waypoint schedule (local maneuver): settle a hover holding the corridor center, then ramp
 * the forward (x) setpoint to WAYPOINT_X and hold, while keeping the lateral setpoint at the
 * corridor center (y=0). Gentle ramp so TinyMPC stays within its constraints. */
#ifndef SETTLE_ITERS
#define SETTLE_ITERS  600      /* 3 s: acquire walls + steady hover at center */
#endif
#define RAMP_ITERS    1000     /* 5 s: ramp x 0 -> WAYPOINT_X */
#define WAYPOINT_X    1.0f      /* advance 1 m down the corridor */

/* Navigation mode (compile-time; select with -DNAV_MODE via CMake, default waypoint):
 *   0 = WAYPOINT: regulate an x POSITION setpoint that ramps 0 -> WAYPOINT_X and holds.
 *   1 = VELOCITY: command a constant forward CRUISE velocity; x-position error is dropped.
 * Both keep the same estimator + TinyMPC; only the reference/error construction differs. */
#ifndef NAV_MODE
#define NAV_MODE 0
#endif
#define CRUISE_VX          0.4f   /* m/s forward cruise setpoint (NAV_MODE=1) */
#define CRUISE_RAMP_ITERS  200    /* ramp vx 0 -> CRUISE_VX (~1 s) so the MPC stays in-constraint */

/* Threaded producer/consumer build (see the ROSE_THREADED block below). Default = single loop. */
#ifndef ROSE_THREADED
#define ROSE_THREADED 0
#endif

/* ---- DroNet vision navigation (compile-time; -DROSE_DRONET_NAV=1) ---------------------------
 * A forward FPV camera frame is captured over the bridge, preprocessed (rose_preproc, the P1/P2
 * pipeline), and run through the quantized DroNet on the SoC. Its (steer, collision) outputs
 * drive the TinyMPC setpoint: collision -> forward speed (slow/stop near obstacles), steer ->
 * yaw (turn away). DroNet runs at a low rate (every VISION_DIV control ticks) with the command
 * zero-order-held between runs.
 *
 * Threaded: the IO thread does the camera CAPTURE (bridge I/O, kept single-threaded with the
 * sensor reads), and a low-priority vision thread does the COMPUTE (preproc + inference, ~450 ms
 * modeled) in the background so the control loop keeps meeting its rate via the ZOH'd command. */
#ifndef ROSE_DRONET_NAV
#define ROSE_DRONET_NAV 0
#endif
#if ROSE_DRONET_NAV
#include <zephyr/drivers/video.h>
#include <zephyr/sys/crc.h>
#include <rose/rose_preproc.h>
extern "C" {
#include "model.h"
}
/* DroNet output dequant scales (graph.json output tensors: linear1=steer, sigmoid1=collision). */
#define DRONET_STEER_SCALE      0.0002903275954441761f
#define DRONET_COLLISION_SCALE  0.003767134401741929f
#define VISION_DIV        100      /* attempt a DroNet run every 100 control ticks */
#define DRONET_CRUISE_VX  0.4f     /* max forward cruise (m/s), scaled by (1-collision) */
#define DRONET_YAW_GAIN   1.0f     /* steer -> yaw-angle setpoint (rad per unit steer) */
#define DRONET_CAM_NODE   DT_ALIAS(fpv)

static uint8_t g_cam_frame[320 * 240 * 3];
static int8_t  g_cam_tensor[ROSE_DRONET_INPUT_ELEMS];
static int8_t  g_cam_out[MODEL_DRONET_OUTPUT_SIZE];
static struct video_buffer g_cam_vbuf;
static uint16_t g_cam_w, g_cam_h;

struct vision_cmd { float steer; float collision; bool valid; };
static struct vision_cmd g_vision = { 0.0f, 0.5f, false };   /* ZOH; start neutral */

/* Bridge I/O: capture one RGB frame into g_cam_frame (called from the IO thread / single loop). */
static bool capture_frame(const struct device *cam)
{
	struct video_buffer *out = NULL;
	g_cam_vbuf.buffer = g_cam_frame;
	g_cam_vbuf.size = sizeof(g_cam_frame);
	g_cam_vbuf.type = VIDEO_BUF_TYPE_OUTPUT;
	if (video_enqueue(cam, &g_cam_vbuf) != 0) {
		return false;
	}
	if (video_dequeue(cam, &out, K_FOREVER) != 0 || out == NULL) {
		return false;
	}
	struct video_format fmt = { .type = VIDEO_BUF_TYPE_OUTPUT };
	video_get_format(cam, &fmt);
	g_cam_w = fmt.width;
	g_cam_h = fmt.height;
	return true;
}

/* Pure compute: preprocess the captured frame + run DroNet -> (steer, collision). No bridge I/O,
 * so it is safe to run in a background thread while the IO thread owns the transport. */
static void vision_infer(void)
{
	if (rose_preproc_rgb_to_dronet_i8(g_cam_frame, g_cam_w, g_cam_h, g_cam_tensor) != 0) {
		return;
	}
	run_model_dronet(g_cam_tensor, g_cam_out, NULL);
}
#endif /* ROSE_DRONET_NAV */

static const struct device *accel_dev = DEVICE_DT_GET(DT_ALIAS(bmi088_accel));
static const struct device *gyro_dev  = DEVICE_DT_GET(DT_ALIAS(bmi088_gyro));
#define HAVE_FLOW DT_NODE_EXISTS(DT_ALIAS(flow))
#define HAVE_TOF  DT_NODE_EXISTS(DT_ALIAS(tof))
#if HAVE_FLOW
static const struct device *flow_dev = DEVICE_DT_GET(DT_ALIAS(flow));
#endif
#if HAVE_TOF
static const struct device *tof_dev  = DEVICE_DT_GET(DT_ALIAS(tof));
#endif

/* Four horizontal multizone ToF (nearest wall per direction). */
static const struct device *tof_f = DEVICE_DT_GET(DT_ALIAS(tof_front));
static const struct device *tof_r = DEVICE_DT_GET(DT_ALIAS(tof_right));
static const struct device *tof_b = DEVICE_DT_GET(DT_ALIAS(tof_back));
static const struct device *tof_l = DEVICE_DT_GET(DT_ALIAS(tof_left));
static bool g_have_walls;
/* Altitude-hold setpoint (m). Defaults to TARGET_Z but is set to the measured spawn altitude at
 * startup (see main) so a hover holds where it starts — z-error≈0 → u≈hover — instead of being
 * commanded a step it over-reacts to. */
static float g_target_z = TARGET_Z;

/* Actuator-command staleness instrumentation: max mtime gap between consecutive send_control()
 * calls. In the single loop an inline vision inference blocks the loop, so the gap spikes to the
 * inference latency; in the threaded build control keeps sending, so the gap stays ~one grant. */
static uint64_t g_last_send, g_max_send_gap;
static uint32_t g_send_n;
static inline void track_send_gap(void)
{
	uint64_t now = k_cycle_get_64();
	if (g_last_send) {
		uint64_t d = now - g_last_send;
		if (d > g_max_send_gap) {
			g_max_send_gap = d;
		}
	}
	g_last_send = now;
	g_send_n++;
}

/* GT-STATE isolating test (-DROSE_GT_STATE=1): feed TinyMPC the drone's TRUE 12-vec state
 * from the sim (cmd 0x16) instead of the on-SoC estimator, to separate plant/controller from
 * estimator issues (mirrors IsaacCrazyflieMPCEnv's validated GT-state->TinyMPC->hover). */
#ifndef ROSE_GT_STATE
#define ROSE_GT_STATE 0
#endif
#define HAVE_ROSE DT_HAS_COMPAT_STATUS_OKAY(ucbbar_roseadapter)
#if HAVE_ROSE
#include <rose/rose.h>
#include <rose/rose_proto.h>
#define ROSE_CMD_CONTROL 0x20u
#define ROSE_CMD_GT_STATE 0x16u
static const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
static void send_control(const float *u)
{
	rose_tx(rose, ROSE_CMD_CONTROL);
	rose_tx(rose, NACTIONS * sizeof(float));
	for (int i = 0; i < NACTIONS; i++) {
		/* Clamp to TinyMPC's actuator box [u_min,u_max]=[-0.583,+0.417] before actuating, so
		 * the applied thrust never exceeds the physical motor range even if the solver's
		 * box constraint isn't tight this iteration (n = u+0.583 in [0,1]). */
		float uc = u[i];
		if (uc < -0.583f) uc = -0.583f;
		if (uc >  0.417f) uc =  0.417f;
		uint32_t w;
		memcpy(&w, &uc, sizeof(float));
		rose_tx(rose, w);
	}
	track_send_gap();
}
#else
static void send_control(const float *u) { (void)u; track_send_gap(); }
#endif

static TinyCache     cache;
static TinyWorkspace work;
static TinySettings  settings;
static TinySolver    solver;
static IStateEstimator &est = active_estimator();

/* ---- Fused-vision nav (ROSE_FUSED_NAV=1): the Stage-1 fused model drives the VELOCITY setpoint
 * while the on-SoC EKF + TinyMPC (this controller) close the loop -> motor thrusts. The model runs
 * at a low rate (every FUSED_VISION_DIV control ticks, ZOH between); its inputs are the same
 * pre-quantized front int8 / tof_cross int8 / lowdim f32 Stage-1 served, EXCEPT the guest OVERWRITES
 * the lowdim quat with its OWN EKF quaternion so the vision runs on the on-SoC attitude estimate. */
#ifndef ROSE_FUSED_NAV
#define ROSE_FUSED_NAV 0
#endif
#if ROSE_FUSED_NAV
#include <zephyr/drivers/video.h>
extern "C" {
#include "model.h"
}
#define FMPC_CMD_TOF_CROSS  0x41u
#define FMPC_CMD_LOWDIM     0x42u
#define FMPC_TOF_CROSS_CH   1
#define FMPC_LOWDIM_CH      2
#define FMPC_FRONT_ELEMS    5400
#define FMPC_TOF_ELEMS      256
#define FMPC_TOF_WORDS      64
#define FMPC_LOWDIM_ELEMS   21
#ifndef FUSED_VISION_DIV
#define FUSED_VISION_DIV    10        /* run the fused model every 10 ticks (20 Hz at 200 Hz).
                                        * 20 Hz fits the current 1x SoC (RVV inference 27.5ms < 50ms
                                        * budget) and threads gate 2 controlled (2/4) vs 10 Hz's 1/4 —
                                        * finer between-gate steering shrinks the post-gate over-swing. */
#endif
static uint8_t   fmpc_front[FMPC_FRONT_ELEMS];
static struct video_buffer fmpc_vbuf;
static int8_t    fmpc_tof[FMPC_TOF_ELEMS];
static float     fmpc_lowf[FMPC_LOWDIM_ELEMS];
static _Float16  fmpc_low16[FMPC_LOWDIM_ELEMS];
static _Float16  fmpc_out[MODEL_OUTPUT_SIZE];
static float     g_fwd = 0.0f, g_yr = 0.0f;   /* ZOH'd fused-model velocity command */
static float     g_misalign = 0.0f;           /* ZOH'd body-frame bearing to the goal gate (rad),
                                               * from the served desired_vel; 0 when heading at gate.
                                               * Drives the ALIGNMENT forward gate (Thread A). */
static bool      g_vision_valid = false;
static const struct device *fmpc_cam = DEVICE_DT_GET(DT_ALIAS(fpv));

#ifdef FMPC_CAM_REQRSP
/* Camera over the REQRSP path (bypass the ch0 DMA). The FPGA RoSEDMA delivers only the FIRST
 * frame then freezes (proven via the guest->sync VDIAG echo: cam hash constant while tof/lowdim
 * update and the sync serves fresh frames), so route cam_front (0x11) as a reqrsp on ch1 in the
 * env yaml and read it word-by-word like tof — the reqrsp path is the delivery-fixed, validated
 * one. 5400 int8 = 1350 words. */
#define FMPC_CMD_CAM_REQ  0x11u
#define FMPC_CAM_CH       1
#define FMPC_FRONT_WORDS  (FMPC_FRONT_ELEMS / 4)   /* 5400/4 = 1350 */
static bool fmpc_capture_front(void)
{
	static uint32_t raw[FMPC_FRONT_WORDS];
	rose_request(rose, FMPC_CMD_CAM_REQ, 0U);
	if (rose_recv_reqrsp(rose, FMPC_CAM_CH, raw, FMPC_FRONT_WORDS) < (int)FMPC_FRONT_WORDS) {
		return false;
	}
	memcpy(fmpc_front, raw, FMPC_FRONT_ELEMS);
	return true;
}
#else
static bool fmpc_capture_front(void)
{
	struct video_buffer *out = NULL;
	fmpc_vbuf.buffer = fmpc_front; fmpc_vbuf.size = sizeof(fmpc_front);
	fmpc_vbuf.type = VIDEO_BUF_TYPE_OUTPUT;
	if (video_enqueue(fmpc_cam, &fmpc_vbuf) != 0) return false;
	if (video_dequeue(fmpc_cam, &out, K_FOREVER) != 0 || out == NULL) return false;
	return true;
}
#endif
static bool fmpc_read_tof(void)
{
	uint32_t raw[FMPC_TOF_WORDS];
	rose_request(rose, FMPC_CMD_TOF_CROSS, 0U);
	if (rose_recv_reqrsp(rose, FMPC_TOF_CROSS_CH, raw, FMPC_TOF_WORDS) < FMPC_TOF_WORDS) return false;
	memcpy(fmpc_tof, raw, FMPC_TOF_ELEMS);
	return true;
}
static bool fmpc_read_lowdim(void)
{
	uint32_t raw[FMPC_LOWDIM_ELEMS];
	rose_request(rose, FMPC_CMD_LOWDIM, 0U);
	if (rose_recv_reqrsp(rose, FMPC_LOWDIM_CH, raw, FMPC_LOWDIM_ELEMS) < FMPC_LOWDIM_ELEMS) return false;
	for (int i = 0; i < FMPC_LOWDIM_ELEMS; i++) memcpy(&fmpc_lowf[i], &raw[i], sizeof(float));
	/* desired_vel occupies lowdim[12..14] (body-frame goal guidance). Body-frame bearing to the
	 * goal gate = atan2(dv_y, dv_x): 0 when the drone heads straight at the gate. This is the proper
	 * ALIGNMENT signal (goes to 0 on alignment) for the forward gate — unlike the noisy model |yr|. */
	g_misalign = atan2f(fmpc_lowf[13], fmpc_lowf[12]);
	return true;
}
/* Overwrite the lowdim quat slot [5..8]=[w,x,y,z] with the on-SoC EKF's OWN quaternion, from the
 * state's Rodrigues params r=q_xyz/q_w  ->  q=[1,r1,r2,r3]/|[1,r1,r2,r3]|. Then fp32->fp16 + run. */
static void fmpc_run_model(const float *state)
{
	float r1 = state[3], r2 = state[4], r3 = state[5];
	float n = sqrtf(1.0f + r1*r1 + r2*r2 + r3*r3);
	fmpc_lowf[5] = 1.0f / n; fmpc_lowf[6] = r1 / n; fmpc_lowf[7] = r2 / n; fmpc_lowf[8] = r3 / n;
	for (int i = 0; i < FMPC_LOWDIM_ELEMS; i++) fmpc_low16[i] = (_Float16)fmpc_lowf[i];
	run_model_fused_full((const model_fused_full_input0_t *)fmpc_front, fmpc_tof, fmpc_low16,
			     fmpc_out, NULL);
	g_yr  = (float)fmpc_out[0];
	g_fwd = (float)fmpc_out[1];
	g_vision_valid = true;
#ifdef FMPC_VISION_DBG
	/* Compute-vs-content discriminator, delivered over the WORKING guest->sync data channel
	 * (guest console printk is NOT captured on FPGA; a CRC-in-printk also stalls the loop).
	 * Order-sensitive FNV-1a hash of the exact model INPUTS + raw fp16 OUTPUT bits, sent as an
	 * unknown cmd 0x21 the sync logs+drops. inputs match spike + out differs => Saturn HW fp16
	 * compute; inputs differ => bridge/RTL content corruption. */
	{
		uint32_t hc = 2166136261u, ht = 2166136261u, hl = 2166136261u;
		const uint8_t *pc = (const uint8_t *)fmpc_front;
		for (int i = 0; i < FMPC_FRONT_ELEMS; i++) { hc ^= pc[i]; hc *= 16777619u; }
		const uint8_t *pt = (const uint8_t *)fmpc_tof;
		for (int i = 0; i < FMPC_TOF_ELEMS; i++) { ht ^= pt[i]; ht *= 16777619u; }
		const uint8_t *pl = (const uint8_t *)fmpc_low16;
		for (unsigned i = 0; i < sizeof(fmpc_low16); i++) { hl ^= pl[i]; hl *= 16777619u; }
		uint32_t o0 = 0, o1 = 0;
		memcpy(&o0, &fmpc_out[0], 2); memcpy(&o1, &fmpc_out[1], 2);
		rose_tx(rose, 0x21u);
		rose_tx(rose, 5u * 4u);
		rose_tx(rose, hc); rose_tx(rose, ht); rose_tx(rose, hl);
		rose_tx(rose, o0); rose_tx(rose, o1);
	}
#endif
}
#endif /* ROSE_FUSED_NAV */

static void mpc_init(void)
{
	solver.cache = &cache; solver.work = &work; solver.settings = &settings;
	tiny_init(&solver);
	init_VectorNx(&work.x1); init_VectorNx(&work.x2); init_VectorNx(&work.x3);
	init_VectorNu(&work.u1); init_VectorNu(&work.u2);
	cache.rho = rho_value;
	matsetv(cache.Kinf.data, Kinf_data, cache.Kinf.outer, cache.Kinf.inner);
	transpose(cache.Kinf.data, cache.KinfT.data, NINPUTS, NSTATES);
	matsetv(cache.Pinf.data, Pinf_data, cache.Pinf.outer, cache.Pinf.inner);
	transpose(cache.Pinf.data, cache.PinfT.data, NSTATES, NSTATES);
	matsetv(cache.Quu_inv.data, Quu_inv_data, cache.Quu_inv.outer, cache.Quu_inv.inner);
	matsetv(cache.AmBKt.data, AmBKt_data, cache.AmBKt.outer, cache.AmBKt.inner);
	transpose(cache.AmBKt.data, cache.AmBKtT.data, NSTATES, NSTATES);
	matsetv(cache.coeff_d2p.data, coeff_d2p_data, cache.coeff_d2p.outer, cache.coeff_d2p.inner);
	matsetv(work.Adyn.data, Adyn_data, work.Adyn.outer, work.Adyn.inner);
	transpose(work.Adyn.data, work.AdynT.data, NSTATES, NSTATES);
	matsetv(work.Bdyn.data, Bdyn_data, work.Bdyn.outer, work.Bdyn.inner);
	transpose(work.Bdyn.data, work.BdynT.data, NSTATES, NINPUTS);
	matsetv(work.Q.data, Q_data, work.Q.outer, work.Q.inner);
	matsetv(work.R.data, R_data, work.R.outer, work.R.inner);
	matset(work.u_min.data, -0.583, work.u_min.outer, work.u_min.inner);
	matset(work.u_max.data, 1 - 0.583, work.u_max.outer, work.u_max.inner);
	matset(work.x_min.data, -5, work.x_min.outer, work.x_min.inner);
	matset(work.x_max.data, 5, work.x_max.outer, work.x_max.inner);
	float Xref_origin[NSTATES] = {0};
	for (int j = 0; j < NHORIZON; j++) {
		matsetv(work.Xref.vector[j], Xref_origin, 1, NSTATES);
	}
}

static float read_dist(const struct device *dev)
{
	/* CENTER (perpendicular) zone: distance to the FACING wall for position-relative-to-a-wall. */
	struct sensor_value v;
	if (sensor_channel_get(dev, (enum sensor_channel)ROSE_SENSOR_CHAN_TOF_ZONE_CENTER, &v) < 0) {
		return TOF_MAX_RANGE;
	}
	return (float)sensor_value_to_double(&v);
}

/* A single set of sensor readings (one control tick's worth). */
struct nav_frame {
	float accel[3], gyro[3], flow[2], height;
	bool flow_valid, tof_valid;
	float df, db, dl, dr;   /* front/back/left/right wall distances (m) */
};

/* Bridge I/O: issue all sensor requests, then collect. Returns false on an IMU error. */
static bool read_nav_frame(struct nav_frame *f)
{
	int rc_a = sensor_sample_fetch(accel_dev);
	int rc_g = sensor_sample_fetch(gyro_dev);
#if HAVE_FLOW
	sensor_sample_fetch(flow_dev);
#endif
	f->tof_valid = false;
#if HAVE_TOF
	f->tof_valid = (sensor_sample_fetch(tof_dev) == 0);
#endif
	if (g_have_walls) {
		sensor_sample_fetch(tof_f); sensor_sample_fetch(tof_r);
		sensor_sample_fetch(tof_b); sensor_sample_fetch(tof_l);
	}
	if (rc_a < 0 || rc_g < 0) {
		return false;
	}

	struct sensor_value av[3], gv[3];
	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, av);
	sensor_channel_get(gyro_dev,  SENSOR_CHAN_GYRO_XYZ,  gv);
	for (int i = 0; i < 3; i++) {
		f->accel[i] = (float)sensor_value_to_double(&av[i]);
		f->gyro[i]  = (float)sensor_value_to_double(&gv[i]);
	}
	f->flow[0] = f->flow[1] = 0.0f;
	f->flow_valid = true;
#if HAVE_FLOW
	{
		struct sensor_value vx, vy;
		sensor_channel_get(flow_dev, (enum sensor_channel)ROSE_SENSOR_CHAN_FLOW_VX, &vx);
		sensor_channel_get(flow_dev, (enum sensor_channel)ROSE_SENSOR_CHAN_FLOW_VY, &vy);
		f->flow[0] = (float)sensor_value_to_double(&vx);
		f->flow[1] = (float)sensor_value_to_double(&vy);
		if (f->flow[0] != f->flow[0] || f->flow[1] != f->flow[1]) {
			f->flow_valid = false; f->flow[0] = f->flow[1] = 0.0f;
		}
	}
#endif
	f->height = START_Z;
#if HAVE_TOF
	if (f->tof_valid) {
		struct sensor_value h;
		sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &h);
		f->height = (float)sensor_value_to_double(&h);
	}
#endif
	if (g_have_walls) {
		f->df = read_dist(tof_f); f->db = read_dist(tof_b);
		f->dl = read_dist(tof_l); f->dr = read_dist(tof_r);
	}
	return true;
}

/* Run the estimator on one frame -> state[NSTATES]. */
static void run_estimator(const struct nav_frame *f, float *state)
{
	est.update(f->accel, f->gyro, f->flow, f->flow_valid, f->height, f->tof_valid, CTRL_DT);
	if (g_have_walls) {
		est.fuse_walls(f->df, f->db, f->dl, f->dr, TOF_MAX_RANGE);   /* front,back,left,right */
	}
	est.get_state(state);
}

/* Build the reference/error for the selected mode and solve TinyMPC -> u[NACTIONS].
 * steer/collision are the (ZOH'd) DroNet command; ignored unless ROSE_DRONET_NAV. */
static void solve_nav(int iter, const float *state, float steer, float collision,
		      float *u, float *logcmd)
{
	float setpoint[NSTATES] = {0.0f, 0.0f, g_target_z, 0,0,0, 0,0,0, 0,0,0};
	float err[NSTATES];
	(void)steer; (void)collision;
#if ROSE_FUSED_NAV
	/* Fused-vision velocity nav with the FAITHFUL body yaw-RATE command (the intended interface;
	 * matches Stage-1's Lee velocity tracker which flew 4/4). The policy emits body-frame
	 * (yaw_rate, fwd): drive setpoint[11]=yaw_rate directly (smooth rate tracking, no bang-bang) and
	 * NEUTRALIZE the yaw-ANGLE channel (setpoint[5]=state[5] -> err[5]=0) so Q5 doesn't fight. A
	 * yaw RATE needs no absolute-yaw reference (drift-immune). Usable now that (a) the plant applies
	 * the drag yaw torque to the BASE and (b) the yawfix params weight/mix yaw-rate correctly. */
	float yr_cmd = (iter > SETTLE_ITERS) ? g_yr : 0.0f;
	float vx_cmd = (iter > SETTLE_ITERS) ? g_fwd * CRUISE_SCALE : 0.0f;  /* cruise scale -> ~1.2-1.5 m/s */
	if (vx_cmd < 0.0f) vx_cmd = 0.0f;
	if (vx_cmd > VX_MAX) vx_cmd = VX_MAX;                                /* cap so it stays flyable */
	/* ALIGNMENT forward gate (Thread A): slow forward while the HEADING is mis-aligned with the goal
	 * gate (|g_misalign| large), so off-line-spawn drift can't accumulate during the yaw correction;
	 * restores full speed on alignment (g_misalign -> 0). Driven by the served desired_vel bearing
	 * (a proper alignment signal that goes to 0 when aligned) -- unlike the model's noisy |yr|, which
	 * stays high even when aligned and deadlocks the |yr|-gate. TBG_FLOOR keeps min forward progress. */
	float align = 1.0f - fabsf(g_misalign) / ALIGN_FULL;
	if (align < 0.0f) align = 0.0f;
	float gate = TBG_FLOOR + (1.0f - TBG_FLOOR) * align * align;
	vx_cmd *= gate;
	/* REFERENCE-FRAME FIX: the EKF emits state[6],[7] as WORLD-frame velocity (optical flow rotated
	 * body->world, "ground-truth convention"), but the fused policy's fwd is a BODY-forward speed and
	 * the TinyMPC model is hover-linearized at yaw=0 (its x-axis == the nose). Writing fwd into the
	 * world-vx slot made the drone yaw its NOSE while still translating the OLD world direction --
	 * confirmed in the trace: body-lateral sideslip grows to ~1 m/s as the nose turns off the motion.
	 * Command-rotation alone would be WRONG (a yaw-0 K would roll to chase world-y instead of pitching
	 * along the nose); the correct fix is to DEROTATE the velocity feedback into the body-heading frame
	 * so K's body assumption holds. Then err[6]=body-forward-vel - fwd and err[7]=body-lateral-vel->0:
	 * the MPC turns the velocity vector to follow the nose (coordinated turn), matching Stage-1's Lee
	 * tracker (which rotated fwd by yaw) and the host prototype (which flies in the body frame). */
	float rr1 = state[3], rr2 = state[4], rr3 = state[5];   /* Rodrigues attitude r = q_xyz/q_w */
	float qni = 1.0f / sqrtf(1.0f + rr1*rr1 + rr2*rr2 + rr3*rr3);
	float qw = qni, qx = rr1*qni, qy = rr2*qni, qz = rr3*qni;
	float yaw = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
	float cy = cosf(yaw), sy = sinf(yaw);
	float vfwd =  cy*state[6] + sy*state[7];               /* world vel projected onto the nose */
	float vlat = -sy*state[6] + cy*state[7];               /* body-lateral velocity (sideslip) */
	setpoint[5]  = state[5];                                /* neutralize yaw-ANGLE channel (err[5]=0) */
	setpoint[11] = yr_cmd;                                  /* faithful body yaw RATE (wz) */
	for (int i = 0; i < NSTATES; i++) err[i] = state[i] - setpoint[i];
	err[0] = 0.0f; err[1] = 0.0f;                           /* x,y position free */
	err[6] = vfwd - vx_cmd;                                 /* body-forward velocity tracks fwd cmd */
	err[7] = vlat;                                          /* drive body-lateral sideslip to 0 */
	*logcmd = vx_cmd;
#elif ROSE_DRONET_NAV
	/* DroNet vision nav: collision -> forward speed, steer -> yaw. Hover through settle. */
	float vx_cmd = (1.0f - collision) * DRONET_CRUISE_VX;
	if (vx_cmd < 0.0f) vx_cmd = 0.0f;
	if (iter <= SETTLE_ITERS) vx_cmd = 0.0f;
	setpoint[5] = steer * DRONET_YAW_GAIN;      /* yaw angle from steer */
	setpoint[6] = vx_cmd;                       /* forward velocity from collision */
	for (int i = 0; i < NSTATES; i++) err[i] = state[i] - setpoint[i];
	err[0] = 0.0f;                              /* cruise: x position free */
	if (!g_have_walls) err[1] = 0.0f;
	*logcmd = vx_cmd;
#elif NAV_MODE == 1
	float vx_cmd = 0.0f;
	if (iter > SETTLE_ITERS) {
		float fr = (float)(iter - SETTLE_ITERS) / (float)CRUISE_RAMP_ITERS;
		if (fr > 1.0f) fr = 1.0f;
		vx_cmd = CRUISE_VX * fr;
	}
	setpoint[6] = vx_cmd;
	for (int i = 0; i < NSTATES; i++) err[i] = state[i] - setpoint[i];
	err[0] = 0.0f;
	if (!g_have_walls) err[1] = 0.0f;
	*logcmd = vx_cmd;
#else
	float sx = 0.0f;
	if (iter > SETTLE_ITERS) {
		float fr = (float)(iter - SETTLE_ITERS) / (float)RAMP_ITERS;
		if (fr > 1.0f) fr = 1.0f;
		sx = WAYPOINT_X * fr;
	}
	setpoint[0] = sx;
	for (int i = 0; i < NSTATES; i++) err[i] = state[i] - setpoint[i];
	if (!g_have_walls) { err[0] = 0.0f; err[1] = 0.0f; }
	*logcmd = sx;
#endif

	matsetv(work.x.vector[0], err, 1, NSTATES);
	matset(work.y.data, 0.0, work.y.outer, work.y.inner);
	matset(work.g.data, 0.0, work.g.outer, work.g.inner);
	tiny_solve(&solver);
	for (int i = 0; i < NACTIONS; i++) {
		u[i] = work.u.vector[0][i];
	}
}

static const char *nav_tag(void)
{
#if ROSE_FUSED_NAV
	return "fused";
#else
	return ROSE_DRONET_NAV ? "dronet" : (NAV_MODE == 1 ? "vel" : "wp");
#endif
}

/* ============================ timer-driven control (co-sim-agnostic) ============================ */
#if ROSE_THREADED
/* Real-prototype paradigm: a periodic control task paced by its OWN timer, plus a background
 * vision task. The app has no co-sim awareness — the same structure runs on real hardware. The
 * control rate is set by the timer, NOT by the simulator; if a control tick overruns, the physics
 * just keeps stepping on the last (ZOH-latched) actuation until the next command — realistic
 * degradation rather than the simulator waiting for the SoC.
 *
 * mtime advances during WFI (the digital-top-clock model, verified against this spike's clint), so
 * the control task idles on the period semaphore between ticks with NO keepalive; that idle CPU
 * goes to the lower-priority vision task by ordinary preemption (instruction-granular,
 * tick-independent). ALL bridge I/O (sensor reqrsp, camera capture, control TX) stays in the
 * control task, so the transport is never touched concurrently; the vision task is compute-only. */
#ifndef CTRL_HZ
#define CTRL_HZ 200
#endif
#define PRIO_CTRL    3
#define PRIO_VISION  10
K_THREAD_STACK_DEFINE(ctrl_stack, 327680);   /* TinyMPC working set */
static struct k_thread ctrl_t;

/* Periodic control clock: the timer gives sem_tick every 1/CTRL_HZ; the control task blocks on it
 * (WFI-idle) and runs one iteration per tick. A pending tick at wake => that iteration overran. */
static struct k_timer ctrl_timer;
K_SEM_DEFINE(sem_tick, 0, 1);
static void ctrl_timer_fn(struct k_timer *t) { ARG_UNUSED(t); k_sem_give(&sem_tick); }

#if ROSE_DRONET_NAV
static struct k_thread vision_t;
K_THREAD_STACK_DEFINE(vision_stack, 16384);
K_MUTEX_DEFINE(mtx_vision);
K_SEM_DEFINE(sem_vision, 0, 1);            /* control -> vision: a fresh frame was captured */
static atomic_t vision_busy;               /* 1 while vision holds g_cam_frame */
static const struct device *g_cam;
#endif

#if ROSE_DRONET_NAV
static void vision_block(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (;;) {
		k_sem_take(&sem_vision, K_FOREVER);   /* frame already captured by IO into g_cam_frame */
		vision_infer();                       /* preproc + DroNet (pure compute, preemptible) */
		k_mutex_lock(&mtx_vision, K_FOREVER);
		g_vision.steer = (float)g_cam_out[0] * DRONET_STEER_SCALE;
		g_vision.collision = (float)g_cam_out[1] * DRONET_COLLISION_SCALE;
		g_vision.valid = true;
		k_mutex_unlock(&mtx_vision);
		atomic_set(&vision_busy, 0);          /* release g_cam_frame back to IO */
	}
}
#endif

/* Periodic control task: sensors -> estimate -> (ZOH vision cmd) -> solve -> actuate, once per
 * timer tick. Sequential like a real flight-control ISR; the only asynchronous work is vision. */
static void ctrl_block(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	float state[NSTATES], u[NACTIONS];
	uint64_t last = 0, min_p = ~0ULL, max_p = 0;
	uint32_t overruns = 0;
	for (int iter = 0; iter < CTRL_ITERS; iter++) {
		k_sem_take(&sem_tick, K_FOREVER);                 /* next period (WFI-idle -> vision runs) */
		if (k_sem_take(&sem_tick, K_NO_WAIT) == 0) {      /* a tick was already pending => overran */
			overruns++;
		}

		uint64_t tp0 = k_cycle_get_64();
		struct nav_frame f;
		if (!read_nav_frame(&f)) {
			continue;
		}
		uint64_t tp1 = k_cycle_get_64();   /* sensor-read (reqrsp round-trip) latency */
#if ROSE_DRONET_NAV
		/* Kick a background vision run: capture the frame here (bridge I/O stays in this task),
		 * hand the compute to the vision task. Skip if it is still busy. */
		if (g_cam && (iter % VISION_DIV) == 0 && atomic_get(&vision_busy) == 0) {
			if (capture_frame(g_cam)) {
				atomic_set(&vision_busy, 1);
				k_sem_give(&sem_vision);
			}
		}
#endif
		run_estimator(&f, state);
		uint64_t tp2 = k_cycle_get_64();   /* estimator compute */

		float steer = 0.0f, collision = 0.5f;
#if ROSE_DRONET_NAV
		k_mutex_lock(&mtx_vision, K_FOREVER);
		steer = g_vision.steer; collision = g_vision.collision;
		k_mutex_unlock(&mtx_vision);
#endif
		float logcmd = 0.0f;
		solve_nav(iter, state, steer, collision, u, &logcmd);
		uint64_t tp3 = k_cycle_get_64();   /* TinyMPC solve compute */
		send_control(u);

		uint64_t now = k_cycle_get_64();
		uint64_t period = last ? (now - last) : 0;
		last = now;
		if (period) { if (period < min_p) min_p = period; if (period > max_p) max_p = period; }
		if ((iter % 40) == 0) {
			printk("  tick_us: sensor_read=%d est=%d solve=%d (mtime@10MHz)\n",
			       (int)((tp1 - tp0) / 10), (int)((tp2 - tp1) / 10), (int)((tp3 - tp2) / 10));
			printk("nav[%s]: iter=%d z=%d.%03d vx=%d.%03d cmd=%d.%03d period_us=%d overruns=%u\n",
			       nav_tag(), iter,
			       (int)state[2], (int)(state[2]*1000)%1000,
			       (int)state[6], (int)(state[6]*1000)%1000,
			       (int)logcmd, (int)(logcmd*1000)%1000,
			       (int)(period / 10), overruns);   /* mtime @10 MHz -> /10 = microseconds */
#if ROSE_DRONET_NAV
			printk("  dronet: steer_m=%d coll_m=%d out_i8=[%d,%d] valid=%d busy=%d\n",
			       (int)(steer*1000), (int)(collision*1000),
			       g_cam_out[0], g_cam_out[1], g_vision.valid, (int)atomic_get(&vision_busy));
#endif
		}
	}
	k_timer_stop(&ctrl_timer);
	printk("nav_controller: done (%d iters, timer-driven @%d Hz) period_us[min=%d max=%d] "
	       "overruns=%u max_send_gap_us=%d sends=%u\n",
	       CTRL_ITERS, CTRL_HZ, (int)(min_p / 10), (int)(max_p / 10),
	       overruns, (int)(g_max_send_gap / 10), g_send_n);
}
#endif /* ROSE_THREADED */

int main(void)
{
	if (!device_is_ready(accel_dev) || !device_is_ready(gyro_dev)) {
		printk("nav_controller: FAIL (IMU not ready)\n");
		return -1;
	}
	g_have_walls = device_is_ready(tof_f) && device_is_ready(tof_r) &&
		       device_is_ready(tof_b) && device_is_ready(tof_l);
	enable_vector_operations();
	mpc_init();
	/* Initialize the EKF altitude to the FIRST measured down-ToF height (= the actual spawn
	 * altitude) instead of the fixed START_Z, so there is NO large ToF innovation transient for
	 * any co-sim seed. Force past the ToF driver's decimation so the first fetch actually ranges. */
	float init_z = START_Z;
#if HAVE_TOF
	if (device_is_ready(tof_dev)) {
		for (int k = 0; k < 12; k++) {
			if (sensor_sample_fetch(tof_dev) == 0) {
				struct sensor_value hv;
				if (sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &hv) == 0) {
					init_z = (float)sensor_value_to_double(&hv);
				}
				break;
			}
		}
	}
#endif
	est.init(0.0f, 0.0f, init_z);   /* EKF starts at the MEASURED spawn (no ToF innovation transient) */
	est.set_init_yaw(START_YAW);   /* one-time takeoff-heading calibration (gyro-only yaw has no ref) */
	/* Hold the GATE-CENTERLINE / trained altitude (TARGET_Z, the FUSED_GATES centre z=2.0), NOT the
	 * spawn height. The warehouse PLAY reset spawns ~2.5 m (pose_range +0.5 default offset), 0.5 m
	 * ABOVE the gates -> the front camera sees each gate low in-frame (off-distribution vs the
	 * expert/Stage-1 which flew altitude-hold to 2.0). Descending to 2.0 puts the gates centered in
	 * frame (in-distribution) and threads them at the centerline. (Stage-1's TARGET_H=2.0 pattern.) */
	g_target_z = TARGET_Z;
	printk("nav_controller: EKF init z=%d.%03d m, hover_target=%d.%03d m (target=gate centerline), "
	       "yaw_seed=%d mrad\n",
	       (int)init_z, ((int)(init_z * 1000)) % 1000,
	       (int)g_target_z, ((int)(g_target_z * 1000)) % 1000, (int)(START_YAW * 1000));

#if ROSE_DRONET_NAV
	const struct device *cam = DEVICE_DT_GET(DRONET_CAM_NODE);
	if (!device_is_ready(cam)) {
		printk("nav_controller: WARN DroNet camera not ready — vision disabled\n");
		cam = NULL;
	} else {
		printk("nav_controller: DroNet vision nav ENABLED (model=%s in=%d out=%d, every %d ticks)\n",
		       MODEL_DRONET_NAME, MODEL_DRONET_INPUT_SIZE, MODEL_DRONET_OUTPUT_SIZE, VISION_DIV);
	}
#endif
	printk("nav_controller: estimator=%s + TinyMPC ready, walls=%d, threaded=%d, mode=%s\n",
	       est.name(), g_have_walls, ROSE_THREADED, nav_tag());
#if ROSE_FUSED_NAV
	if (!device_is_ready(fmpc_cam)) {
		printk("nav_controller: FAIL fused camera not ready\n");
		return -1;
	}
	printk("nav_controller: FUSED vision ENABLED (model=%s in=%d out=%d, every %d ticks)\n",
	       MODEL_NAME, MODEL_INPUT_SIZE, MODEL_OUTPUT_SIZE, FUSED_VISION_DIV);
#endif

#if ROSE_THREADED
#if ROSE_DRONET_NAV
	g_cam = cam;
	k_thread_create(&vision_t, vision_stack, K_THREAD_STACK_SIZEOF(vision_stack),
			vision_block, NULL, NULL, NULL, PRIO_VISION, 0, K_NO_WAIT);
#endif
	k_thread_create(&ctrl_t, ctrl_stack, K_THREAD_STACK_SIZEOF(ctrl_stack),
			ctrl_block, NULL, NULL, NULL, PRIO_CTRL, 0, K_NO_WAIT);
	/* Start the periodic control clock (first tick after one period, then every 1/CTRL_HZ). The
	 * control task paces itself off this timer — independent of the simulator's step rate. */
	k_timer_init(&ctrl_timer, ctrl_timer_fn, NULL);
	k_timer_start(&ctrl_timer, K_USEC(1000000 / CTRL_HZ), K_USEC(1000000 / CTRL_HZ));
	k_thread_join(&ctrl_t, K_FOREVER);   /* run until the control task finishes its iterations */
	return 0;
#else
	/* ---- single loop (A/B baseline) ---- */
	float state[NSTATES], u[NACTIONS];
	for (int iter = 0; iter < CTRL_ITERS; iter++) {
#if ROSE_GT_STATE
		/* GT-STATE test: TinyMPC on the sim's TRUE 12-vec state (bypass the estimator). */
		{
			uint32_t sb[NSTATES];
			rose_request(rose, ROSE_CMD_GT_STATE, 0u);
			int got = rose_recv_reqrsp(rose, 1, sb, NSTATES);
			for (int i = 0; i < NSTATES && i < got; i++) {
				memcpy(&state[i], &sb[i], sizeof(float));
			}
		}
#else
		struct nav_frame f;
		if (!read_nav_frame(&f)) {
			printk("nav_controller: IMU fetch error\n");
			continue;
		}
		run_estimator(&f, state);
#endif

#if ROSE_FUSED_NAV
		/* Low-rate inline fused-model run (ZOH between): capture front + tof_cross + lowdim from
		 * the (frozen) sensor frame, insert the EKF quat, run the model -> (yaw_rate, fwd). */
		if ((iter % FUSED_VISION_DIV) == 0) {
			if (fmpc_capture_front() && fmpc_read_tof() && fmpc_read_lowdim()) {
				fmpc_run_model(state);
			}
		}
#endif
		float steer = 0.0f, collision = 0.5f;
#if ROSE_DRONET_NAV
		if (cam && (iter % VISION_DIV) == 0) {
			if (capture_frame(cam)) {
				vision_infer();      /* INLINE: blocks the loop for the whole inference */
				g_vision.steer = (float)g_cam_out[0] * DRONET_STEER_SCALE;
				g_vision.collision = (float)g_cam_out[1] * DRONET_COLLISION_SCALE;
				g_vision.valid = true;
			}
		}
		steer = g_vision.steer; collision = g_vision.collision;
#endif
		float logcmd = 0.0f;
		solve_nav(iter, state, steer, collision, u, &logcmd);
		if ((iter % 20) == 0) {
			printk("nav[%s]: iter=%d z=%d.%03d vx=%d.%03d cmd=%d.%03d max_send_gap_us=%d\n",
			       nav_tag(), iter,
			       (int)state[2], (int)(state[2]*1000)%1000,
			       (int)state[6], (int)(state[6]*1000)%1000,
			       (int)logcmd, (int)(logcmd*1000)%1000,
			       (int)(g_max_send_gap / 10));
#if ROSE_FUSED_NAV
			printk("  fused: yr_m=%d fwd_m=%d valid=%d\n",
			       (int)(g_yr*1000), (int)(g_fwd*1000), (int)g_vision_valid);
#endif
		}
		send_control(u);
	}
	printk("nav_controller: done (%d iters, single-loop) max_send_gap_us=%d sends=%u\n",
	       CTRL_ITERS, (int)(g_max_send_gap / 10), g_send_n);
	return 0;
#endif
}
