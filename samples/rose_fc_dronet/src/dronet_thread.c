/*
 * DroNet background inference thread for the integrated FC + DroNet build on the
 * riskybird v3 FPGA Rocket combined core RocketArty200TDroneGemminiSaturnFp16At35Config
 * (Gemmini + Saturn VLEN=128 + full drone periphery, misa.F=0 / Zfh-without-F).
 *
 * WHAT THIS IS
 *   The flight controller runs its PID control loop in main() at Zephyr prio 0.
 *   This thread (prio DRONET_PRIO, below the loop) runs DroNet inference over a
 *   static 112x112x1 grayscale tensor as fast as the shared hart allows, in the
 *   slack the control loop leaves. It self-verifies bit-exactness vs the baked
 *   golden (max_abs_err <= 3) and reports concurrent fps. Modeled on the proven
 *   rose_fused_mpc / rose_fused_nav low-prio vision-thread + ZOH pattern.
 *
 * PREEMPTION SAFETY -- MEASURED on the At35 (2026-09-05):
 *   (1) FENCE JITTER: DroNet's Gemmini conv (tiled_conv_auto) drains its write
 *       DMA with a hart-stalling `fence` that holds off the timer ISR for the
 *       whole array drain -> concurrent PID loop stretched to 16-24 ms. FIX
 *       (owned by the load-once poll conv, a28efeb): per-tile WDMA_BYTES_SENT
 *       poll + yield so the conv is preemptible. conv2d_s8 dispatches are left
 *       UNMASKED here so that poll conv can run and the PID loop preempts it.
 *   (2) OUTPUT CORRUPTION under full preemption (max_abs_err 10-245, saturated).
 *       Isolation (this file supports it): rose_fused_mpc's PURE-RVV vision net
 *       preempts cleanly under the same eager V save/restore, so the corruption
 *       is NOT fundamental RVV preemption -- it is the Gemmini<->RVV interaction
 *       (a context switch, whose eager V save runs vse8.v, taken while a Gemmini
 *       mvout DMA is in flight). Zephyr's v.c saves/restores all of
 *       v0..v31 + vstart/vl/vtype/vcsr, VLEN=128 <= MAX_LEN so the save area
 *       does not overflow, and switch.S runs no V op after restore -- so it is
 *       not a save/restore gap or a MAX_LEN drift.
 *
 * TOOLS FOR THE ISOLATION / FIX (all compile-time):
 *   - Atomic baseline: the first inference runs fully IRQ-locked (no preemption)
 *     -> its max_abs_err is the control case (must be <=3) and it populates the
 *     per-op IR-kind records used to classify ops below.
 *   - DRONET_MASK_OPS: comma-separated substrings of IR op kinds to run with
 *     IRQs masked (never preempted mid-op). Tune to the MINIMAL set that makes
 *     max_abs_err<=3 under preemption while keeping each masked op < the PID
 *     budget. If the Gemmini<->RVV race is confirmed, this is empty and the poll
 *     conv (drain-then-yield, per-tile masked) carries the Gemmini safety; if
 *     any short RVV op also corrupts, add it here (bn/add/relu/relayout are the
 *     us-scale candidates -- masking them adds only us of PID jitter).
 *   - DRONET_IRQ_GATE=1: coarse fallback -- mask EVERY op (correct but breaks
 *     >=200 Hz because the long conv is then non-preemptible; correctness proof
 *     only).
 */

#include <zephyr/kernel.h>
#include <string.h>
#include "model.h"
#include "test_io.h"   /* model_dronet_test_input[], model_dronet_test_golden[] */

#ifndef DRONET_PRIO
#define DRONET_PRIO 7          /* below PID loop (0), above tof(8)/camera(12) */
#endif
#ifndef DRONET_STACK_SIZE
#define DRONET_STACK_SIZE (256 * 1024)
#endif
#ifndef DRONET_START_DELAY_MS
#define DRONET_START_DELAY_MS 4000
#endif
#ifndef DRONET_REPORT_EVERY
#define DRONET_REPORT_EVERY 1
#endif
#ifndef DRONET_IRQ_GATE
#define DRONET_IRQ_GATE 0
#endif
#ifndef DRONET_YIELD_DIV
#define DRONET_YIELD_DIV 1   /* time-slice knob: k_yield every N ops (see dronet_run_once) */
#endif
#ifndef DRONET_VERIFY_ATOL
#define DRONET_VERIFY_ATOL 3   /* max_abs_err vs the golden that still counts as bit-exact */
#endif

/* ---- live navigation input (ROSE_DRONET_NAV) ------------------------------------------------
 * Instead of looping on the baked test tensor, run DroNet on camera frames and publish each output
 * to the control loop (dronet_cmd.h), stamped with the frame's capture time. The Gemmini<->RVV
 * context-switch hazard described above corrupts outputs silently and depends on preemption timing,
 * so a golden check on one run says nothing about the next. Two defences, both UNDER PREEMPTION:
 *   - DRONET_DUAL_RUN: every live frame is inferred twice and published only if the runs agree --
 *     a transient corruption would have to repeat identically to get through. Halves the rate.
 *   - DRONET_VERIFY_EVERY: the golden input is re-run periodically.
 * Either failing withdraws the latest command (dronet_cmd_invalidate) and stops the thread for good;
 * the control loop then sees no command and holds hover. */
#ifndef ROSE_DRONET_NAV
#define ROSE_DRONET_NAV 0
#endif
#if ROSE_DRONET_NAV
#include "camera_dma.h"
#include "dronet_cmd.h"
/* Pixel -> int8 input as q = pixel - DRONET_IN_ZERO (input tensor zero point 0, scale 0.0329 in the
 * gray graph). The committed gray model was calibrated on a synthetic N(0,1) frame, so this centring
 * is a placeholder until a trained grayscale model fixes the real preprocessing. */
#ifndef DRONET_IN_ZERO
#define DRONET_IN_ZERO 128
#endif
#ifndef DRONET_GRAB_TIMEOUT_MS
#define DRONET_GRAB_TIMEOUT_MS 500
#endif
#ifndef DRONET_VERIFY_EVERY
#define DRONET_VERIFY_EVERY 20   /* golden re-check every N live frames; 0 = boot baseline only */
#endif
#ifndef DRONET_DUAL_RUN
#define DRONET_DUAL_RUN 1        /* infer each live frame twice; publish only if they agree */
#endif
#define DRONET_IN_W 112
#define DRONET_IN_H 112
BUILD_ASSERT(DRONET_IN_W * DRONET_IN_H == MODEL_DRONET_INPUT_SIZE, "DroNet input is not 112x112x1");
BUILD_ASSERT(MODEL_DRONET_INPUT_SIZE <= CAMERA_DMA_GRAB_MAX, "camera grab buffer too small");
static uint8_t live_gray[MODEL_DRONET_INPUT_SIZE];
static model_dronet_input_t live_in[MODEL_DRONET_INPUT_SIZE];
#endif
/* Bit-exact-under-preemption set (measured on the At35). The poll+yield conv
 * (dronet_model_poll/kernels.c) handles the Gemmini convs; these SHORT RVV ops
 * are IRQ-masked whole (each << the >=200 Hz budget) because the Saturn
 * mis-resumes their mid-execution strided vector memory ops at vstart!=0.
 *
 * maxpool2d_s8: FLIGHT DEFAULT is the fault-free WHOLE-OP mask (in the list
 * below) -- zero preemption window, guaranteed no Saturn strided mis-resume.
 * COST: the maxpool runs ~25 ms IRQ-off, so the control loop sees ONE ~25 ms
 * gap per DroNet frame (bench: FC+DroNet mean 166 Hz, worst-case jitter 43 ms).
 * The maxpool2d_s8 kernel ALSO self-protects PER CHANNEL (irq_lock a ~0.8 ms
 * channel around the strided vlse8, k_yield between channels); to trade the
 * safety margin for a tighter loop, build with a DRONET_MASK_OPS that OMITS
 * "maxpool" -- the per-channel self-protect then keeps the loop running
 * (bench: mean 168 Hz, worst-case jitter 18 ms, still err=3/no-fault across the
 * runs measured, but leaves a small per-channel preemption window). */
#ifndef DRONET_MASK_OPS
#define DRONET_MASK_OPS "batchnorm", "add_s8", "relu", "linear", "sigmoid", "maxpool"
#endif

static model_dronet_output_t dronet_out[MODEL_DRONET_OUTPUT_SIZE];
static uint8_t op_masked[MODEL_DRONET_OP_COUNT];

static int op_should_mask(const char *op)
{
	if (op == NULL) return 0;
	static const char *masks[] = { DRONET_MASK_OPS };
	for (size_t i = 0; i < sizeof(masks) / sizeof(masks[0]); i++) {
		if (masks[i] && strstr(op, masks[i]) != NULL) return 1;
	}
	return 0;
}

static int dronet_verify(void)
{
	int max_abs_err = 0;
	for (int i = 0; i < MODEL_DRONET_TEST_OUTPUT_LEN; i++) {
		int d = (int)dronet_out[i] - (int)model_dronet_test_golden[i];
		if (d < 0) d = -d;
		if (d > max_abs_err) max_abs_err = d;
	}
	return max_abs_err;
}

/* One inference. steady=0: whole inference IRQ-locked (atomic control case +
 * populates op-kind records). steady=1: per-op masking from op_masked[], with a
 * k_yield at each op boundary so the PID loop preempts. Returns summed per-op
 * compute cycles; *wall_ticks gets the mtime delta (includes PID-steal time). */
static unsigned long dronet_run_once(const model_dronet_input_t *input, int steady,
				     unsigned long *wall_ticks)
{
	model_dronet_state_t s = {
		.input = input, .output = dronet_out, .pool = NULL,
	};
	model_dronet_reset_profile();
	unsigned long t0 = (unsigned long)k_cycle_get_64();

	if (!steady) {
		unsigned int key = irq_lock();
		for (int i = 0; i < MODEL_DRONET_OP_COUNT; i++)
			MODEL_DRONET_DISPATCH_FNS[i](&s);
		irq_unlock(key);
	} else {
		for (int i = 0; i < MODEL_DRONET_OP_COUNT; i++) {
#if DRONET_IRQ_GATE
			unsigned int key = irq_lock();
			MODEL_DRONET_DISPATCH_FNS[i](&s);
			irq_unlock(key);
#else
			if (op_masked[i]) {
				unsigned int key = irq_lock();
				MODEL_DRONET_DISPATCH_FNS[i](&s);
				irq_unlock(key);
			} else {
				MODEL_DRONET_DISPATCH_FNS[i](&s);
			}
			/* Time-slice knob (loop-rate <-> DroNet-fps tradeoff): yield to the PID
			 * loop every DRONET_YIELD_DIV ops. 1 = yield every op (max loop rate,
			 * lowest DroNet fps -- the ops are tiny so the loop preempts often); larger
			 * = DroNet runs more ops between yields (higher fps, lower loop rate);
			 * DRONET_IRQ_GATE=1 = never yield = atomic (max fps, lowest loop rate). */
			if ((i % DRONET_YIELD_DIV) == (DRONET_YIELD_DIV - 1)) k_yield();
#endif
		}
	}

	__asm__ volatile("fence rw, rw" ::: "memory");
	*wall_ticks = (unsigned long)k_cycle_get_64() - t0;

	int n = 0;
	const model_dronet_op_record_t *rec = model_dronet_profile_records(&n);
	unsigned long compute = 0;
	for (int i = 0; i < n; i++) compute += rec[i].cycles;
	return compute;
}

static void dronet_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	const uint32_t hz = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;

	printk("DRONET: thread up (prio %d, gate=%d) model=%s in=%d out=%d ops=%d\n",
	       DRONET_PRIO, DRONET_IRQ_GATE, MODEL_DRONET_NAME,
	       MODEL_DRONET_INPUT_SIZE, MODEL_DRONET_OUTPUT_SIZE, MODEL_DRONET_OP_COUNT);

	/* Atomic baseline + op classification (one-time). */
	unsigned long wt = 0;
	unsigned long compute0 = dronet_run_once(model_dronet_test_input, 0, &wt);
	int base_err = dronet_verify();
	int n = 0;
	const model_dronet_op_record_t *rec = model_dronet_profile_records(&n);
	int nmask = 0;
	for (int i = 0; i < n && i < MODEL_DRONET_OP_COUNT; i++) {
		op_masked[i] = (uint8_t)op_should_mask(rec[i].op);
		if (op_masked[i]) nmask++;
	}
	printk("DRONET: baseline(atomic) compute=%lu kcyc err=%d out=[%d,%d]; masking %d/%d ops\n",
	       compute0 / 1000UL, base_err, (int)dronet_out[0], (int)dronet_out[1], nmask, n);

#if ROSE_DRONET_NAV
	if (base_err > DRONET_VERIFY_ATOL) {
		printk("DRONET: baseline err=%d > %d -- navigation commands DISABLED\n",
		       base_err, DRONET_VERIFY_ATOL);
		k_sleep(K_FOREVER);   /* never publish; give the CPU back to the flight controller */
	}
	unsigned long frames = 0, misses = 0;
	for (;;) {
		int64_t t_capture = 0;
		int32_t frame = camera_dma_grab_gray(live_gray, DRONET_IN_W, DRONET_IN_H,
						     DRONET_GRAB_TIMEOUT_MS, &t_capture);
		if (frame < 0) {
			misses++;
			if ((misses % 10) == 1) {
				printk("DRONET: no camera frame (%d), %lu misses\n", (int)frame, misses);
			}
			k_msleep(50);   /* an immediate error must not spin at this priority */
			continue;
		}
		for (int i = 0; i < MODEL_DRONET_INPUT_SIZE; i++) {
			int q = (int)live_gray[i] - DRONET_IN_ZERO;
			live_in[i] = (model_dronet_input_t)(q < -128 ? -128 : (q > 127 ? 127 : q));
		}
		unsigned long wall_ticks = 0;
		unsigned long compute = dronet_run_once(live_in, 1, &wall_ticks);
		const int8_t steer_q = dronet_out[0];
		int8_t coll_q = dronet_out[1];
		frames++;
		int err = 0;
		const char *why = NULL;
		if (DRONET_DUAL_RUN) {
			unsigned long rt = 0;
			(void)dronet_run_once(live_in, 1, &rt);
			const int8_t coll2 = dronet_out[1];
			int ds = (int)dronet_out[0] - steer_q, dc = (int)coll2 - coll_q;
			err = MAX(ds < 0 ? -ds : ds, dc < 0 ? -dc : dc);
			why = err > DRONET_VERIFY_ATOL ? "dual-run mismatch" : NULL;
			/* Agreeing within the tolerance can still straddle the stop codes (127 or negative,
			 * see fc_dronet_nav.h): honour a stop from either run, else the larger collision. */
			const bool stop = coll_q < 0 || coll_q == INT8_MAX || coll2 < 0 || coll2 == INT8_MAX;
			coll_q = stop ? INT8_MAX : MAX(coll_q, coll2);
		}
		if (why == NULL) {
			dronet_cmd_publish(steer_q, coll_q, t_capture);
			if (DRONET_VERIFY_EVERY > 0 && (frames % DRONET_VERIFY_EVERY) == 0) {
				unsigned long vt = 0;
				(void)dronet_run_once(model_dronet_test_input, 1, &vt);
				err = dronet_verify();
				why = err > DRONET_VERIFY_ATOL ? "golden check" : NULL;
			}
		}
		if (why != NULL) {
			dronet_cmd_invalidate();
			printk("DRONET: %s err=%d under preemption -- commands WITHDRAWN and STOPPED\n",
			       why, err);
			k_sleep(K_FOREVER);
		}
		if ((frames % DRONET_REPORT_EVERY) == 0) {
			unsigned long wall_us = (hz != 0)
				? (unsigned long)(((uint64_t)wall_ticks * 1000000ULL) / hz) : 0;
			printk("DRONET: n=%lu frame=%d age=%dms compute=%lu kcyc wall=%lu.%03lu ms "
			       "out=[%d,%d] misses=%lu\n",
			       frames, (int)frame, (int)(k_uptime_get() - t_capture), compute / 1000UL,
			       wall_us / 1000UL, wall_us % 1000UL, (int)steer_q, (int)coll_q, misses);
		}
	}
#else
	unsigned long cnt = 0;
	for (;;) {
		unsigned long wall_ticks = 0;
		unsigned long compute = dronet_run_once(model_dronet_test_input, 1, &wall_ticks);
		cnt++;
		if ((cnt % DRONET_REPORT_EVERY) == 0) {
			int err = dronet_verify();
			unsigned long wall_us = (hz != 0)
				? (unsigned long)(((uint64_t)wall_ticks * 1000000ULL) / hz) : 0;
			unsigned long mfps = (wall_us != 0)
				? (unsigned long)(1000000000ULL / wall_us) : 0; /* fps*1000 */
			printk("DRONET: n=%lu compute=%lu kcyc wall=%lu.%03lu ms fps=%lu.%03lu err=%d out=[%d,%d]\n",
			       cnt, compute / 1000UL, wall_us / 1000UL, wall_us % 1000UL,
			       mfps / 1000UL, mfps % 1000UL, err, (int)dronet_out[0], (int)dronet_out[1]);
		}
	}
#endif
}

K_THREAD_DEFINE(dronet_tid, DRONET_STACK_SIZE, dronet_thread_fn,
		NULL, NULL, NULL, DRONET_PRIO, 0, DRONET_START_DELAY_MS);
