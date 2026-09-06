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
static unsigned long dronet_run_once(int steady, unsigned long *wall_ticks)
{
	model_dronet_state_t s = {
		.input = model_dronet_test_input, .output = dronet_out, .pool = NULL,
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
			k_yield();
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
	unsigned long compute0 = dronet_run_once(0, &wt);
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

	unsigned long cnt = 0;
	for (;;) {
		unsigned long wall_ticks = 0;
		unsigned long compute = dronet_run_once(1, &wall_ticks);
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
}

K_THREAD_DEFINE(dronet_tid, DRONET_STACK_SIZE, dronet_thread_fn,
		NULL, NULL, NULL, DRONET_PRIO, 0, DRONET_START_DELAY_MS);
