/*
 * DroNet background inference thread for the integrated FC + DroNet build on the
 * riskybird v3 FPGA Rocket combined core RocketArty200TDroneGemminiSaturnFp16At35Config
 * (Gemmini + Saturn V128D64 + the full drone periphery, misa.F=0 / vector-fp16 only).
 *
 * WHAT THIS IS
 *   The flight controller (samples/rose_flight_controller) runs its PID control
 *   loop in main() at the highest *preemptible* priority (Zephyr prio 0). This
 *   file adds a SEPARATE, LOW-priority thread that runs DroNet inference over a
 *   static in-memory input tensor as fast as the shared hart allows, WITHOUT
 *   holding the CPU away from the PID loop: the PID loop preempts this thread
 *   whenever its k_usleep(300) pace expires (preemptive scheduling), and this
 *   thread additionally k_yield()s at every op boundary so the sensor/camera
 *   threads also get their I2C-wait gaps.
 *
 *   Phase 1 (this file): the input is the model's baked, deterministic
 *   112x112x1 grayscale test tensor (model_dronet_test_input), so each inference
 *   can be verified bit-exact against model_dronet_test_golden IN-BINARY
 *   (max_abs_err must stay <= 3, the fast-conv numeric envelope). Phase 2 swaps
 *   this static tensor for a preprocessed OSPI camera frame (see camera_dma.c).
 *
 * PREEMPTION SAFETY (the real risk) -- see the integration report / memory:
 *   DroNet is the ONLY code in this binary that issues RISC-V Vector (Saturn RVV)
 *   instructions: V codegen is confined to the generated kernels.c translation
 *   unit via its per-source `-march=rv64imac_zve64x` override, while every other
 *   TU (this file, the whole FC, Zephyr, libc) is built for `rv64imac_zicsr_...`
 *   (no V) -- i.e. CONFIG_RISCV_V_KERNEL_ONLY=y. Correct preemption of a
 *   mid-vector kernel therefore requires Zephyr's EAGER vector context switch:
 *   CONFIG_RISCV_ISA_EXT_V=y + CONFIG_RISCV_ISA_EXT_V_LAZY=n (set in
 *   boards/chipyard_riscv64.conf). With that, a trap taken while this thread is
 *   inside a vector kernel saves/restores the full vector register file (and
 *   vstart) across the switch, and the PID loop -- which touches no V state --
 *   never perturbs it.
 *
 *   Gemmini ops are custom-RoCC (not vector) and fire-and-forget + fenced inside
 *   the kernel; the PID thread never touches Gemmini, so the accelerator state is
 *   safe as long as the RVV context switch is correct.
 *
 *   FALLBACK: if bench testing shows the At35 Saturn corrupts state on a mid-V
 *   preemption (as an earlier multi-hart FireSim Saturn config did), build with
 *   -DDRONET_IRQ_GATE=1. That masks IRQs around each *individual* dispatch so no
 *   op is ever preempted mid-vector. NOTE this raises worst-case PID latency to
 *   the longest single op's runtime, so it is a correctness fallback, not the
 *   >=200 Hz design point -- the big Gemmini conv (RoCC + scalar fence, no RVV in
 *   flight) is safe to preempt anyway, so a finer gate that locks only the short
 *   RVV ops (bn/add/relu/pool/relayout/linear) keeps both properties.
 */

#include <zephyr/kernel.h>
#include <stdio.h>

#include "model.h"
#include "test_io.h"   /* model_dronet_test_input[], model_dronet_test_golden[] */

#ifndef DRONET_PRIO
/* Below the PID loop (main, prio 0), the down-ToF thread (8) and the camera DMA
 * thread (12): DroNet only runs in the slack the control loop leaves. */
#define DRONET_PRIO 13
#endif

#ifndef DRONET_STACK_SIZE
/* The Gemmini/Saturn hetero kernels keep their large intermediates in file-static
 * buffers (buffers.c), so the per-op stack need is modest; 256 KiB is generous
 * headroom on the 1 GiB DDR. */
#define DRONET_STACK_SIZE (256 * 1024)
#endif

#ifndef DRONET_START_DELAY_MS
/* Let the FC finish sensor init + gyro-bias calibration before we start stealing
 * hart cycles, so boot cal is clean. */
#define DRONET_START_DELAY_MS 4000
#endif

#ifndef DRONET_IRQ_GATE
#define DRONET_IRQ_GATE 0
#endif

#ifndef DRONET_REPORT_EVERY
#define DRONET_REPORT_EVERY 10   /* print a summary line every N inferences */
#endif

static model_dronet_output_t dronet_out[MODEL_DRONET_OUTPUT_SIZE];

/* One inference over the static test tensor, driving the generated dispatch
 * table op-by-op so we can insert a preemption point (k_yield) -- and optionally
 * an IRQ gate -- at every op boundary. This mirrors run_model_dronet()'s
 * straight-line walk but under our own preemption policy. Returns the summed
 * per-op compute cycles (rdcycle, core clock); *wall_ticks gets the k_cycle_get
 * (mtime) delta so the caller can see wall time including any PID preemption. */
static unsigned long dronet_run_once(unsigned long *wall_ticks)
{
	model_dronet_state_t s = {
		.input  = model_dronet_test_input,
		.output = dronet_out,
		.pool   = NULL,
	};

	model_dronet_reset_profile();
	unsigned long t0 = (unsigned long)k_cycle_get_64();

	for (int i = 0; i < MODEL_DRONET_OP_COUNT; i++) {
#if DRONET_IRQ_GATE
		unsigned int key = irq_lock();
		MODEL_DRONET_DISPATCH_FNS[i](&s);
		irq_unlock(key);
#else
		MODEL_DRONET_DISPATCH_FNS[i](&s);
#endif
		/* Op-boundary preemption point: let the PID loop / sensor threads
		 * run if they are ready. Preemption also happens on the tick even
		 * without this, but yielding here hands off promptly. */
		k_yield();
	}

	/* RVV -> scalar visibility barrier: the kernels store the output via
	 * vector stores; the scalar verify below must see the drained buffer. */
	__asm__ volatile("fence rw, rw" ::: "memory");

	*wall_ticks = (unsigned long)k_cycle_get_64() - t0;

	/* Sum the per-op compute cycles (rdcycle deltas) for a contention-free
	 * "how much did the hart actually spend in DroNet" number. */
	int n = 0;
	const model_dronet_op_record_t *rec = model_dronet_profile_records(&n);
	unsigned long compute = 0;
	for (int i = 0; i < n; i++) {
		compute += rec[i].cycles;
	}
	return compute;
}

static void dronet_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	const uint32_t hz = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC; /* mtime tick rate */
	unsigned long n = 0;

	printk("DRONET: thread up (prio %d, gate=%d) model=%s in=%d out=%d ops=%d\n",
	       DRONET_PRIO, DRONET_IRQ_GATE, MODEL_DRONET_NAME,
	       MODEL_DRONET_INPUT_SIZE, MODEL_DRONET_OUTPUT_SIZE, MODEL_DRONET_OP_COUNT);

	for (;;) {
		unsigned long wall_ticks = 0;
		unsigned long compute = dronet_run_once(&wall_ticks);
		n++;

		if ((n % DRONET_REPORT_EVERY) == 0) {
			/* Bit-exact check vs the baked golden (int8 output). */
			int max_abs_err = 0;
			for (int i = 0; i < MODEL_DRONET_TEST_OUTPUT_LEN; i++) {
				int d = (int)dronet_out[i] - (int)model_dronet_test_golden[i];
				if (d < 0) d = -d;
				if (d > max_abs_err) max_abs_err = d;
			}
			/* wall ms and fps from the mtime delta (includes PID steals);
			 * compute kcyc is contention-free core cycles. Use integer
			 * math x1000 for milli-fps to avoid soft-float in the hot path. */
			unsigned long wall_us = (hz != 0)
				? (unsigned long)(((uint64_t)wall_ticks * 1000000ULL) / hz)
				: 0;
			unsigned long mfps = (wall_us != 0)
				? (unsigned long)(1000000000ULL / wall_us) : 0; /* fps*1000 */

			printk("DRONET: n=%lu compute=%lu kcyc wall=%lu.%03lu ms "
			       "fps=%lu.%03lu err=%d out=[%d,%d]\n",
			       n, compute / 1000UL,
			       wall_us / 1000UL, wall_us % 1000UL,
			       mfps / 1000UL, mfps % 1000UL,
			       max_abs_err,
			       (int)dronet_out[0], (int)dronet_out[1]);
		}
	}
}

K_THREAD_DEFINE(dronet_tid, DRONET_STACK_SIZE, dronet_thread_fn,
		NULL, NULL, NULL, DRONET_PRIO, 0, DRONET_START_DELAY_MS);
