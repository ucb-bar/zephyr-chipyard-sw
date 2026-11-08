/*
 * Zvt tile context-switch test. See README.rst.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/printk.h>
#include <zephyr/arch/riscv/zvt.h>

#define MAXWORDS  (Z_RISCV_ZVT_SAVE_SIZE / 4)
#define NTHREADS  3
#define ROUNDS    6
#define STACK     4096

K_THREAD_STACK_ARRAY_DEFINE(stacks, NTHREADS + 1, STACK);
static struct k_thread threads[NTHREADS + 1];

static Z_RISCV_ZVT_SAVE_AREA_DEFINE(save_area[NTHREADS]);
static uint32_t pattern[NTHREADS][MAXWORDS] __aligned(64);
static uint32_t readback[NTHREADS][MAXWORDS] __aligned(64);
static unsigned int TE, COLS, WORDS;     /* runtime geometry */
static volatile unsigned int migrations;

static volatile int last_runner = -1;
static volatile unsigned int interleavings;
static volatile int errors;
static volatile bool fault_seen;
static volatile int faulting_tid = -1;

/* Full-tile transfer in the e32 view: 4 tile groups x TE rows. */
static void tiles_xfer(uint32_t *buf, bool store)
{
	zvt_msetmtype(ZVT_MTYPE(1, 0, COLS), ZVT_VTYPE_E32);
	if (zvt_msettn(COLS) != COLS) {
		printk("tn != COLS\n");
		errors++;
		return;
	}
	for (unsigned int g = 0; g < 4; g++) {
		for (unsigned int r = 0; r < TE; r++) {
			uint32_t *row = buf + ((g * TE) + r) * COLS;

			if (store) {
				zvt_vtse32(row, ZVT_TSS(g * 4, 0, r));
			} else {
				zvt_vtle32(row, ZVT_TSS(g * 4, 0, r));
			}
		}
	}
}

/* Spin long enough to be time-sliced, recording whether we were. */
static void spin(int tid)
{
	unsigned int cpu = arch_curr_cpu()->id;

	for (volatile int i = 0; i < 400000; i++) {
		if (arch_curr_cpu()->id != cpu) {
			cpu = arch_curr_cpu()->id;
			migrations++;
		}
		if (last_runner != tid) {
			if (last_runner >= 0) {
				interleavings++;
			}
			last_runner = tid;
		}
	}
}

static void tile_thread(void *p1, void *p2, void *p3)
{
	int tid = (int)(uintptr_t)p1;
	int rc = arch_riscv_zvt_enable(save_area[tid], sizeof(save_area[tid]));

	if (WORDS == 0) {
		WORDS = (unsigned int)(arch_riscv_zvt_save_size() / 4);
	}

	if (rc != 0) {
		printk("T%d: arch_riscv_zvt_enable failed: %d\n", tid, rc);
		errors++;
		return;
	}

	for (int round = 0; round < ROUNDS; round++) {
		for (unsigned int i = 0; i < WORDS; i++) {
			pattern[tid][i] = (tid + 1) * 0x01000193u ^ (i * 2654435761u) ^ round;
		}
		tiles_xfer(pattern[tid], false);

		/* A thread-specific tile config that must survive switches. */
		unsigned long tm = 2 + tid, tn = 2 + 2 * tid; /* <= COLS for TE >= 8 */
		unsigned long want_mtype = ZVT_MTYPE(1, 0, tm);

		zvt_msetmtype(want_mtype, ZVT_VTYPE_E32);
		unsigned long got_tn = zvt_msettn(tn);

		spin(tid);
		if (round & 1) {
			k_yield();       /* cooperative switch path too */
		}

		unsigned long mtype = zvt_read_mtype();
		unsigned long vl;

		__asm__ volatile("csrr %0, vl" : "=r"(vl));
		if (mtype != want_mtype || vl != got_tn) {
			printk("T%d r%d: config lost: mtype %lx (want %lx) vl %lu (want %lu)\n",
			       tid, round, mtype, want_mtype, vl, got_tn);
			errors++;
		}

		tiles_xfer(readback[tid], true);
		for (unsigned int i = 0; i < WORDS; i++) {
			if (readback[tid][i] != pattern[tid][i]) {
				printk("T%d r%d: tile word %u = %08x, want %08x\n", tid, round,
				       i, readback[tid][i], pattern[tid][i]);
				errors++;
				break;
			}
		}
	}
}

/* Never opts in: its first tile instruction must fault (MS = Off). */
static void no_tile_thread(void *p1, void *p2, void *p3)
{
	faulting_tid = (int)(uintptr_t)p1;
	ZVT_VTZERO(0);
	printk("no-tile thread executed vtzero without faulting\n");
	errors++;
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);
	if (k_current_get() == &threads[NTHREADS] && reason == K_ERR_CPU_EXCEPTION) {
		fault_seen = true;
		return;          /* abort just that thread */
	}
	printk("unexpected fatal error %u\n", reason);
	k_fatal_halt(reason);
}

int main(void)
{
	if (arch_riscv_zvt_geometry(&TE, &COLS) != 0) {
		printk("ZVT_CONTEXT: no usable Zvt\nZVT_CONTEXT: FAIL\n");
		return 0;
	}
	WORDS = 4 * TE * COLS;
	printk("ZVT_CONTEXT: TE=%u cols=%u save=%u B, %s, %u CPU(s), %d threads x %d rounds\n",
	       TE, COLS, WORDS * 4, IS_ENABLED(CONFIG_RISCV_ZVT_LAZY) ? "lazy" : "eager",
	       arch_num_cpus(), NTHREADS, ROUNDS);

	for (int t = 0; t < NTHREADS; t++) {
		k_thread_create(&threads[t], stacks[t], STACK, tile_thread,
				(void *)(uintptr_t)t, NULL, NULL, 5, K_FP_REGS, K_NO_WAIT);
	}
	printk("(an illegal-instruction dump from the no-tile thread is expected)\n");
	k_thread_create(&threads[NTHREADS], stacks[NTHREADS], STACK, no_tile_thread,
			(void *)(uintptr_t)NTHREADS, NULL, NULL, 5, K_FP_REGS, K_NO_WAIT);

	for (int t = 0; t <= NTHREADS; t++) {
		k_thread_join(&threads[t], K_FOREVER);
	}

	printk("interleavings=%u migrations=%u fault_seen=%d errors=%d\n", interleavings,
	       migrations, fault_seen, errors);
	printk("tile stats: nested_traps=%u lazy_traps=%u stores=%u loads=%u\n",
	       z_riscv_zvt_stats.nested, z_riscv_zvt_stats.lazy_traps, z_riscv_zvt_stats.stores,
	       z_riscv_zvt_stats.loads);
	if (!fault_seen) {
		errors++;
	}
	if (interleavings < NTHREADS) {
		printk("too few preemptions to be a meaningful test\n");
		errors++;
	}
	printk("ZVT_CONTEXT: %s\n", errors ? "FAIL" : "PASS");
	return 0;
}
