/*
 * zephyr_gemmini_canonical_test
 *
 * Calls gemmini-rocc-tests's tiled_matmul_auto directly from a Zephyr
 * thread to determine whether LOOP_WS opcodes work under Zephyr's
 * runtime context. Mirrors bareMetalC/tiled_matmul_ws.c, just wrapped
 * in a Zephyr main(). No IREE.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <stdint.h>

/* Pull in canonical gemmini headers + helpers. */
#define BAREMETAL 1
#include "include/gemmini_testutils.h"

#define MAT_DIM_I 64
#define MAT_DIM_J 64
#define MAT_DIM_K 64
#define NO_BIAS 1

/* Flush print buffer by writing 256 bytes of padding (forces auto-flush at
 * CONFIG_UART_HTIF_BUFFERED_OUTPUT_SIZE=256). */
static void force_flush(void)
{
	char pad[256];
	for (int i = 0; i < 256; ++i) pad[i] = '.';
	pad[255] = '\0';
	printk("%s\n", pad);
	k_busy_wait(10);
}

static elem_t A[MAT_DIM_I][MAT_DIM_K] row_align(1);
static elem_t B[MAT_DIM_K][MAT_DIM_J] row_align(1);
static acc_t  D[MAT_DIM_I][MAT_DIM_J] row_align_acc(1);
static elem_t C[MAT_DIM_I][MAT_DIM_J] row_align(1);

int main(void)
{
	printk("=== zephyr_gemmini_canonical_test ===\n");
	printk("DIM_I=%d DIM_J=%d DIM_K=%d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
	force_flush();

	/* Manually set mstatus.XS=Dirty to ensure custom-3 (RoCC) traps don't fire.
	 * Zephyr's reset.S sets FS and VS but NOT XS. The bare-metal
	 * gemmini-rocc-tests sets all three via lui 0x1e + csrs.
	 */
	uintptr_t mstatus_set = (3UL << 13) | (3UL << 15); /* FS=Dirty | XS=Dirty */
	__asm__ volatile ("csrs mstatus, %0" : : "r"(mstatus_set));
	printk("[setup] set mstatus FS|XS to Dirty\n");
	force_flush();

	/* Init A and B with small constants to keep results predictable. */
	for (int i = 0; i < MAT_DIM_I; ++i)
		for (int j = 0; j < MAT_DIM_K; ++j)
			A[i][j] = 1;
	for (int i = 0; i < MAT_DIM_K; ++i)
		for (int j = 0; j < MAT_DIM_J; ++j)
			B[i][j] = 1;

	/* Disable interrupts around the matmul so timer ticks can't preempt
	 * a multi-cycle LOOP_WS instruction. Hypothesis: Zephyr's trap handler
	 * doesn't save/restore Gemmini RoCC state, so an interrupt mid-LOOP_WS
	 * leaves config registers in an undefined state on return.
	 */
	unsigned int key = irq_lock();
	printk("[setup] irq_lock acquired\n");
	force_flush();

	/* Initialize Gemmini config (canonical-style) before tiled_matmul_auto. */
	gemmini_flush(0);
	printk("[setup] gemmini_flush(0) returned\n");
	force_flush();

	uint64_t start = read_cycles();
	tiled_matmul_auto(MAT_DIM_I, MAT_DIM_J, MAT_DIM_K,
		(elem_t *)A, (elem_t *)B, NO_BIAS ? NULL : (acc_t *)D, (elem_t *)C,
		MAT_DIM_K, MAT_DIM_J, MAT_DIM_J, MAT_DIM_J,
		MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
		NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, false,
		false, false,
		false, /*low_D=*/ true,
		0,
		WS);
	uint64_t end = read_cycles();
	uint64_t cycles = end - start;

	irq_unlock(key);
	printk("[setup] irq_unlock done\n");
	force_flush();

	/* Verify all output cells == MAT_DIM_K (sum of 1*1 over K dim). */
	int wrong = 0;
	for (int i = 0; i < MAT_DIM_I; ++i)
		for (int j = 0; j < MAT_DIM_J; ++j)
			if ((int)C[i][j] != MAT_DIM_K)
				++wrong;

	printk("[result] cycles=%llu wrong_cells=%d\n",
		(unsigned long long)cycles, wrong);
	if (wrong == 0)
		printk("[result] PASS\n");
	else
		printk("[result] FAIL\n");

	return 0;
}
