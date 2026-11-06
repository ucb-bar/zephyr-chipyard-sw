/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared self-check helpers for the samples/rose RoSE-bridge tests. Each test
 * receives the known PatternEnv sequence (PATTERN_BASE + 0,1,2,...) over the RoSE
 * bridge and validates every word, emitting a single machine-greppable result
 * marker so it can be checked from the (F)PGA/metasim uartlog.
 */

#ifndef ROSE_CHECK_H_
#define ROSE_CHECK_H_

#include <stdint.h>
#include <zephyr/kernel.h>

/* PatternEnv serves this known ramp (16 uint32 words). */
#define ROSE_PATTERN_BASE 0xC0DE0000u
#define ROSE_PATTERN_LEN  16

/* Test command -> channel mapping (see deploy/config/config_gym_PatternEnv-v0.yaml). */
#define ROSE_TEST_CMD_DMA    0x11u /* -> channel 0 (DMA0) */
#define ROSE_TEST_CMD_REQRSP 0x12u /* -> channel 2 (reqrsp1) */
#define ROSE_TEST_REQRSP_CH  2
#define ROSE_TEST_DMA_CH     0

/**
 * @brief Validate @p n words of @p buf against the known ramp base+0,1,2,...
 *        and print a "ROSE <name>: PASS/FAIL ..." marker.
 * @return 1 on PASS, 0 on FAIL.
 */
static inline int rose_check_pattern(const char *name, const uint32_t *buf, int n, uint32_t base)
{
	int ok = 1;
	int first_bad = -1;

	if (n <= 0) {
		printk("ROSE %s: FAIL (no words received)\n", name);
		return 0;
	}
	for (int i = 0; i < n; i++) {
		if (buf[i] != base + (uint32_t)i) {
			ok = 0;
			if (first_bad < 0) {
				first_bad = i;
			}
		}
	}
	if (ok) {
		printk("ROSE %s: PASS (%d words, [0]=0x%08x [%d]=0x%08x)\n",
		       name, n, buf[0], n - 1, buf[n - 1]);
	} else {
		printk("ROSE %s: FAIL at word %d (got 0x%08x, want 0x%08x)\n",
		       name, first_bad, buf[first_bad], base + (uint32_t)first_bad);
	}
	return ok;
}

#endif /* ROSE_CHECK_H_ */
