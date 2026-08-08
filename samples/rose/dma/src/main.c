/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE bridge: interrupt-driven camera-DMA RX. Arms the DMA channel, requests a
 * frame, blocks on the DMA-complete PLIC interrupt (no busy-poll), then validates
 * the buffer the CamDMAEngine wrote to memory.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <rose/rose.h>
#include "rose_check.h"

#define BUF_BYTES (ROSE_PATTERN_LEN * 4)

int main(void)
{
	const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
	volatile uint32_t *dma;
	uint32_t buf[ROSE_PATTERN_LEN];
	int rc;

	if (!device_is_ready(rose)) {
		printk("ROSE dma: FAIL (device not ready)\n");
		return 1;
	}

	rose_dma_arm(rose, ROSE_TEST_DMA_CH, BUF_BYTES);
	rose_tx(rose, ROSE_TEST_CMD_DMA);
	rose_tx(rose, 0);

	rc = rose_dma_wait(rose, ROSE_TEST_DMA_CH, K_FOREVER);
	if (rc != 0) {
		printk("ROSE dma: FAIL (dma_wait rc=%d)\n", rc);
		return 1;
	}

	dma = (volatile uint32_t *)rose_dma_buffer(rose, ROSE_TEST_DMA_CH);
	for (int i = 0; i < ROSE_PATTERN_LEN; i++) {
		buf[i] = dma[i];
	}

	return rose_check_pattern("dma", buf, ROSE_PATTERN_LEN, ROSE_PATTERN_BASE) ? 0 : 1;
}
