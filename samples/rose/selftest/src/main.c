/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE bridge combined self-test: exercises both RX paths in one binary for
 * one-shot FPGA validation.
 *   1) DMA path   (ch0, interrupt-driven): rose_dma_arm + request + rose_dma_wait
 *   2) reqrsp path(ch2, protocol layer):   rose_request + rose_recv_reqrsp
 * Prints per-path markers plus a final summary.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <rose/rose.h>
#include <rose/rose_proto.h>
#include "rose_check.h"

#define BUF_BYTES (ROSE_PATTERN_LEN * 4)

static int test_dma(const struct device *rose)
{
	volatile uint32_t *dma;
	uint32_t buf[ROSE_PATTERN_LEN];

	rose_dma_arm(rose, ROSE_TEST_DMA_CH, BUF_BYTES);
	rose_tx(rose, ROSE_TEST_CMD_DMA);
	rose_tx(rose, 0);
	if (rose_dma_wait(rose, ROSE_TEST_DMA_CH, K_FOREVER) != 0) {
		printk("ROSE dma: FAIL (dma_wait)\n");
		return 0;
	}
	dma = (volatile uint32_t *)rose_dma_buffer(rose, ROSE_TEST_DMA_CH);
	for (int i = 0; i < ROSE_PATTERN_LEN; i++) {
		buf[i] = dma[i];
	}
	return rose_check_pattern("dma", buf, ROSE_PATTERN_LEN, ROSE_PATTERN_BASE);
}

static int test_reqrsp(const struct device *rose)
{
	uint32_t buf[ROSE_PATTERN_LEN];
	int n;

	rose_request(rose, ROSE_TEST_CMD_REQRSP, 0);
	n = rose_recv_reqrsp(rose, ROSE_TEST_REQRSP_CH, buf, ROSE_PATTERN_LEN);
	return rose_check_pattern("reqrsp", buf, n, ROSE_PATTERN_BASE);
}

int main(void)
{
	const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
	int dma_ok, reqrsp_ok;

	if (!device_is_ready(rose)) {
		printk("ROSE selftest: FAIL (device not ready)\n");
		return 1;
	}

	dma_ok = test_dma(rose);
	reqrsp_ok = test_reqrsp(rose);

	printk("ROSE selftest: dma=%s reqrsp=%s => %s\n",
	       dma_ok ? "PASS" : "FAIL",
	       reqrsp_ok ? "PASS" : "FAIL",
	       (dma_ok && reqrsp_ok) ? "PASS" : "FAIL");

	return (dma_ok && reqrsp_ok) ? 0 : 1;
}
