/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE bridge: low-level reqrsp RX. Uses the raw driver API (rose_tx / rose_rx)
 * to request data and read the framed [header, num_bytes, data...] response off
 * reqrsp channel 2, then validates the known pattern.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <rose/rose.h>
#include "rose_check.h"

int main(void)
{
	const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
	uint32_t header, num_bytes;
	uint32_t buf[ROSE_PATTERN_LEN];
	int n;

	if (!device_is_ready(rose)) {
		printk("ROSE reqrsp: FAIL (device not ready)\n");
		return 1;
	}

	/* request over TX (cmd + num_bytes arg) */
	rose_tx(rose, ROSE_TEST_CMD_REQRSP);
	rose_tx(rose, 0);

	/* framed response on channel 2: header (cmd echo), num_bytes, then data */
	rose_rx(rose, ROSE_TEST_REQRSP_CH, &header);
	rose_rx(rose, ROSE_TEST_REQRSP_CH, &num_bytes);
	n = (int)(num_bytes / sizeof(uint32_t));
	if (n > ROSE_PATTERN_LEN) {
		n = ROSE_PATTERN_LEN;
	}
	for (int i = 0; i < n; i++) {
		rose_rx(rose, ROSE_TEST_REQRSP_CH, &buf[i]);
	}

	return rose_check_pattern("reqrsp", buf, n, ROSE_PATTERN_BASE) ? 0 : 1;
}
