/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE bridge: high-level transport/protocol layer. Uses rose_request +
 * rose_recv_reqrsp, which strips the [header, num_bytes] framing so the caller
 * gets the served data words directly, then validates the known pattern.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <rose/rose_proto.h>
#include "rose_check.h"

int main(void)
{
	const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
	uint32_t buf[ROSE_PATTERN_LEN];
	int n;

	if (!device_is_ready(rose)) {
		printk("ROSE protocol: FAIL (device not ready)\n");
		return 1;
	}

	rose_request(rose, ROSE_TEST_CMD_REQRSP, 0);
	n = rose_recv_reqrsp(rose, ROSE_TEST_REQRSP_CH, buf, ROSE_PATTERN_LEN);

	return rose_check_pattern("protocol", buf, n, ROSE_PATTERN_BASE) ? 0 : 1;
}
