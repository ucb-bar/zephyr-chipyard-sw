/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE multizone-ToF data-collection probe. Validates that the guest can collect the four
 * horizontal VL53L5CX zone grids (front/right/back/left) over the RoSE bridge through the
 * REUSABLE ucbbar,rose-tof-zone driver + the standard Zephyr sensor API. Each control step it
 * batches all four sample_fetch (pipelined reqrsp issue) then all four channel_get, and prints
 * the per-direction nearest-wall distance (SENSOR_CHAN_DISTANCE = min zone). On real hardware
 * the same aliases bind to st,vl53l5cx unchanged.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <rose/rose_sensor.h>

#define ITERS 400

static const struct device *const tof_front = DEVICE_DT_GET(DT_ALIAS(tof_front));
static const struct device *const tof_right = DEVICE_DT_GET(DT_ALIAS(tof_right));
static const struct device *const tof_back  = DEVICE_DT_GET(DT_ALIAS(tof_back));
static const struct device *const tof_left  = DEVICE_DT_GET(DT_ALIAS(tof_left));

static float read_min(const struct device *dev)
{
	struct sensor_value v;

	if (sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &v) < 0) {
		return -1.0f;
	}
	return (float)sensor_value_to_double(&v);
}

int main(void)
{
	const struct device *devs[4] = { tof_front, tof_right, tof_back, tof_left };
	const char *names[4] = { "front", "right", "back", "left" };

	for (int i = 0; i < 4; i++) {
		if (!device_is_ready(devs[i])) {
			printk("rose_multisensor_probe: %s ToF not ready\n", names[i]);
			return -1;
		}
	}
	printk("rose_multisensor_probe: 4x VL53L5CX multizone ToF ready, collecting over RoSE\n");

	for (int it = 0; it < ITERS; it++) {
		/* Phase 1: issue all four zone-grid requests (pipelined). A decimated sensor
		 * returns -EAGAIN on non-refresh steps -> reuse its last grid. */
		bool fresh[4];
		for (int i = 0; i < 4; i++) {
			fresh[i] = (sensor_sample_fetch(devs[i]) == 0);
		}
		/* Phase 2: collect (blocking read on the first get) -> nearest wall per direction. */
		float d[4];
		for (int i = 0; i < 4; i++) {
			d[i] = read_min(devs[i]);
		}
		if (it % 20 == 0) {
			/* millimetres as integers (no float printf needed) */
			printk("iter=%3d  front=%4dmm  right=%4dmm  back=%4dmm  left=%4dmm  (fresh %d%d%d%d)\n",
			       it, (int)(d[0] * 1000), (int)(d[1] * 1000), (int)(d[2] * 1000),
			       (int)(d[3] * 1000), fresh[0], fresh[1], fresh[2], fresh[3]);
		}
	}
	printk("rose_multisensor_probe: done (%d steps)\n", ITERS);
	return 0;
}
