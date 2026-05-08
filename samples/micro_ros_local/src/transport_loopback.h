/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Custom-transport callbacks backed by two Zephyr message queues (rmw -> broker
 * and broker -> rmw), so micro-ROS's rmw_microxrcedds layer talks to a
 * target-resident broker thread instead of a real serial-connected agent.
 *
 * Set framing=false on rmw_uros_set_custom_transport() — the message queue
 * preserves message boundaries natively, so we don't need micro-XRCE-DDS's
 * stream-framing wrapper.
 */
#ifndef MICRO_ROS_LOCAL_TRANSPORT_LOOPBACK_H
#define MICRO_ROS_LOCAL_TRANSPORT_LOOPBACK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>

/* MTU must match -DUCLIENT_CUSTOM_TRANSPORT_MTU in colcon.meta (512). */
#define LOOPBACK_MTU 512

/* One queued message + length prefix. */
struct loopback_slot {
	uint16_t len;
	uint8_t  data[LOOPBACK_MTU];
};

struct uxrCustomTransport;  /* forward */

/* Four custom transport hooks, register via rmw_uros_set_custom_transport(). */
bool   loopback_open(struct uxrCustomTransport *t);
bool   loopback_close(struct uxrCustomTransport *t);
size_t loopback_write(struct uxrCustomTransport *t,
		      const uint8_t *buf, size_t len, uint8_t *err);
size_t loopback_read(struct uxrCustomTransport *t,
		     uint8_t *buf, size_t len, int timeout_ms, uint8_t *err);

/* Broker-side accessors — used by broker.c only. */
int loopback_broker_recv(struct loopback_slot *out, k_timeout_t to);
int loopback_broker_send(const struct loopback_slot *in);

#endif
