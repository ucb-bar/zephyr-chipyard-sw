/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * uxrCustomTransport callbacks backed by Zephyr message queues, so
 * micro-ROS rmw_microxrcedds talks to a target-resident broker thread
 * instead of a real serial-connected agent.
 *
 * Multi-session: up to LOOPBACK_MAX_SESSIONS independent client sessions.
 * Each session has its own pair of k_msgq's (rmw -> broker, broker -> rmw)
 * so two micro-ROS executor threads pinned to different harts can each
 * own their own session without sharing rmw state. The session index is
 * carried through the upstream API as transport->args:
 *
 *   rmw_uros_set_custom_transport(false,
 *                                 (void *)(intptr_t)session_idx,
 *                                 loopback_open, loopback_close,
 *                                 loopback_write, loopback_read);
 *
 * Each callback recovers the index from transport->args and routes to
 * that session's queue pair. Set framing=false on
 * rmw_uros_set_custom_transport — k_msgq preserves message boundaries
 * natively so micro-XRCE-DDS's stream framing wrapper isn't needed.
 */
#ifndef MICRO_ROS_LOCAL_TRANSPORT_LOOPBACK_H
#define MICRO_ROS_LOCAL_TRANSPORT_LOOPBACK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#define LOOPBACK_MAX_SESSIONS 4

/* MTU must match -DUCLIENT_CUSTOM_TRANSPORT_MTU in colcon.meta (512). */
#define LOOPBACK_MTU 512

struct loopback_slot {
	uint16_t len;
	uint8_t  data[LOOPBACK_MTU];
};

struct uxrCustomTransport;  /* forward */

/* Four uxrCustomTransport hooks. transport->args is interpreted as a
 * (intptr_t)-cast session index in [0, LOOPBACK_MAX_SESSIONS). */
bool   loopback_open(struct uxrCustomTransport *t);
bool   loopback_close(struct uxrCustomTransport *t);
size_t loopback_write(struct uxrCustomTransport *t,
		      const uint8_t *buf, size_t len, uint8_t *err);
size_t loopback_read(struct uxrCustomTransport *t,
		     uint8_t *buf, size_t len, int timeout_ms, uint8_t *err);

/* ----- Broker-side accessors -------------------------------------------
 *
 * The broker drains rmw->broker traffic from any session via the "any"
 * variant (uses k_poll under the hood; needs CONFIG_POLL=y). It sends
 * replies to a specific session's broker->rmw queue.
 */

/* Wait until a message is available on ANY session's rmw->broker queue,
 * deliver it into *out, and write the originating session index into
 * *out_session_idx. Returns 0 on success, -EAGAIN on timeout, -errno
 * otherwise. K_FOREVER blocks indefinitely. */
int loopback_broker_recv_any(struct loopback_slot *out, int *out_session_idx,
			     k_timeout_t to);

/* Push a reply onto a specific session's broker->rmw queue. */
int loopback_broker_send(int session_idx, const struct loopback_slot *in);

#endif
