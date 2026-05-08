/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Loopback transport implementation. See transport_loopback.h.
 */

#include <microros_local/transport_loopback.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <zephyr/sys/printk.h>
#include <errno.h>
#include <string.h>

/* Per-session in-flight depth. Tune up if rmw bursts more than this. */
#define LOOPBACK_DEPTH 8

static inline int session_index_from(struct uxrCustomTransport *t)
{
	if (!t) {
		return -1;
	}
	intptr_t v = (intptr_t)t->args;
	if (v < 0 || v >= LOOPBACK_MAX_SESSIONS) {
		return -1;
	}
	return (int)v;
}

/* --- Per-session queue pairs ------------------------------------------ */

#define DEFINE_PAIR(idx)                                                       \
	K_MSGQ_DEFINE(rmw_to_broker_##idx,                                     \
		      sizeof(struct loopback_slot), LOOPBACK_DEPTH, 4);        \
	K_MSGQ_DEFINE(broker_to_rmw_##idx,                                     \
		      sizeof(struct loopback_slot), LOOPBACK_DEPTH, 4)

DEFINE_PAIR(0);
DEFINE_PAIR(1);
DEFINE_PAIR(2);
DEFINE_PAIR(3);
BUILD_ASSERT(LOOPBACK_MAX_SESSIONS == 4,
	     "transport_loopback: bump DEFINE_PAIR list to match LOOPBACK_MAX_SESSIONS");

static struct k_msgq *const rmw_to_broker[LOOPBACK_MAX_SESSIONS] = {
	&rmw_to_broker_0, &rmw_to_broker_1, &rmw_to_broker_2, &rmw_to_broker_3,
};
static struct k_msgq *const broker_to_rmw[LOOPBACK_MAX_SESSIONS] = {
	&broker_to_rmw_0, &broker_to_rmw_1, &broker_to_rmw_2, &broker_to_rmw_3,
};

/* --- Four uxrCustomTransport hooks ------------------------------------ */

bool loopback_open(struct uxrCustomTransport *t)
{
	int idx = session_index_from(t);
	if (idx < 0) {
		printk("loopback_open: bad session idx\n");
		return false;
	}
	k_msgq_purge(rmw_to_broker[idx]);
	k_msgq_purge(broker_to_rmw[idx]);
	return true;
}

bool loopback_close(struct uxrCustomTransport *t)
{
	(void)t;
	return true;
}

size_t loopback_write(struct uxrCustomTransport *t,
		      const uint8_t *buf, size_t len, uint8_t *err)
{
	(void)err;
	int idx = session_index_from(t);
	if (idx < 0 || len > LOOPBACK_MTU) {
		return 0;
	}
	struct loopback_slot s;
	s.len = (uint16_t)len;
	memcpy(s.data, buf, len);
	if (k_msgq_put(rmw_to_broker[idx], &s, K_MSEC(50)) != 0) {
		printk("loopback_write[s%d]: queue full, dropped %u B\n",
		       idx, (unsigned)len);
		return 0;
	}
	return len;
}

size_t loopback_read(struct uxrCustomTransport *t,
		     uint8_t *buf, size_t len, int timeout_ms, uint8_t *err)
{
	(void)err;
	int idx = session_index_from(t);
	if (idx < 0) {
		return 0;
	}
	struct loopback_slot s;
	if (k_msgq_get(broker_to_rmw[idx], &s, K_MSEC(timeout_ms)) != 0) {
		return 0;
	}
	size_t n = MIN((size_t)s.len, len);
	memcpy(buf, s.data, n);
	return n;
}

/* --- Broker-side accessors -------------------------------------------- */

int loopback_broker_recv_any(struct loopback_slot *out, int *out_session_idx,
			     k_timeout_t to)
{
	struct k_poll_event ev[LOOPBACK_MAX_SESSIONS];
	for (int i = 0; i < LOOPBACK_MAX_SESSIONS; i++) {
		k_poll_event_init(&ev[i],
				  K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
				  K_POLL_MODE_NOTIFY_ONLY,
				  rmw_to_broker[i]);
	}

	int rc = k_poll(ev, LOOPBACK_MAX_SESSIONS, to);
	if (rc != 0) {
		return rc;  /* -EAGAIN on timeout, -errno otherwise */
	}

	/* k_poll returned because at least one queue has data. Find a ready
	 * one and drain a single message. We deliberately drain only one
	 * per call so the broker can serve sessions in round-robin order
	 * if the kernel happens to mark several ready in the same wake. */
	for (int i = 0; i < LOOPBACK_MAX_SESSIONS; i++) {
		if (ev[i].state != K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
			continue;
		}
		if (k_msgq_get(rmw_to_broker[i], out, K_NO_WAIT) == 0) {
			*out_session_idx = i;
			return 0;
		}
	}
	/* Edge case: poll said ready but the message was already consumed.
	 * Treat as a spurious wakeup. */
	return -EAGAIN;
}

int loopback_broker_send(int session_idx, const struct loopback_slot *in)
{
	if (session_idx < 0 || session_idx >= LOOPBACK_MAX_SESSIONS) {
		return -EINVAL;
	}
	return k_msgq_put(broker_to_rmw[session_idx], in, K_MSEC(50));
}
