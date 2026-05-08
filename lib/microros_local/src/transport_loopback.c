/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Loopback transport implementation. See transport_loopback.h.
 */

#include <microros_local/transport_loopback.h>

#include <zephyr/sys/printk.h>
#include <string.h>

#define LOOPBACK_DEPTH 8   /* in-flight messages each direction */

K_MSGQ_DEFINE(rmw_to_broker_q, sizeof(struct loopback_slot), LOOPBACK_DEPTH, 4);
K_MSGQ_DEFINE(broker_to_rmw_q, sizeof(struct loopback_slot), LOOPBACK_DEPTH, 4);

bool loopback_open(struct uxrCustomTransport *t)
{
	(void)t;
	k_msgq_purge(&rmw_to_broker_q);
	k_msgq_purge(&broker_to_rmw_q);
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
	(void)t;
	(void)err;

	if (len > LOOPBACK_MTU) {
		printk("loopback_write: oversize %u\n", (unsigned)len);
		return 0;
	}
	struct loopback_slot s;
	s.len = (uint16_t)len;
	memcpy(s.data, buf, len);
	if (k_msgq_put(&rmw_to_broker_q, &s, K_MSEC(50)) != 0) {
		printk("loopback_write: queue full, dropped %u bytes\n", (unsigned)len);
		return 0;
	}
	return len;
}

size_t loopback_read(struct uxrCustomTransport *t,
		     uint8_t *buf, size_t len, int timeout_ms, uint8_t *err)
{
	(void)t;
	(void)err;

	struct loopback_slot s;
	if (k_msgq_get(&broker_to_rmw_q, &s, K_MSEC(timeout_ms)) != 0) {
		return 0;
	}
	size_t n = MIN((size_t)s.len, len);
	memcpy(buf, s.data, n);
	return n;
}

int loopback_broker_recv(struct loopback_slot *out, k_timeout_t to)
{
	return k_msgq_get(&rmw_to_broker_q, out, to);
}

int loopback_broker_send(const struct loopback_slot *in)
{
	return k_msgq_put(&broker_to_rmw_q, in, K_MSEC(50));
}
