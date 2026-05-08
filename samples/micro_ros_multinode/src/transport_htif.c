/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * HDLC-on-HTIF micro-ROS custom transport. See transport_htif.h for protocol.
 *
 * Single-threaded design: no separate reader thread. transport_read drives the
 * HDLC parser inline as it pulls bytes from HTIF. Avoids mutex contention with
 * printk and the htif driver's blocking poll_in semantics under no-input.
 */

#include "transport_htif.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#define HTIF_NODE   DT_NODELABEL(htif)
#define FLAG        0x7Eu
#define ESC         0x7Du
#define ESC_XOR     0x20u

/* Sized for one XRCE-DDS message. Our colcon.meta caps MTU at 512;
 * pad for headers + worst-case escaping. */
#define MAX_PAYLOAD 1024

/* CRC16-CCITT (poly 0x1021, init 0xFFFF, no xorout, no reflect). */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFFu;
	for (size_t i = 0; i < len; i++) {
		crc ^= ((uint16_t)data[i]) << 8;
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
					      : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/* ---------- module-private state ---------- */

static const struct device *htif_dev;

enum parser_state {
	PS_OUT_OF_FRAME,
	PS_IN_FRAME,
	PS_IN_FRAME_ESC,
};

/* The parser carries state across transport_read calls so a frame split across
 * multiple reads is handled correctly. */
static enum parser_state parse_state = PS_OUT_OF_FRAME;
static uint8_t parse_accum[MAX_PAYLOAD + 2];
static size_t  parse_len;

/* When a complete, CRC-validated frame is decoded inside transport_read, its
 * payload is staged here and drained byte-by-byte to the caller's buffer. */
static uint8_t staged_payload[MAX_PAYLOAD];
static size_t  staged_len;
static size_t  staged_off;

/* Push one raw byte into the parser. If a frame just completed and was valid,
 * stage its payload for delivery. Returns true if a frame is now staged. */
static bool feed_parser(uint8_t raw)
{
	switch (parse_state) {
	case PS_OUT_OF_FRAME:
		if (raw == FLAG) {
			parse_len = 0;
			parse_state = PS_IN_FRAME;
		}
		/* else: out-of-frame (interleaved printk byte), ignore */
		return false;

	case PS_IN_FRAME:
		if (raw == FLAG) {
			bool ok = false;
			if (parse_len >= 3) {
				size_t plen = parse_len - 2;
				uint16_t got = ((uint16_t)parse_accum[plen] << 8) |
					       (uint16_t)parse_accum[plen + 1];
				uint16_t exp = crc16_ccitt(parse_accum, plen);
				if (got == exp) {
					memcpy(staged_payload, parse_accum, plen);
					staged_len = plen;
					staged_off = 0;
					ok = true;
				}
			}
			parse_len = 0;
			parse_state = PS_IN_FRAME;  /* trailing FLAG = leading FLAG */
			return ok;
		}
		if (raw == ESC) {
			parse_state = PS_IN_FRAME_ESC;
			return false;
		}
		if (parse_len < sizeof(parse_accum)) {
			parse_accum[parse_len++] = raw;
		} else {
			parse_len = 0;
			parse_state = PS_OUT_OF_FRAME;
		}
		return false;

	case PS_IN_FRAME_ESC:
		if (parse_len < sizeof(parse_accum)) {
			parse_accum[parse_len++] = raw ^ ESC_XOR;
			parse_state = PS_IN_FRAME;
		} else {
			parse_len = 0;
			parse_state = PS_OUT_OF_FRAME;
		}
		return false;
	}
	return false;
}

/* ---------- the four uxrCustomTransport hooks ---------- */

bool htif_transport_open(struct uxrCustomTransport *t)
{
	(void)t;
	htif_dev = DEVICE_DT_GET(HTIF_NODE);
	if (!device_is_ready(htif_dev)) {
		printk("htif transport: device not ready\n");
		return false;
	}
	parse_state = PS_OUT_OF_FRAME;
	parse_len = 0;
	staged_len = 0;
	staged_off = 0;
	return true;
}

bool htif_transport_close(struct uxrCustomTransport *t)
{
	(void)t;
	return true;
}

static inline void emit_escaped(uint8_t b)
{
	if (b == FLAG || b == ESC) {
		uart_poll_out(htif_dev, ESC);
		uart_poll_out(htif_dev, b ^ ESC_XOR);
	} else {
		uart_poll_out(htif_dev, b);
	}
}

size_t htif_transport_write(struct uxrCustomTransport *t,
			    const uint8_t *buf, size_t len, uint8_t *err)
{
	(void)t; (void)err;

	uart_poll_out(htif_dev, FLAG);
	for (size_t i = 0; i < len; i++) {
		emit_escaped(buf[i]);
	}
	uint16_t crc = crc16_ccitt(buf, len);
	emit_escaped((uint8_t)(crc >> 8));
	emit_escaped((uint8_t)(crc & 0xFFu));
	uart_poll_out(htif_dev, FLAG);
	return len;
}

/*
 * IMPORTANT — runtime expectation:
 *
 * spike's HTIF console GETC implementation BLOCKS when its stdin has no
 * data available; it does not return -1 on EOF or timeout the way a serial
 * UART would. There is no non-blocking mode for the HTIF byte channel.
 *
 * Consequence: this transport_read can only make forward progress when the
 * agent is actively feeding spike's stdin. The supported runtime path is
 * `tools/microros/run_with_agent.sh`, which launches both spike and
 * micro-ros-agent and pipes the agent's output into spike's stdin via the
 * HDLC proxy. With that in place, GETC unblocks promptly on each agent
 * response, and ping_agent / discovery / data flow all work normally.
 *
 * Running `spike <elf>` directly (no agent) WILL hang here on the first
 * ping_agent transport_read. That is expected for this configuration —
 * it's a stdin-blocking limitation of spike's HTIF frontend, not a bug
 * in this transport.
 */
size_t htif_transport_read(struct uxrCustomTransport *t,
			   uint8_t *buf, size_t len, int timeout_ms, uint8_t *err)
{
	(void)t; (void)err;

	size_t out = 0;

	/* Drain any payload bytes already staged from a previous call. */
	while (staged_off < staged_len && out < len) {
		buf[out++] = staged_payload[staged_off++];
	}
	if (out > 0) {
		return out;
	}

	const int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;
	const int slice_ms = 2;
	for (;;) {
		if (k_uptime_get() >= deadline) {
			return out;
		}
		unsigned char raw;
		int rc = uart_poll_in(htif_dev, &raw);
		if (rc == 0) {
			if (feed_parser((uint8_t)raw)) {
				while (staged_off < staged_len && out < len) {
					buf[out++] = staged_payload[staged_off++];
				}
				return out;
			}
			continue;
		}
		k_msleep(slice_ms);
	}
}
