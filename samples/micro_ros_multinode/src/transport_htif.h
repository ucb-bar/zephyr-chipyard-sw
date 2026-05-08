/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * micro-ROS custom transport over HTIF with HDLC-style framing, so micro-ROS
 * bytes coexist with printk on the shared HTIF console. The four hooks here
 * are wired in via rmw_uros_set_custom_transport() in main.c.
 *
 * Frame format (one frame per transport_write() call):
 *
 *     0x7E  <escaped payload>  <escaped CRC16-CCITT, big-endian>  0x7E
 *
 * Escape rule:  0x7E -> 0x7D 0x5E,   0x7D -> 0x7D 0x5D
 *
 * On read, a Zephyr thread polls HTIF byte-by-byte, drops bytes that aren't
 * inside a frame (those are interleaved printk output, ignored by the
 * transport), unescapes payload, validates CRC, and pushes the payload onto
 * a ring buffer that transport_read() drains.
 */
#ifndef MICRO_ROS_MULTINODE_TRANSPORT_HTIF_H
#define MICRO_ROS_MULTINODE_TRANSPORT_HTIF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct uxrCustomTransport;

#ifdef __cplusplus
extern "C" {
#endif

bool   htif_transport_open(struct uxrCustomTransport *t);
bool   htif_transport_close(struct uxrCustomTransport *t);
size_t htif_transport_write(struct uxrCustomTransport *t,
                            const uint8_t *buf, size_t len, uint8_t *err);
size_t htif_transport_read(struct uxrCustomTransport *t,
                           uint8_t *buf, size_t len, int timeout_ms, uint8_t *err);

#ifdef __cplusplus
}
#endif

#endif
