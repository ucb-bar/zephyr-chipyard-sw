/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flash-backed flight logger (see flightlog.h). The control loop only appends records to a RAM ring
 * buffer (fast, non-blocking); a dedicated LOW-PRIORITY logger thread drains the ring in ~4 KB
 * chunks and does the (slow, CPU-stalling) flash writes off the control path. Records stream
 * contiguously (record-aligned, no page padding) to the "storage" partition so the dump reads one
 * continuous stream ending at the first erased (0xFF) record.
 */
#include "flightlog.h"

#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define LOG_PARTITION   storage_partition
#define REC_SIZE        ((uint32_t)sizeof(struct flight_rec))   /* 20 (multiple of 4) */
/* Flush chunk kept small so each flash write (a whole-CPU XIP stall on ESP32, ~1.5us/byte) is
 * short: 50 records = 1000 B ~= 1.5 ms, vs ~6 ms for a 4 KB page. At ~50 Hz logging that's one
 * ~1.5 ms stall per ~1 s -- shorter than the vehicle's response time, so control is undisturbed. */
#define FLUSH_RECS      50u
#define FLUSH_CHUNK     (FLUSH_RECS * REC_SIZE)                 /* 1000 B: whole records, 4-aligned */
#define RB_SIZE         8192u                                   /* several chunks of slack */

static const struct flash_area *g_fa;
static uint32_t g_area_size;
static uint32_t g_write_off;
static bool     g_full;
static uint32_t g_dropped;                 /* records dropped if the logger falls behind */

RING_BUF_DECLARE(g_rb, RB_SIZE);
static uint8_t  g_wbuf[FLUSH_CHUNK];       /* logger-thread scratch (consumer only) */

K_SEM_DEFINE(g_wake, 0, 1);                /* producer -> logger: data ready / flush requested */
K_SEM_DEFINE(g_flush_done, 0, 1);          /* logger -> producer: requested flush completed */
static volatile bool g_flush_req;
static volatile bool g_run;

K_THREAD_STACK_DEFINE(g_logger_stack, 2048);
static struct k_thread g_logger_thread;

/* Write nbytes from g_wbuf to flash (consumer thread only). Times the write so we can report the
 * per-flush CPU stall. nbytes is always a multiple of REC_SIZE (=> 4-byte aligned). */
static void do_write(uint32_t nbytes)
{
	if (nbytes == 0 || g_full) {
		return;
	}
	if (g_write_off + nbytes > g_area_size) {
		g_full = true;
		printk("flightlog: partition full (%u bytes) -- logging stopped\n", g_area_size);
		return;
	}
	uint32_t c0 = k_cycle_get_32();
	int rc = flash_area_write(g_fa, g_write_off, g_wbuf, nbytes);
	uint32_t us = k_cyc_to_us_floor32(k_cycle_get_32() - c0);
	if (rc) {
		g_full = true;
		printk("flightlog: write @%u failed (%d) -- logging stopped\n", g_write_off, rc);
		return;
	}
	g_write_off += nbytes;
	static uint32_t nflush;
	if ((nflush++ % 8) == 0) {
		printk("flightlog: flash write %u B took %u us (off=%u)\n", nbytes, us, g_write_off);
	}
}

static void drain_full_chunks(void)
{
	while (!g_full && ring_buf_size_get(&g_rb) >= FLUSH_CHUNK) {
		uint32_t got = ring_buf_get(&g_rb, g_wbuf, FLUSH_CHUNK);
		do_write(got);
	}
}

static void logger_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (g_run) {
		k_sem_take(&g_wake, K_MSEC(200));   /* wake on data/flush, or poll periodically */
		drain_full_chunks();
		if (g_flush_req) {
			/* drain everything remaining, record-aligned, in <=chunk writes */
			uint32_t avail = ring_buf_size_get(&g_rb);
			avail -= avail % REC_SIZE;
			while (!g_full && avail > 0) {
				uint32_t want = avail > FLUSH_CHUNK ? FLUSH_CHUNK : avail;
				uint32_t got = ring_buf_get(&g_rb, g_wbuf, want);
				do_write(got);
				avail -= got;
			}
			g_flush_req = false;
			k_sem_give(&g_flush_done);
		}
	}
}

int flightlog_init(void)
{
	int rc = flash_area_open(FIXED_PARTITION_ID(LOG_PARTITION), &g_fa);
	if (rc) {
		printk("flightlog: flash_area_open failed (%d)\n", rc);
		return rc;
	}
	g_area_size = (uint32_t)g_fa->fa_size;
	rc = flash_area_erase(g_fa, 0, g_area_size);
	if (rc) {
		printk("flightlog: erase failed (%d)\n", rc);
		return rc;
	}
	g_write_off = 0;
	g_full = false;
	g_dropped = 0;
	g_flush_req = false;
	g_run = true;
	ring_buf_reset(&g_rb);
	/* Low priority (below control loop + ToF thread) so flash writes never preempt control; the
	 * flash op itself briefly stalls the CPU, but only runs when control has yielded. */
	k_thread_create(&g_logger_thread, g_logger_stack, K_THREAD_STACK_SIZEOF(g_logger_stack),
			logger_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	k_thread_name_set(&g_logger_thread, "flightlog");
	printk("flightlog: ready, %u bytes (%u records), background writer\n",
	       g_area_size, g_area_size / REC_SIZE);
	return 0;
}

void flightlog_write(const struct flight_rec *rec)
{
	if (g_full || g_fa == NULL) {
		return;
	}
	uint32_t n = ring_buf_put(&g_rb, (const uint8_t *)rec, REC_SIZE);
	if (n < REC_SIZE) {
		g_dropped++;   /* ring full: logger fell behind (shouldn't happen at ~50 Hz) */
		return;
	}
	if (ring_buf_size_get(&g_rb) >= FLUSH_CHUNK) {
		k_sem_give(&g_wake);
	}
}

void flightlog_flush(void)
{
	if (g_fa == NULL) {
		return;
	}
	g_flush_req = true;
	k_sem_give(&g_wake);
	k_sem_take(&g_flush_done, K_MSEC(2000));   /* wait (bounded) for the logger to drain */
	if (g_dropped) {
		printk("flightlog: %u records dropped (logger overrun)\n", g_dropped);
	}
}

int flightlog_dump(void)
{
	const struct flash_area *fa;
	int rc = flash_area_open(FIXED_PARTITION_ID(LOG_PARTITION), &fa);
	if (rc) {
		printk("flightlog: dump open failed (%d)\n", rc);
		return 0;
	}
	uint32_t size = (uint32_t)fa->fa_size;
	printk("FLIGHTLOG_CSV_BEGIN\n");
	printk("t_ms,roll_mrad,pitch_mrad,yaw_mrad,z_mm,vz_mmps,duty0,duty1,duty2,duty3,flags\n");
	int n = 0;
	struct flight_rec r;
	for (uint32_t off = 0; off + REC_SIZE <= size; off += REC_SIZE) {
		if (flash_area_read(fa, off, &r, REC_SIZE) != 0) {
			break;
		}
		if (r.t_ms == 0xFFFFFFFFu) {
			break;    /* reached erased region -> end of log */
		}
		printk("%u,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u\n", r.t_ms, r.roll_mrad, r.pitch_mrad,
		       r.yaw_mrad, r.z_mm, r.vz_mmps, r.duty[0], r.duty[1], r.duty[2], r.duty[3],
		       r.flags);
		n++;
	}
	printk("FLIGHTLOG_CSV_END (%d records)\n", n);
	flash_area_close(fa);
	return n;
}
