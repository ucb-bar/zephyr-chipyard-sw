/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flash-backed flight logger (see flightlog.h). Records are buffered in a RAM page and written to
 * the "storage" flash partition one 4 KB page at a time. ESP32 flash writes briefly stall the CPU
 * (XIP cache), so page flushes are infrequent (one per ~200 records); for real flight, drive
 * flightlog_write() from a low-priority thread rather than the control loop.
 */
#include "flightlog.h"

#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define LOG_PARTITION   storage_partition
#define REC_SIZE        ((uint32_t)sizeof(struct flight_rec))   /* 20 (multiple of 4 -> flash-aligned) */
#define FLUSH_RECS      200u
#define FLUSH_CHUNK     (FLUSH_RECS * REC_SIZE)                 /* 4000 B: whole records, 4-aligned */

static const struct flash_area *g_fa;
static uint32_t g_area_size;
static uint32_t g_write_off;                 /* next flash write offset (record-aligned) */
static uint8_t  g_buf[FLUSH_CHUNK];          /* RAM staging (whole records only, no page padding) */
static uint32_t g_buf_used;                  /* bytes staged */
static bool     g_full;

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
	g_buf_used = 0;
	g_full = false;
	printk("flightlog: ready, %u bytes (%u records) on 'storage' partition\n",
	       g_area_size, g_area_size / REC_SIZE);
	return 0;
}

/* Write staged whole-record bytes to flash contiguously (no page padding, so the dump reads one
 * continuous stream and the first 0xFF record marks the true end). g_buf_used is always a multiple
 * of REC_SIZE (=20 -> 4-byte aligned), satisfying the flash write-block alignment. */
static void flush_chunk(void)
{
	if (g_buf_used == 0 || g_full) {
		return;
	}
	if (g_write_off + g_buf_used > g_area_size) {
		g_full = true;
		printk("flightlog: partition full (%u bytes) -- logging stopped\n", g_area_size);
		return;
	}
	int rc = flash_area_write(g_fa, g_write_off, g_buf, g_buf_used);
	if (rc) {
		printk("flightlog: write @%u failed (%d) -- logging stopped\n", g_write_off, rc);
		g_full = true;
		return;
	}
	g_write_off += g_buf_used;
	g_buf_used = 0;
}

void flightlog_write(const struct flight_rec *rec)
{
	if (g_full || g_fa == NULL) {
		return;
	}
	memcpy(&g_buf[g_buf_used], rec, REC_SIZE);
	g_buf_used += REC_SIZE;
	if (g_buf_used >= FLUSH_CHUNK) {
		flush_chunk();
	}
}

void flightlog_flush(void)
{
	flush_chunk();
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
			break;    /* reached erased/unwritten region -> end of log */
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
