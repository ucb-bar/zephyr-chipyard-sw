/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal on-board binary flight logger for riskybird bring-up. Writes compact fixed-size records
 * to the flash "storage" partition (survives reset / power-loss / reflash of the app slot), for
 * reading back over USB after an untethered flight. EARLY-PROTOTYPING ONLY -- erases + rewrites the
 * partition every boot, so remove/disable before it matters for flash wear.
 *
 * Usage (build-flag gated in main.cpp):
 *   -DROSE_FLIGHTLOG=1       log during the run (erase at boot, append per control tick / DIV)
 *   -DROSE_FLIGHTLOG_DUMP=1  on boot, dump the stored log as CSV over USB and halt (no erase)
 */
#ifndef ROSE_FLIGHTLOG_H
#define ROSE_FLIGHTLOG_H

#include <stdint.h>
#include <stdbool.h>

/* One log record. Packed to 20 bytes; fixed-point to stay compact. */
struct __attribute__((packed)) flight_rec {
	uint32_t t_ms;        /* uptime ms (0xFFFFFFFF = erased/empty sentinel) */
	int16_t  roll_mrad;   /* attitude, milliradians */
	int16_t  pitch_mrad;
	int16_t  yaw_mrad;
	int16_t  z_mm;        /* altitude estimate, mm */
	int16_t  vz_mmps;     /* vertical velocity, mm/s */
	uint8_t  duty[4];     /* per-motor commanded duty, 0-200 (=0.5% units) */
	uint8_t  flags;       /* bit0=estop, bit1=tof_valid */
	uint8_t  _pad;
};

#define FLIGHT_FLAG_ESTOP     0x01u
#define FLIGHT_FLAG_TOF_VALID 0x02u

#ifdef __cplusplus
extern "C" {
#endif

/* Open + erase the storage partition; ready to append. Returns 0 on success. */
int flightlog_init(void);
/* Append one record (buffered; flushed to flash a page at a time). No-op once the partition fills. */
void flightlog_write(const struct flight_rec *rec);
/* Flush any buffered partial page to flash (call at end of flight / before power-off). */
void flightlog_flush(void);
/* Read the stored log back and print it as CSV over the console. Returns #records printed. */
int flightlog_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* ROSE_FLIGHTLOG_H */
