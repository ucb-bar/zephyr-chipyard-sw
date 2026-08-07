/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Four side-facing VL53L5CX (8x8) wall sensors for the flight controller's "bumper" wall-avoid.
 * Bring-up (enable via ADS7128 GPIO + readdress off 0x29 to 0x31-0x34, persistence-aware) and the
 * blocking center-zone reads run on a DEDICATED low-priority thread, so the ~1 kHz control loop
 * never stalls on I2C -- it just reads the latest cached snapshot via side_tof_get().
 *
 * Body-frame facing (verified: front empirically; rest from the PCB netlist + design):
 *   0x31 = GPIO1 = J9  = back (-x)
 *   0x32 = GPIO2 = J2  = front (+x)
 *   0x33 = GPIO3 = J11 = right (-y)
 *   0x34 = GPIO4 = J10 = left (+y)
 */
#ifndef ROSE_SIDE_TOF_H
#define ROSE_SIDE_TOF_H

#include <stdint.h>
#include <stdbool.h>

/* Latest nearest-wall distance per body direction (center zone, mm). A value <= 0 means "no valid
 * reading / no target in range" for that direction (treat as no wall). */
struct side_walls {
	int16_t front_mm;   /* +x */
	int16_t back_mm;    /* -x */
	int16_t left_mm;    /* +y */
	int16_t right_mm;   /* -y */
	uint32_t seq;       /* increments each successful read cycle (0 = never read) */
};

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up all four side sensors (readdress + init) and start the background reader thread.
 * Returns the number of sensors successfully brought up (0-4); 0 means the bumper is unavailable. */
int side_tof_init(void);

/* Non-blocking: copy the latest cached wall snapshot. Safe from the control loop. */
void side_tof_get(struct side_walls *out);

#ifdef __cplusplus
}
#endif

#endif /* ROSE_SIDE_TOF_H */
