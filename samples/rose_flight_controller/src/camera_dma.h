/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Camera frame capture for the FPGA flight controller (ROSE_CAMERA). On a DMA-capable OSPI shell
 * (RocketArty200TDroneFullDDRDmaConfig), a dedicated LOW-priority thread configures the HM01B0 and
 * continuously DMAs frames straight into a DDR buffer -- no CPU per-pixel drain, so the ~2 kHz
 * control loop is undisturbed. The control loop doesn't consume pixels yet; it only reads the frame
 * counter for telemetry (proves camera capture is integrated + running alongside the loop).
 */
#ifndef ROSE_CAMERA_DMA_H_
#define ROSE_CAMERA_DMA_H_

#include <stdint.h>

#ifndef ROSE_CAMERA
#define ROSE_CAMERA 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

void     camera_dma_init(void);    /* start the capture thread (no-op if ROSE_CAMERA=0) */
uint32_t camera_dma_frames(void);  /* frames DMA'd since boot (0 if disabled) */
uint32_t camera_dma_last_mean(void); /* mean pixel of the last frame (cheap liveness signal) */

/* Request an on-demand snapshot: the capture thread nearest-neighbour downsamples the next frame to
 * w x h and streams it as base64 "IMG s=<seq> k=.." lines over uart1 (relayed to the GCS). seq tags
 * the frame so the GCS can group its chunks; w/h are clamped to [8, sensor size]. */
void camera_dma_request_snapshot(uint32_t seq, int w, int h);

#ifdef __cplusplus
}
#endif

#endif /* ROSE_CAMERA_DMA_H_ */
