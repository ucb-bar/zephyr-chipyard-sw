/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * RoSE fused-vision navigation controller (Milestone-1 transport bring-up).
 *
 * Runs the winning fused-sensor nav model (int8 vision + int8 depth encoders, fp16 tail,
 * fp16 lowdim) ON the SoC, in the RoSE↔IsaacLab warehouse-gate co-sim. Unlike the DroNet
 * study this drops the on-SoC TinyMPC/estimator: in Stage-1 IsaacLab owns the inner
 * velocity tracker, so the SoC's job is purely sensor-in → model → policy-out.
 *
 * Per-tick transport (wire contract, matched to the Isaac bridge env + config_gym yaml):
 *   - 0x11 DMA ch0   : front_grey int8[5400] (1x1x60x90) — pre-quantized by Isaac, delivered
 *                      over the reusable ucbbar,rose-camera DMA path as a GREY 90x60 "frame"
 *                      (5400 bytes). Used verbatim as model input0 (bytes reinterpreted int8).
 *   - 0x41 reqrsp    : tof_cross int8[256] (1x4x8x8) = 64 u32 words → input1.
 *   - 0x42 reqrsp    : lowdim float32[21] = 21 words; each cast to _Float16 → input2
 *                      (matches numpy float16 rounding of the host reference).
 *   - 0x20 action    : (yaw_rate, forward_speed) as float32[2] guest→Isaac (fp16 out → f32).
 *
 * The model uses a Zfh (scalar half-precision) fp16 tail, so the guest MUST be built with
 * CONFIG_RISCV_ISA_EXT_ZFH=y and RUN on spike with --isa=rv64gc_zfh.
 */
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <string.h>

#include <rose/rose.h>
#include <rose/rose_proto.h>

extern "C" {
#include "model.h"
}

#if ROSE_RVV
/* Curated rvv_f16 kernels emit RVV (V) + Zvfh vector-half instructions. Set mstatus VS/FS/XS to
 * Dirty so vector state is active (mirrors matlib.h enable_vector_operations()); with
 * CONFIG_RISCV_ISA_EXT_V_LAZY=n Zephyr also eager-saves V per thread. */
static inline void enable_vector_operations(void)
{
	unsigned long mstatus;
	__asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
	mstatus |= (0x3UL << 9) | (0x3UL << 13) | (0x3UL << 15);   /* VS | FS | XS = Dirty */
	__asm__ volatile("csrw mstatus, %0" :: "r"(mstatus));
}
#endif

/* Command IDs / reqrsp channels — MUST match config_gym_WarehouseFusedNavBridgeEnv-v0.yaml. */
#define ROSE_CMD_ACTION   0x20u
#define CMD_TOF_CROSS     0x41u
#define CMD_LOWDIM        0x42u
#define TOF_CROSS_CH      1
#define LOWDIM_CH         2

#define FRONT_ELEMS   5400   /* 60x90 int8 front_grey */
#define TOF_ELEMS     256    /* 4x8x8 int8 tof_cross  */
#define TOF_WORDS     64     /* 256 bytes = 64 u32    */
#define LOWDIM_ELEMS  21     /* fp32 over the wire; cast to fp16 on-guest */

#ifndef CTRL_ITERS
#define CTRL_ITERS 200
#endif

static const struct device *rose = DEVICE_DT_GET_ONE(ucbbar_roseadapter);
static const struct device *cam  = DEVICE_DT_GET(DT_ALIAS(fpv));

/* int8 model input0 delivered as raw bytes over the camera-DMA path. */
static uint8_t   g_front[FRONT_ELEMS];
static struct video_buffer g_vbuf;
static int8_t    g_input1[TOF_ELEMS];
static _Float16  g_input2[LOWDIM_ELEMS];
static _Float16  g_output[MODEL_OUTPUT_SIZE];

/* 0x11 DMA: request + block for the pre-quantized int8 front frame (5400 B). */
static bool capture_front(void)
{
	struct video_buffer *out = NULL;
	g_vbuf.buffer = g_front;
	g_vbuf.size = sizeof(g_front);
	g_vbuf.type = VIDEO_BUF_TYPE_OUTPUT;
	if (video_enqueue(cam, &g_vbuf) != 0) {
		return false;
	}
	if (video_dequeue(cam, &out, K_FOREVER) != 0 || out == NULL) {
		return false;
	}
	return true;
}

/* 0x41 reqrsp: tof_cross int8[256] (64 words) → input1 (bytes verbatim). */
static bool read_tof_cross(void)
{
	uint32_t raw[TOF_WORDS];
	rose_request(rose, CMD_TOF_CROSS, 0U);
	int got = rose_recv_reqrsp(rose, TOF_CROSS_CH, raw, TOF_WORDS);
	if (got < TOF_WORDS) {
		return false;
	}
	memcpy(g_input1, raw, TOF_ELEMS);   /* 64 words = 256 int8 bytes */
	return true;
}

/* 0x42 reqrsp: lowdim float32[21] → cast each to _Float16 → input2. */
static bool read_lowdim(void)
{
	uint32_t raw[LOWDIM_ELEMS];
	rose_request(rose, CMD_LOWDIM, 0U);
	int got = rose_recv_reqrsp(rose, LOWDIM_CH, raw, LOWDIM_ELEMS);
	if (got < LOWDIM_ELEMS) {
		return false;
	}
	for (int i = 0; i < LOWDIM_ELEMS; i++) {
		float f;
		memcpy(&f, &raw[i], sizeof(float));
		g_input2[i] = (_Float16)f;   /* == numpy float16 rounding of the wire fp32 */
	}
	return true;
}

/* 0x20 action: fp16 (yaw_rate, forward_speed) → float32[2] → Isaac. */
static void send_action(const _Float16 *out)
{
	rose_tx(rose, ROSE_CMD_ACTION);
	rose_tx(rose, MODEL_OUTPUT_SIZE * sizeof(float));
	for (int i = 0; i < MODEL_OUTPUT_SIZE; i++) {
		float f = (float)out[i];
		uint32_t w;
		memcpy(&w, &f, sizeof(float));
		rose_tx(rose, w);
	}
}

int main(void)
{
#if ROSE_RVV
	enable_vector_operations();
	printf("fused_nav: boot model=%s in=%d out=%d (RVV rvv_f16 curated + zfh/zvfh)\n",
	       MODEL_NAME, MODEL_INPUT_SIZE, MODEL_OUTPUT_SIZE);
#else
	printf("fused_nav: boot model=%s in=%d out=%d (scalar zfh fp16 tail)\n",
	       MODEL_NAME, MODEL_INPUT_SIZE, MODEL_OUTPUT_SIZE);
#endif
	if (!device_is_ready(rose)) {
		printf("fused_nav: FAIL rose adapter not ready\n");
		return -1;
	}
	if (!device_is_ready(cam)) {
		printf("fused_nav: FAIL fpv camera not ready\n");
		return -1;
	}

	for (int iter = 0; iter < CTRL_ITERS; iter++) {
		bool ok = capture_front();
		ok = read_tof_cross() && ok;
		ok = read_lowdim() && ok;
		if (!ok) {
			printf("fused_nav: iter=%d SENSE ERROR (front/tof/lowdim)\n", iter);
			continue;
		}

		/* LSTM state is internal file-static and persists across calls (seq-1 stepping). */
		run_model_fused_full((const model_fused_full_input0_t *)g_front,
				     g_input1, g_input2, g_output, NULL);
		send_action(g_output);

		if (iter < 5 || (iter % 20) == 0) {
			printf("FUSED_NAV_OUT iter=%d yr=%.9g fwd=%.9g\n",
			       iter, (double)(float)g_output[0], (double)(float)g_output[1]);
		}
	}
	printf("fused_nav: done (%d iters)\n", CTRL_ITERS);
	return 0;
}
