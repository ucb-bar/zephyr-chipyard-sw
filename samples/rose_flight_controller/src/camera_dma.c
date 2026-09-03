/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * See camera_dma.h. Body compiled only when ROSE_CAMERA=1; otherwise empty stubs so non-camera
 * builds are unaffected. Requires a DMA-capable OSPI shell (RocketArty200TDroneFullDDRDmaConfig).
 */
#include "camera_dma.h"

#if ROSE_CAMERA

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

/* OSPI capture + DMA register map (base 0x10080000). */
#define OSPI_BASE       0x10080000UL
#define O_CTRL          0x00
#define O_MCLKDIV       0x08
#define O_FIFOCOUNT     0x0c
#define O_CAPACITY      0x24
#define O_PIXTARGET     0x28
#define O_FVLDCNT       0x34
#define O_DMA_ADDR_LO   0x44
#define O_DMA_ADDR_HI   0x48
#define O_DMA_LEN       0x4c
#define O_DMA_CTRL      0x50
#define O_DMA_STATUS    0x54
#define CTRL_EN   (1u<<0)
#define CTRL_CLR  (1u<<4)
#define CTRL_FLU  (1u<<5)
#define CTRL_ARM  (1u<<6)
#define DMA_EN    (1u<<0)
#define DMA_START (1u<<1)
#define DMA_CLEAR (1u<<3)
#define DMA_BUSY  (1u<<0)
#define DMA_DONE  (1u<<1)

#define CAM_ADDR   0x24
#define CAP_PIXELS 104000
#define MCLK_DIV   1

static uint8_t __aligned(64) cam_buf[128 * 1024];   /* DDR sink for the DMA */
static volatile uint32_t g_frames;
static volatile uint32_t g_last_mean;

/* ---- on-demand snapshot streaming over uart1 (GCS "SNAP <w> <h>" command) ------------------- *
 * The captured frame is ~326 wide; CAP_PIXELS/326 full rows fit the DDR buffer. On request we
 * nearest-neighbour downsample cam_buf to w x h, base64 it, and emit newline-framed chunks the
 * ESP bridge relays to the GCS on UDP :14550:
 *     IMG s=<seq> k=<chunk>/<total> w=<W> h=<H> <base64 of W*H grayscale bytes>
 * Streaming runs in this low-prio camera thread (between captures) so the control loop is undisturbed. */
#include "telem_uart.h"
#include <zephyr/sys/printk.h>   /* snprintk */

/* True line stride of the captured frame in cam_buf = 324 (HM01B0 active line). Verified by
 * reconstructing a raw buffer dump: at 324 the image is coherent with straight vertical edges; at
 * 326/325 it sheared/duplicated (326 was the old guess; a 648=2x324 autocorrelation alias briefly
 * looked like 325). 104000/324 = 320 rows captured. */
#define SRC_W 324
#define SRC_H (CAP_PIXELS / SRC_W)   /* 320 rows */

/* Periodic auto-snapshot: stream a snapshot every Nth captured frame so the GCS shows a live camera
 * view even without the uplink command path. 0 = off (on-demand SNAP only). */
#ifndef ROSE_CAM_AUTOSNAP
#define ROSE_CAM_AUTOSNAP 0
#endif
/* 128x126 keeps the sensor's near-square 324x320 aspect (vs a stretched 80x60) and is high enough
 * that the phone doesn't show chunky pixels; ~21.5 KB base64 -> ~2 s per frame over the 115200 link. */
#ifndef ROSE_CAM_AUTOSNAP_W
#define ROSE_CAM_AUTOSNAP_W 128
#endif
#ifndef ROSE_CAM_AUTOSNAP_H
#define ROSE_CAM_AUTOSNAP_H 126
#endif

static volatile int      g_snap_w, g_snap_h;
static volatile uint32_t g_snap_req;   /* nonzero seq = a snapshot is requested */

static const char B64[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/* base64-encode n (<=3) bytes into out (4 chars, with '=' padding); returns 4. */
static int b64_triplet(const uint8_t *in, int n, char *out)
{
	uint32_t v = (uint32_t)in[0] << 16;
	if (n > 1) v |= (uint32_t)in[1] << 8;
	if (n > 2) v |= (uint32_t)in[2];
	out[0] = B64[(v >> 18) & 0x3F];
	out[1] = B64[(v >> 12) & 0x3F];
	out[2] = (n > 1) ? B64[(v >> 6) & 0x3F] : '=';
	out[3] = (n > 2) ? B64[v & 0x3F] : '=';
	return 4;
}

static void stream_snapshot(uint32_t seq, int w, int h)
{
	if (w < 8) w = 8;  if (w > SRC_W) w = SRC_W;
	if (h < 8) h = 8;  if (h > SRC_H) h = SRC_H;

	enum { RAW_PER = 150 };            /* multiple of 3 -> clean base64; 200 b64/chunk (~240 B line).
					    * Safe now the ESP bridge does interrupt-driven RX into an 8 KB ring
					    * (fpga_telem_bridge) -- it no longer loses bytes on a sustained burst. */
	const int total = (w * h + RAW_PER - 1) / RAW_PER;
	char line[560];
	uint8_t raw[RAW_PER];
	int rn = 0, k = 0;

	for (int oy = 0; oy < h; oy++) {
		int sy0 = oy * SRC_H / h, sy1 = (oy + 1) * SRC_H / h;
		if (sy1 <= sy0) { sy1 = sy0 + 1; }
		for (int ox = 0; ox < w; ox++) {
			int sx0 = ox * SRC_W / w, sx1 = (ox + 1) * SRC_W / w;
			if (sx1 <= sx0) { sx1 = sx0 + 1; }
			/* Box-average the whole source block into one output pixel (not nearest-neighbour).
			 * Averaging >=2 source rows cancels the HM01B0 even/odd row brightness pattern
			 * (period-2 row FPN, ~92 vs ~74) that a single-row pick aliases into horizontal
			 * stripes as sy=oy*SRC_H/h beats through the parity; it also anti-aliases the scale. */
			uint32_t sum = 0, cnt = 0;
			for (int sy = sy0; sy < sy1; sy++) {
				const uint8_t *row = &cam_buf[(uint32_t)sy * SRC_W];
				for (int sx = sx0; sx < sx1; sx++) { sum += row[sx]; cnt++; }
			}
			raw[rn++] = (uint8_t)(sum / cnt);
			if (rn == RAW_PER) {
				int off = snprintk(line, sizeof(line), "IMG s=%u k=%d/%d w=%d h=%d ",
						   seq, k++, total, w, h);
				for (int i = 0; i < rn; i += 3)
					off += b64_triplet(&raw[i], (rn - i) < 3 ? (rn - i) : 3, &line[off]);
				line[off++] = '\n'; line[off] = '\0';
				telem_uart_line(line);
				k_msleep(5);   /* light pace so the ISR-relayed ring drains via UDP + RBT interleaves */
				rn = 0;
			}
		}
	}
	if (rn > 0) {
		int off = snprintk(line, sizeof(line), "IMG s=%u k=%d/%d w=%d h=%d ",
				   seq, k++, total, w, h);
		for (int i = 0; i < rn; i += 3)
			off += b64_triplet(&raw[i], (rn - i) < 3 ? (rn - i) : 3, &line[off]);
		line[off++] = '\n'; line[off] = '\0';
		telem_uart_line(line);
	}
}

/* Himax reference init (clock-safe subset; matches camera_image_fpga / camera_dma_fpga). */
static const struct { uint16_t r; uint8_t v; } HM_INIT[] = {
	{0x1003,0x08},{0x1007,0x08},{0x1000,0x43},{0x1001,0x40},{0x1002,0x32},{0x0350,0x7F},
	{0x1006,0x01},{0x1008,0x00},{0x1009,0xA0},{0x100A,0x60},{0x100B,0x90},{0x100C,0x40},
	{0x3044,0x0A},{0x3045,0x00},{0x3047,0x0A},{0x3050,0xC0},{0x3051,0x42},{0x3052,0x50},
	{0x3053,0x00},{0x3054,0x03},{0x3055,0xF7},{0x3056,0xF8},{0x3057,0x29},{0x3058,0x1F},
	{0x3064,0x00},{0x3065,0x04},{0x3022,0x01},{0x1012,0x01},{0x2000,0x07},
	{0x2100,0x01},{0x2101,0x40},{0x2102,0x0A},{0x2103,0x03},{0x2104,0x07},
	{0x2105,0x02},{0x2106,0x20},{0x2108,0x03},{0x2109,0x03},{0x210B,0x80},
	{0x0101,0x01},{0x0104,0x01},
};

static inline void     w32(uintptr_t o, uint32_t v) { *(volatile uint32_t *)(OSPI_BASE + o) = v; }
static inline uint32_t r32(uintptr_t o) { return *(volatile uint32_t *)(OSPI_BASE + o); }

static int cam_wr(const struct device *b, uint16_t reg, uint8_t val)
{
	uint8_t tx[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val };
	return i2c_write(b, tx, sizeof(tx), CAM_ADDR);
}
static int cam_rd(const struct device *b, uint16_t reg, uint8_t *val)
{
	uint8_t rb[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
	return i2c_write_read(b, CAM_ADDR, rb, sizeof(rb), val, 1);
}

static void cam_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	/* The one-time SCCB init is ~40 I2C transactions on the shared i2c0 bus. At the capture-loop
	 * priority (12) it is starved forever by the flat-out control loop + the down-ToF thread (which
	 * always win the bus mutex). Boost above them for the init, then drop back so the capture loop
	 * never disturbs the control loop. */
	k_thread_priority_set(k_current_get(), 5);
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(bus)) { printk("camera_dma: i2c0 not ready\n"); return; }
	printk("camera_dma: OSPI capacity=%u\n", r32(O_CAPACITY));

	if (r32(O_CAPACITY) == 0) { printk("camera_dma: no OSPI capture peripheral (need DMA shell)\n"); return; }

	w32(O_MCLKDIV, MCLK_DIV);
	w32(O_CTRL, CTRL_EN);

	uint8_t hi = 0, lo = 0;
	if (cam_rd(bus, 0x0000, &hi) || cam_rd(bus, 0x0001, &lo) || (((uint16_t)hi << 8) | lo) != 0x01B0) {
		printk("camera_dma: HM01B0 MODEL_ID 0x%02x%02x (bad) -- camera off\n", hi, lo);
		return;
	}
	cam_wr(bus, 0x0103, 0x01); k_msleep(50); cam_wr(bus, 0x0100, 0x00);   /* SW reset + standby */
	for (unsigned i = 0; i < sizeof(HM_INIT) / sizeof(HM_INIT[0]); i++) { cam_wr(bus, HM_INIT[i].r, HM_INIT[i].v); }
	cam_wr(bus, 0x0601, 0x00);          /* live (no test pattern) */
	cam_wr(bus, 0x0100, 0x01);          /* stream */
	k_msleep(1200);
	printk("camera_dma: HM01B0 streaming; DMA capture thread running (buf @ %p)\n", (void *)cam_buf);
	k_thread_priority_set(k_current_get(), 12);   /* init done -> low prio for the capture loop */

	uintptr_t dst = (uintptr_t)cam_buf;
	for (;;) {
		/* Frame-aligned bounded capture into the OSPI frame buffer. Short bounded spins with
		 * yields so this low-prio thread never hogs the CPU from the control loop. */
		w32(O_PIXTARGET, CAP_PIXELS);
		w32(O_CTRL, CTRL_EN | CTRL_CLR | CTRL_FLU);
		uint32_t fv = r32(O_FVLDCNT);
		for (int g = 0; g < 60 && r32(O_FVLDCNT) == fv; g++) { k_busy_wait(200); }
		w32(O_CTRL, CTRL_EN | CTRL_CLR | CTRL_FLU);
		w32(O_CTRL, CTRL_EN | CTRL_ARM);
		w32(O_CTRL, CTRL_EN);
		for (int t = 0; t < 60 && r32(O_FIFOCOUNT) < (CAP_PIXELS - 4096); t++) { k_msleep(1); }

		/* Kick the DMA to drain the frame buffer to DDR (HW moves the bytes; CPU just waits). */
		w32(O_DMA_ADDR_LO, (uint32_t)(dst & 0xffffffffu));
		w32(O_DMA_ADDR_HI, (uint32_t)(dst >> 32));
		w32(O_DMA_LEN, CAP_PIXELS);
		w32(O_DMA_CTRL, DMA_EN | DMA_START);
		w32(O_DMA_CTRL, DMA_EN);
		uint32_t st = 0;
		for (int d = 0; d < 40; d++) {
			st = r32(O_DMA_STATUS);
			if (!(st & DMA_BUSY) || (st & DMA_DONE)) break;
			k_msleep(1);
		}
		w32(O_DMA_CTRL, DMA_EN | DMA_CLEAR);

		/* Cheap liveness signal: mean of a sparse sample of the DDR frame (don't read every pixel). */
		uint32_t sum = 0, n = 0;
		for (uint32_t i = 0; i < CAP_PIXELS; i += 512) { sum += cam_buf[i]; n++; }
		g_last_mean = n ? sum / n : 0;
		g_frames++;

		/* Service a pending on-demand snapshot request with the frame we just captured. */
		uint32_t req = g_snap_req;
		if (req) {
			stream_snapshot(req, g_snap_w, g_snap_h);
			g_snap_req = 0;
		}
#if ROSE_CAM_AUTOSNAP
		/* Periodic auto-snapshot (high seq bit set to mark it auto vs on-demand) so the GCS shows a
		 * live camera view without needing the uart1 RX uplink. */
		else if ((g_frames % ROSE_CAM_AUTOSNAP) == 0) {
			stream_snapshot(0x80000000u | g_frames, ROSE_CAM_AUTOSNAP_W, ROSE_CAM_AUTOSNAP_H);
		}
#endif

		k_msleep(20);   /* ~cap the camera thread rate + yield to the control loop */
	}
}

/* Low priority (below main / the control loop) so it only runs in the loop's I2C-wait gaps. */
K_THREAD_DEFINE(cam_tid, 8192, cam_thread, NULL, NULL, NULL, 12, 0, 0);

void camera_dma_init(void) { /* thread auto-starts */ }
uint32_t camera_dma_frames(void) { return g_frames; }
uint32_t camera_dma_last_mean(void) { return g_last_mean; }

void camera_dma_request_snapshot(uint32_t seq, int w, int h)
{
	g_snap_w = w;
	g_snap_h = h;
	g_snap_req = seq ? seq : 1;   /* nonzero -> serviced by the camera thread */
}

#else  /* ROSE_CAMERA == 0 */

void camera_dma_init(void) {}
uint32_t camera_dma_frames(void) { return 0; }
uint32_t camera_dma_last_mean(void) { return 0; }
void camera_dma_request_snapshot(uint32_t seq, int w, int h) { (void)seq; (void)w; (void)h; }

#endif
