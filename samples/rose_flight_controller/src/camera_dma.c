/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * See camera_dma.h. Body compiled only when ROSE_CAMERA_DMA=1; otherwise empty stubs so non-camera
 * builds are unaffected. Requires a DMA-capable OSPI shell (RocketArty200TDroneFullDDRDmaConfig).
 */
#include "camera_dma.h"

#if ROSE_CAMERA_DMA

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

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
#define DMA_ERROR (1u<<2)   /* a denied/corrupt bus response during the transfer */
#define O_DMA_BYTES     0x58 /* bytes the current/last transfer wrote to DDR */

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

/* On-board grab (camera_dma_grab_gray). Everything below is under grab_mtx: the caller posts a
 * request {gen, w, h}; the capture thread answers the gen it captured for (done_*) and gives
 * grab_sem. A caller only accepts the answer to its own gen, and cancels its request on timeout, so a
 * late answer to an abandoned request can never be taken for a newer one. cam_wake cuts the idle poll
 * short so a grab does not wait out the 100 ms snapshot poll. */
static uint8_t  grab_buf[CAMERA_DMA_GRAB_MAX];
static uint32_t grab_req_gen;      /* pending request; 0 = none */
static int      grab_req_w, grab_req_h;
static uint32_t grab_done_gen;     /* request the result below answers */
static bool     grab_done_ok;
static uint32_t grab_done_frame;
static int64_t  grab_done_t_ms;
K_MUTEX_DEFINE(grab_mtx);
K_SEM_DEFINE(grab_sem, 0, 1);
K_SEM_DEFINE(cam_wake, 0, 1);

/* Bound on the scheduler-locked frame-edge spin when a grab is being served. That spin keeps the
 * control loop off the CPU; the pre-sleep leaves ~3 ms to the edge, so a longer wait means the edge
 * was missed or streaming stopped, and the grab fails rather than stalling the loop. */
#ifndef CAM_GRAB_EDGE_SPIN_US
#define CAM_GRAB_EDGE_SPIN_US 5000
#endif

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

/* Output pixel (ox, oy) of a w x h downsample of cam_buf. */
static uint8_t box_avg(int ox, int oy, int w, int h)
{
	int sy0 = oy * SRC_H / h, sy1 = (oy + 1) * SRC_H / h;
	if (sy1 <= sy0) { sy1 = sy0 + 1; }
	/* Force an EVEN-length row span so the block holds equal even/odd source rows -- otherwise a
	 * 3-row block (2:1 parity) leaves a residual of the period-2 row FPN. See column note below. */
	if (((sy1 - sy0) & 1) && sy1 < SRC_H) { sy1++; }
	int sx0 = ox * SRC_W / w, sx1 = (ox + 1) * SRC_W / w;
	if (sx1 <= sx0) { sx1 = sx0 + 1; }
	if (((sx1 - sx0) & 1) && sx1 < SRC_W) { sx1++; }
	/* Box-average the whole source block into one output pixel (not nearest-neighbour). The
	 * HM01B0 has period-2 fixed-pattern noise on BOTH axes -- even/odd row offsets (~92 vs 74)
	 * AND even/odd column offsets (per-column ADCs) -- which together read as a checkerboard
	 * grid. Averaging over an EVEN number of rows and columns puts equal even/odd members in
	 * every block, cancelling both; a plain variable 2-or-3 span left a residual grid. It also
	 * anti-aliases the downscale. (A single-row/col pick would alias the FPN into stripes.) */
	uint32_t sum = 0, cnt = 0;
	for (int sy = sy0; sy < sy1; sy++) {
		const uint8_t *row = &cam_buf[(uint32_t)sy * SRC_W];
		for (int sx = sx0; sx < sx1; sx++) { sum += row[sx]; cnt++; }
	}
	return (uint8_t)(sum / cnt);
}

/* Answer grab request gen (if it is still the pending one): downsample cam_buf on success. */
static void grab_answer(uint32_t gen, bool ok, uint32_t frame, int64_t t_ms, int w, int h)
{
	k_mutex_lock(&grab_mtx, K_FOREVER);
	if (grab_req_gen == gen) {
		if (ok) {
			for (int oy = 0; oy < h; oy++) {
				for (int ox = 0; ox < w; ox++) { grab_buf[oy * w + ox] = box_avg(ox, oy, w, h); }
			}
		}
		grab_done_gen = gen;
		grab_done_ok = ok;
		grab_done_frame = frame;
		grab_done_t_ms = t_ms;
	}
	k_mutex_unlock(&grab_mtx);
	k_sem_give(&grab_sem);
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
		for (int ox = 0; ox < w; ox++) {
			raw[rn++] = box_avg(ox, oy, w, h);
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

	/* Measure the FVLDCNT tick period (ms) so the aligned capture below can pre-sleep most of a frame
	 * preemptibly and hold the scheduler lock only for the last few ms while it catches the exact edge. */
	int tick_ms;
	{
		uint32_t c0 = r32(O_FVLDCNT), t0 = k_uptime_get_32();
		while (r32(O_FVLDCNT) - c0 < 10 && k_uptime_get_32() - t0 < 1000) { k_busy_wait(100); }
		uint32_t dc = r32(O_FVLDCNT) - c0, dt = k_uptime_get_32() - t0;
		tick_ms = (dc > 0) ? (int)(dt / dc) : 16;
		if (tick_ms < 2) { tick_ms = 2; }
	}

	const uint32_t grab_spin_cyc = k_us_to_cyc_ceil32(CAM_GRAB_EDGE_SPIN_US);
	uint32_t iter = 0;
	for (;;) {
		/* The aligned capture briefly locks the scheduler -- so only capture when there's actually a
		 * snapshot to send or an on-board grab waiting. */
		iter++;
		uint32_t req = g_snap_req;
		uint32_t ggen = 0;
		int gw = 0, gh = 0;
		k_mutex_lock(&grab_mtx, K_FOREVER);
		if (grab_req_gen != 0 && grab_req_gen != grab_done_gen) {
			ggen = grab_req_gen; gw = grab_req_w; gh = grab_req_h;
		}
		k_mutex_unlock(&grab_mtx);
		bool grab = ggen != 0;
		int sw = 0, sh = 0;
		if (req)                                  { sw = g_snap_w;            sh = g_snap_h; }
#if ROSE_CAM_AUTOSNAP
		else if ((iter % ROSE_CAM_AUTOSNAP) == 0) { sw = ROSE_CAM_AUTOSNAP_W; sh = ROSE_CAM_AUTOSNAP_H; }
#endif
		if (sw == 0 && !grab) { k_sem_take(&cam_wake, K_MSEC(100)); continue; }

		/* --- Frame-aligned bounded capture into the OSPI frame buffer. ---
		 * The capture core starts filling the instant it is ARMed, so ARM must land right at a frame
		 * edge or the buffer begins mid-frame and the image comes out cyclically ROLLED (the corner
		 * lands in the middle). This low-prio thread was being preempted between detecting the FVLD
		 * edge and arming -- a variable ms of pixels stream through in that gap -> intermittent roll.
		 * Fix: pre-sleep most of the frame preemptibly, then k_sched_lock (interrupts stay live) only
		 * while spinning for the exact edge and arming, so no thread can slip in between. */
		/* DMA_CLEAR does not abort a transfer: one that timed out earlier may still be collecting,
		 * and would swallow this frame's pixels and report its own completion as ours. A grab
		 * never starts on a busy engine. */
		if (grab && (r32(O_DMA_STATUS) & DMA_BUSY)) {
			grab_answer(ggen, false, 0, 0, gw, gh);
			if (sw == 0) { k_msleep(tick_ms); continue; }
			grab = false;
		}
		w32(O_PIXTARGET, CAP_PIXELS);
		uint32_t fv = r32(O_FVLDCNT);
		for (int g = 0; g < 200 && r32(O_FVLDCNT) == fv; g++) { k_msleep(1); }   /* phase-lock to a frame */
		if (tick_ms > 4) { k_msleep(tick_ms - 3); }                             /* skip most of it */
		k_sched_lock();
		fv = r32(O_FVLDCNT);
		const uint32_t spin_c0 = k_cycle_get_32();
		for (int g = 0; g < 500000 && r32(O_FVLDCNT) == fv; g++) {                /* catch the exact edge */
			if (grab && k_cycle_get_32() - spin_c0 >= grab_spin_cyc) { break; }
		}
		const bool edge_ok = r32(O_FVLDCNT) != fv;
		if (!edge_ok && sw == 0) {
			k_sched_unlock();                   /* grab only: fail it rather than arm mid-frame */
			grab_answer(ggen, false, 0, 0, gw, gh);
			continue;
		}
		w32(O_CTRL, CTRL_EN | CTRL_CLR | CTRL_FLU);                             /* drop any pre-edge pixels */
		w32(O_CTRL, CTRL_EN | CTRL_ARM);                                        /* arm at the frame start */
		w32(O_CTRL, CTRL_EN);
		const int64_t t_capture = k_uptime_get();   /* the frame starts now, not when we resume */
		k_sched_unlock();
		for (int t = 0; t < 60 && r32(O_FIFOCOUNT) < (CAP_PIXELS - 4096); t++) { k_msleep(1); }
		const bool fill_ok = r32(O_FIFOCOUNT) >= (CAP_PIXELS - 4096);

		/* Kick the DMA to drain the frame buffer to DDR (HW moves the bytes; CPU just waits). */
		w32(O_DMA_ADDR_LO, (uint32_t)(dst & 0xffffffffu));
		w32(O_DMA_ADDR_HI, (uint32_t)(dst >> 32));
		w32(O_DMA_LEN, CAP_PIXELS);
		w32(O_DMA_CTRL, DMA_EN | DMA_START);
		w32(O_DMA_CTRL, DMA_EN);
		uint32_t st = 0;
		for (int d = 0; d < 40; d++) {
			st = r32(O_DMA_STATUS);
			/* A grab waits for DONE itself: not-busy can also be read before the start is taken. */
			if ((!grab && !(st & DMA_BUSY)) || (st & DMA_DONE)) break;
			k_msleep(1);
		}
		/* A usable frame: the transfer finished, no bus error, and it stopped at the length cap -- an
		 * early EOF leaves the tail of cam_buf holding the previous frame. */
		const bool dma_ok = (st & DMA_DONE) && !(st & DMA_ERROR) &&
				    r32(O_DMA_BYTES) == CAP_PIXELS;
		w32(O_DMA_CTRL, DMA_EN | DMA_CLEAR);

		/* Cheap liveness signal: mean of a sparse sample of the DDR frame (don't read every pixel). */
		uint32_t sum = 0, n = 0;
		for (uint32_t i = 0; i < CAP_PIXELS; i += 512) { sum += cam_buf[i]; n++; }
		g_last_mean = n ? sum / n : 0;
		g_frames++;

		if (grab) {
			/* Only a frame that was edge-aligned, filled and fully DMA'd is handed to a consumer. */
			grab_answer(ggen, edge_ok && fill_ok && dma_ok, g_frames, t_capture, gw, gh);
		}

		/* Send it. High seq bit marks an auto-snapshot vs an on-demand one. */
		if (sw != 0) { stream_snapshot(req ? req : (0x80000000u | g_frames), sw, sh); }
		if (req) { g_snap_req = 0; }
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

int32_t camera_dma_grab_gray(uint8_t *dst, int w, int h, int32_t timeout_ms, int64_t *t_capture_ms)
{
	static uint32_t gen;

	if (w < 1 || h < 1 || w > SRC_W || h > SRC_H || w * h > CAMERA_DMA_GRAB_MAX) {
		return -EINVAL;
	}
	if (++gen == 0) { gen = 1; }
	k_mutex_lock(&grab_mtx, K_FOREVER);
	grab_req_gen = gen;
	grab_req_w = w;
	grab_req_h = h;
	k_mutex_unlock(&grab_mtx);
	k_sem_give(&cam_wake);

	const int64_t deadline = k_uptime_get() + timeout_ms;
	for (;;) {
		const int64_t left = deadline - k_uptime_get();
		if (left <= 0 || k_sem_take(&grab_sem, K_MSEC(left)) != 0) {
			k_mutex_lock(&grab_mtx, K_FOREVER);
			if (grab_req_gen == gen) { grab_req_gen = 0; }   /* cancel: a late answer is dropped */
			k_mutex_unlock(&grab_mtx);
			return -EAGAIN;
		}
		k_mutex_lock(&grab_mtx, K_FOREVER);
		if (grab_done_gen != gen) {          /* an answer to an older, abandoned request */
			k_mutex_unlock(&grab_mtx);
			continue;
		}
		const bool ok = grab_done_ok;
		const uint32_t frame = grab_done_frame;
		if (ok) {
			memcpy(dst, grab_buf, (size_t)w * (size_t)h);
			*t_capture_ms = grab_done_t_ms;
		}
		k_mutex_unlock(&grab_mtx);
		return ok ? (int32_t)frame : -EIO;
	}
}

#else  /* ROSE_CAMERA_DMA == 0 */

void camera_dma_init(void) {}
uint32_t camera_dma_frames(void) { return 0; }
uint32_t camera_dma_last_mean(void) { return 0; }
void camera_dma_request_snapshot(uint32_t seq, int w, int h) { (void)seq; (void)w; (void)h; }
int32_t camera_dma_grab_gray(uint8_t *dst, int w, int h, int32_t timeout_ms, int64_t *t_capture_ms)
{
	(void)dst; (void)w; (void)h; (void)timeout_ms; (void)t_capture_ms;
	return -1;
}

#endif
