/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * camera_dma_fpga -- HM01B0 capture with the DMA-capable OSPI core (RocketArty200TDroneFullDDRDmaConfig).
 *
 * Instead of the CPU popping ~105k pixels through DATA (0x20) at ~1 MB/s, hardware DMAs the frame
 * straight into a DDR buffer, freeing the CPU. This app configures the sensor (Himax reference,
 * clock-safe subset -- same as camera_image_fpga), does a frame-aligned bounded capture into the
 * OSPI frame buffer, then kicks the DMA to drain it to DDR, and times both the DMA transfer and the
 * sustained frame rate.
 *
 * DMA register map (offsets from OSPI base 0x10080000), per the peripheral:
 *   0x44 DMA_ADDR_LO, 0x48 DMA_ADDR_HI  (64-bit DDR target, 8-byte aligned)
 *   0x4c DMA_LEN     (bytes; 0 = drain to in-band EOF)
 *   0x50 DMA_CTRL    [0]=enable(route drain to DMA) [1]=start(W1P) [2]=auto [3]=clear(W1P)
 *   0x54 DMA_STATUS  [0]=busy [1]=done [2]=error [3]=sawEof [7:4]=state
 *   0x58 DMA_BYTES   (bytes written, RO)
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/cache.h>
#include <stdint.h>

#define OSPI_BASE       0x10080000UL
#define OSPI_CTRL       0x00
#define OSPI_GEOM       0x04
#define OSPI_MCLKDIV    0x08
#define OSPI_FIFOCOUNT  0x0c
#define OSPI_CAPACITY   0x24
#define OSPI_PIXTARGET  0x28
#define OSPI_FVLDCNT    0x34
#define OSPI_DMA_ADDR_LO 0x44
#define OSPI_DMA_ADDR_HI 0x48
#define OSPI_DMA_LEN     0x4c
#define OSPI_DMA_CTRL    0x50
#define OSPI_DMA_STATUS  0x54
#define OSPI_DMA_BYTES   0x58

#define CTRL_EN     (1u << 0)
#define CTRL_CLEAR  (1u << 4)
#define CTRL_FLUSH  (1u << 5)
#define CTRL_ARM    (1u << 6)

#define DMA_EN      (1u << 0)
#define DMA_START   (1u << 1)
#define DMA_AUTO    (1u << 2)
#define DMA_CLEAR   (1u << 3)
#define DMA_S_BUSY  (1u << 0)
#define DMA_S_DONE  (1u << 1)
#define DMA_S_ERR   (1u << 2)

#define MCLK_DIV    1
#define CPU_HZ      50000000UL
#define HM_ADDR     0x24
#define HM_MODEL_ID 0x01B0
#define CAP_PIXELS  104000              /* frame-aligned bounded capture that fits the buffer */

static uint8_t __aligned(64) framebuf[128 * 1024];   /* DDR sink for the DMA */

/* Himax reference init (clock-safe subset; see camera_image_fpga). */
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
static inline uint64_t rdcycle(void) { uint64_t c; __asm__ volatile("csrr %0, cycle" : "=r"(c)); return c; }

static int hm_wr(const struct device *b, uint16_t reg, uint8_t val)
{
	uint8_t tx[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val };
	return i2c_write(b, tx, sizeof(tx), HM_ADDR);
}
static int hm_rd(const struct device *b, uint16_t reg, uint8_t *val)
{
	uint8_t rb[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
	return i2c_write_read(b, HM_ADDR, rb, sizeof(rb), val, 1);
}

/* Fill the OSPI frame buffer with a frame-aligned bounded capture (as in camera_image_fpga). */
static void capture_to_framebuffer(void)
{
	w32(OSPI_PIXTARGET, CAP_PIXELS);
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);
	uint32_t fv = r32(OSPI_FVLDCNT);
	uint32_t spins = 4000000;
	while (r32(OSPI_FVLDCNT) == fv && spins-- > 0) { }
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);
	w32(OSPI_CTRL, CTRL_EN | CTRL_ARM);
	w32(OSPI_CTRL, CTRL_EN);
	int t = 400;
	while (r32(OSPI_FIFOCOUNT) < (CAP_PIXELS - 4096) && t-- > 0) { k_msleep(1); }
}

int main(void)
{
	printk("\n=== camera_dma_fpga: HM01B0 capture via OSPI DMA ===\n");
	uint32_t cap = r32(OSPI_CAPACITY);
	printk("[1] OSPI capacity=%u beats; DMA sink buf @ %p\n", cap, (void *)framebuf);
	if (cap == 0) { printk("FAIL: no OSPI capture peripheral\n"); return 0; }

	w32(OSPI_MCLKDIV, MCLK_DIV);
	w32(OSPI_CTRL, CTRL_EN);

	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(bus)) { printk("FAIL: i2c0 not ready\n"); return 0; }
	uint8_t hi = 0, lo = 0;
	if (hm_rd(bus, 0x0000, &hi) || hm_rd(bus, 0x0001, &lo) ||
	    (((uint16_t)hi << 8) | lo) != HM_MODEL_ID) { printk("FAIL: MODEL_ID 0x%02x%02x\n", hi, lo); return 0; }

	hm_wr(bus, 0x0103, 0x01); k_msleep(50); hm_wr(bus, 0x0100, 0x00);   /* SW reset + standby */
	for (unsigned i = 0; i < sizeof(HM_INIT) / sizeof(HM_INIT[0]); i++) hm_wr(bus, HM_INIT[i].r, HM_INIT[i].v);
	hm_wr(bus, 0x0601, 0x00);       /* test pattern off (live) */
	hm_wr(bus, 0x0100, 0x01);       /* stream */
	k_msleep(1200);
	printk("[2] sensor configured + streaming\n");

	/* Check the DMA path exists: DMA_STATUS should be a real register (not read-as-0xffffffff). */
	uint32_t st0 = r32(OSPI_DMA_STATUS);
	printk("[3] DMA_STATUS initial = 0x%08x\n", st0);

	/* --- One DMA-drained capture --- */
	capture_to_framebuffer();
	uint32_t fifo = r32(OSPI_FIFOCOUNT);
	printk("[4] frame buffer filled: fifo=%u beats\n", fifo);

	uint32_t nbytes = CAP_PIXELS;
	uintptr_t dst = (uintptr_t)framebuf;
	w32(OSPI_DMA_ADDR_LO, (uint32_t)(dst & 0xffffffffu));
	w32(OSPI_DMA_ADDR_HI, (uint32_t)(dst >> 32));
	w32(OSPI_DMA_LEN, nbytes);
	w32(OSPI_DMA_CTRL, DMA_EN);                 /* route drain to DMA */
	uint64_t c0 = rdcycle();
	w32(OSPI_DMA_CTRL, DMA_EN | DMA_START);     /* kick */
	w32(OSPI_DMA_CTRL, DMA_EN);
	int t = 2000; uint32_t st;
	while (((st = r32(OSPI_DMA_STATUS)) & DMA_S_BUSY) && !(st & DMA_S_DONE) && t-- > 0) { k_busy_wait(50); }
	uint64_t dc = rdcycle() - c0;
	uint32_t us = (uint32_t)(dc * 1000000ULL / CPU_HZ);
	uint32_t bytes = r32(OSPI_DMA_BYTES);
	printk("[5] DMA done: status=0x%08x bytes=%u in %u us -> %u.%02u MB/s (err=%d)\n",
	       st, bytes, us, us ? (uint32_t)((uint64_t)bytes * 100 / us) : 0,
	       us ? (uint32_t)((uint64_t)bytes * 100 / us) % 100 : 0, (st & DMA_S_ERR) ? 1 : 0);

	/* Verify the pixels actually landed in DDR (invalidate cache first for coherence). */
#ifdef CONFIG_DCACHE
	sys_cache_data_invd_range(framebuf, nbytes);
#endif
	uint32_t sum = 0, nz = 0; uint8_t mn = 0xff, mx = 0;
	for (uint32_t i = 0; i < nbytes; i++) {
		uint8_t p = framebuf[i];
		sum += p; if (p) nz++;
		if (p < mn) mn = p; if (p > mx) mx = p;
	}
	printk("[6] DDR buffer: mean=%u range=%u..%u nonzero=%u/%u %s\n",
	       sum / nbytes, mn, mx, nz, nbytes,
	       (nz > nbytes / 2 && mx > mn) ? "-> real image DMA'd to DDR" : "-> suspicious (check DMA)");

	/* --- Sustained rate: how many DMA captures per second --- */
	uint32_t nframes = 0; uint64_t t0 = rdcycle();
	while ((uint32_t)((rdcycle() - t0) * 1000000ULL / CPU_HZ) < 1000000u && nframes < 200) {
		capture_to_framebuffer();
		w32(OSPI_DMA_ADDR_LO, (uint32_t)(dst & 0xffffffffu));
		w32(OSPI_DMA_LEN, nbytes);
		w32(OSPI_DMA_CTRL, DMA_EN | DMA_START);
		w32(OSPI_DMA_CTRL, DMA_EN);
		int tt = 2000;
		while (((st = r32(OSPI_DMA_STATUS)) & DMA_S_BUSY) && !(st & DMA_S_DONE) && tt-- > 0) { k_busy_wait(50); }
		w32(OSPI_DMA_CTRL, DMA_EN | DMA_CLEAR);
		nframes++;
	}
	uint32_t span_us = (uint32_t)((rdcycle() - t0) * 1000000ULL / CPU_HZ);
	printk("[7] sustained: %u DMA captures in %u us -> %u fps (capture+DMA bound)\n",
	       nframes, span_us, span_us ? (uint32_t)((uint64_t)nframes * 1000000ULL / span_us) : 0);
	printk("[done]\n");
	for (;;) { k_msleep(1000); }
	return 0;
}
