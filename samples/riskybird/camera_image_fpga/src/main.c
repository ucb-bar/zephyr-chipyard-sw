/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * camera_image_fpga -- HM01B0 capture over the FPGA OSPI core, using the Himax reference QVGA
 * register init (the config lineage the Crazyflie AI-deck / OpenMV / SparkFun drivers use), plus
 * camera throughput characterization.
 *
 * Why the full init: the minimal "test-pattern + MODE_SELECT" config relied on power-on defaults and
 * left black-level calibration off, so at high gain the dark pedestal dominated (flat images). The
 * reference init turns on BLC (0x1000/0x1003/0x1006), sets explicit 8-bit output (BIT_CONTROL
 * 0x3059=0x02), the oscillator/PCLK divider (0x3060), and QVGA window mode (0x3010=0x01, 320x240).
 * Gain limits per the datasheet: analog max 8x (0x0205 code 0x30, MAX_AGAIN 0x2108=0x03), digital
 * max ~4x (0x020E/0F, MAX_DGAIN 0x210B).
 *
 * Throughput: prints sensor frame rate (from FVLDCNT) and the CPU drain rate (cycle-timed MMIO
 * reads of DATA @ 0x20). The current OSPI core has NO DMA -- the CPU must pop every pixel from the
 * frame buffer through DATA, so the drain rate is the ceiling for sustained (streaming) capture.
 *
 * Build (DroneFullDDR shell):
 *   west build -b chipyard_riscv64 -d build_camimg samples/riskybird/camera_image_fpga -- \
 *     -DDTC_OVERLAY_FILE="<elf>/overlays/fpga-common.overlay;<elf>/overlays/arty200t.overlay" \
 *     -DCONFIG_UART_HTIF=n -DCONFIG_UART_SIFIVE=y -DCONFIG_UART_SIFIVE_PORT_0=y \
 *     -DCONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=50000
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* ---- OSPI capture peripheral register map (base from generated DTS: ospi@10080000). ---- */
#define OSPI_BASE       0x10080000UL
#define OSPI_CTRL       0x00
#define OSPI_GEOM       0x04
#define OSPI_MCLKDIV    0x08
#define OSPI_FIFOCOUNT  0x0c
#define OSPI_FLAGS      0x1c
#define OSPI_DATA       0x20
#define OSPI_CAPACITY   0x24
#define OSPI_PIXTARGET  0x28
#define OSPI_PCLKCNT    0x30
#define OSPI_FVLDCNT    0x34
#define OSPI_LVLDCNT    0x38
#define OSPI_CAPSTAT    0x40

#define CTRL_EN     (1u << 0)
#define CTRL_CLEAR  (1u << 4)
#define CTRL_FLUSH  (1u << 5)
#define CTRL_ARM    (1u << 6)

#define DATA_VALID  (1u << 31)
#define DATA_EOF    (1u << 10)
#define DATA_EOL    (1u << 9)
#define DATA_SOF    (1u << 8)

#define MCLK_DIV       1          /* MCLK = 50/(2*(1+1)) = 12.5 MHz */
#define CPU_HZ         50000000UL

#define HM_ADDR         0x24
#define HM_MODE_SELECT  0x0100
#define HM_MODEL_ID     0x01B0
#define TEST_PATTERN_REG 0x0601
#define TEST_PATTERN     0        /* 0 = live scene */

#define MAX_W       340
#define MAX_H       324
#define CAP_PIXELS  104900        /* ~one full 326x324 frame + markers, < buffer 104977 */

static uint8_t img[MAX_H][MAX_W];

/* Himax HM01B0 reference init (AI-deck / OpenMV / SparkFun lineage), CLOCK-SAFE subset: the full
 * reference sets OSC_CLK_DIV(0x3060)/QVGA-window(0x3010)/VTS/HTS/BIT_CONTROL tuned for the AI-deck's
 * master clock, and applying those to our 12.5 MHz MCLK killed frame output (PCLK dropped 10x,
 * FVLD/LVLD stopped). So we keep our known-good clock + full-frame geometry and apply only the
 * clock-INDEPENDENT quality regs: black-level calibration (the big win -- subtracts the dark
 * pedestal so gain amplifies signal not noise), analog/ADC tuning, and the AE block. */
static const struct { uint16_t r; uint8_t v; } HM_INIT[] = {
	/* black-level calibration + defect-pixel + dgain floor */
	{0x1003,0x08},{0x1007,0x08},
	{0x1000,0x43},{0x1001,0x40},{0x1002,0x32},{0x0350,0x7F},{0x1006,0x01},{0x1008,0x00},
	{0x1009,0xA0},{0x100A,0x60},{0x100B,0x90},{0x100C,0x40},
	/* analog/ADC tuning (no pixel-clock division) */
	{0x3044,0x0A},{0x3045,0x00},{0x3047,0x0A},{0x3050,0xC0},{0x3051,0x42},{0x3052,0x50},
	{0x3053,0x00},{0x3054,0x03},{0x3055,0xF7},{0x3056,0xF8},{0x3057,0x29},{0x3058,0x1F},
	{0x3064,0x00},{0x3065,0x04},
	{0x3022,0x01},{0x1012,0x01},
	/* auto-exposure: enable, target, converge, limits (analog max 8x, digital max) */
	{0x2000,0x07},
	{0x2100,0x01},{0x2101,0x40},{0x2102,0x0A},{0x2103,0x03},{0x2104,0x07},
	{0x2105,0x02},{0x2106,0x20},   /* MAX_INTG ~0x0220 (< our ~562-line frame) */
	{0x2108,0x03},{0x2109,0x03},   /* MAX_AGAIN = 8x */
	{0x210B,0x80},          /* MAX_DGAIN */
	{0x0101,0x01},          /* IMG_ORIENTATION */
	{0x0104,0x01},          /* GRP_PARAM_HOLD -> commit */
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

int main(void)
{
	printk("\n=== camera_image_fpga: HM01B0 reference-config capture + throughput ===\n");
	uint32_t cap = r32(OSPI_CAPACITY);
	if (cap == 0) { printk("FAIL: no OSPI capture peripheral\n"); return 0; }

	w32(OSPI_MCLKDIV, MCLK_DIV);
	w32(OSPI_PIXTARGET, CAP_PIXELS);
	w32(OSPI_CTRL, CTRL_EN);

	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(bus)) { printk("FAIL: i2c0 not ready\n"); return 0; }
	uint8_t hi = 0, lo = 0;
	if (hm_rd(bus, 0x0000, &hi) || hm_rd(bus, 0x0001, &lo) ||
	    (((uint16_t)hi << 8) | lo) != HM_MODEL_ID) { printk("FAIL: MODEL_ID 0x%02x%02x\n", hi, lo); return 0; }

	/* SW-reset first: restores all regs (incl. the clock divider 0x3060) to power-on defaults, so a
	 * prior run that changed clock/timing can't leave the sensor in a stalled state. */
	hm_wr(bus, 0x0103, 0x01);
	k_msleep(50);
	hm_wr(bus, HM_MODE_SELECT, 0x00);   /* standby while we configure */

	/* Apply the clock-safe reference regs, then test pattern + stream. */
	int nerr = 0;
	for (unsigned i = 0; i < sizeof(HM_INIT) / sizeof(HM_INIT[0]); i++) {
		nerr += (hm_wr(bus, HM_INIT[i].r, HM_INIT[i].v) != 0);
	}
	hm_wr(bus, TEST_PATTERN_REG, TEST_PATTERN);
	hm_wr(bus, HM_MODE_SELECT, 0x01);   /* streaming */
	printk("[cfg] applied %u reference regs (%d NACKs), test_pattern=%d, streaming\n",
	       (unsigned)(sizeof(HM_INIT) / sizeof(HM_INIT[0])), nerr, TEST_PATTERN);
	k_msleep(1500);   /* AE convergence */

	/* --- Throughput: sensor frame/line rate from the diagnostic counters --- */
	uint32_t f0 = r32(OSPI_FVLDCNT), l0 = r32(OSPI_LVLDCNT), p0 = r32(OSPI_PCLKCNT);
	k_msleep(500);
	uint32_t dfvld = r32(OSPI_FVLDCNT) - f0, dlvld = r32(OSPI_LVLDCNT) - l0, dpclk = r32(OSPI_PCLKCNT) - p0;
	printk("[rate] over 500ms: frames=%u (%u fps), lines=%u (%u/s), pclk=%u (%u kHz)\n",
	       dfvld, dfvld * 2, dlvld, dlvld * 2, dpclk, dpclk / 500);

	/* --- Frame-aligned bounded capture --- */
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);
	uint32_t fv = r32(OSPI_FVLDCNT);
	uint32_t spins = 4000000;
	while (r32(OSPI_FVLDCNT) == fv && spins-- > 0) { }
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);
	w32(OSPI_CTRL, CTRL_EN | CTRL_ARM);
	w32(OSPI_CTRL, CTRL_EN);
	int t = 500;
	while (r32(OSPI_FIFOCOUNT) < (CAP_PIXELS - 4096) && t-- > 0) { k_msleep(1); }

	/* --- Drain, timing the CPU MMIO pops to characterize drain throughput --- */
	int started = 0, row = 0, col = 0, W = 0;
	uint32_t npix = 0, guard = CAP_PIXELS + 64, sum = 0, reads = 0;
	uint8_t mn = 0xff, mx = 0;
	uint64_t c0 = rdcycle();
	for (;;) {
		uint32_t d = r32(OSPI_DATA);
		reads++;
		if (!(d & DATA_VALID)) break;
		if (guard-- == 0) break;
		if (d & DATA_EOF) { if (started && row >= 4) break; started = 0; row = 0; col = 0; continue; }
		uint8_t px = (uint8_t)(d & 0xff);
		if (!started) { if (d & DATA_SOF) started = 1; else { if (d & DATA_EOL) started = 1; continue; } }
		if (row >= MAX_H) break;
		if (col < MAX_W) img[row][col] = px;
		col++; npix++; sum += px;
		if (px < mn) mn = px;
		if (px > mx) mx = px;
		if (d & DATA_EOL) { if (col > W) W = col; row++; col = 0; }
	}
	uint64_t dc = rdcycle() - c0;
	int H = row;
	uint32_t us = (uint32_t)(dc * 1000000ULL / CPU_HZ);            /* drain time in microseconds */
	uint32_t fps = dfvld * 2;
	/* Drain rate: pixel bytes moved per microsecond == MB/s. Report x100 for two decimals. */
	uint32_t drain_x100  = us ? (uint32_t)((uint64_t)npix * 100 / us) : 0;
	uint32_t ns_per_read = reads ? (uint32_t)(dc * 20ULL / reads) : 0;   /* 20 ns/cycle @ 50 MHz */
	/* Sensor rate: pixels/frame * fps, in MB/s x100. */
	uint32_t sensor_x100 = (uint32_t)((uint64_t)npix * fps / 10000);
	printk("[img] %dx%d  %u pixels  mean=%u range=%u..%u\n", W, H, npix, npix ? sum / npix : 0, mn, mx);
	printk("[drain] %u beats in %u us -> %u.%02u MB/s pixel payload, %u ns/read\n",
	       reads, us, drain_x100 / 100, drain_x100 % 100, ns_per_read);
	printk("[tput] sensor %u.%02u MB/s (%dx%d @ %u fps) vs CPU drain %u.%02u MB/s -> %s\n",
	       sensor_x100 / 100, sensor_x100 % 100, W, H, fps, drain_x100 / 100, drain_x100 % 100,
	       (drain_x100 > sensor_x100) ? "CPU keeps up at this fps" : "CPU CANNOT sustain -> DMA needed");

	/* Dump the frame as hex rows. */
	if (npix > 100) {
		printk("<<<PGM %d %d>>>\n", W, H);
		for (int y = 0; y < H; y++) {
			char hexline[MAX_W * 2 + 2]; int k = 0;
			static const char hx[] = "0123456789abcdef";
			for (int x = 0; x < W; x++) { hexline[k++] = hx[(img[y][x] >> 4) & 0xf]; hexline[k++] = hx[img[y][x] & 0xf]; }
			hexline[k] = 0; printk("%s\n", hexline);
		}
		printk("<<<END>>>\n");
	}
	printk("[done]\n");
	for (;;) { k_msleep(1000); }
	return 0;
}
