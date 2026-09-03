/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * camera_image_fpga -- capture and reconstruct a full HM01B0 frame over the FPGA OSPI core.
 *
 * Builds on camera_capture_fpga (which validated the data bus). Here we grab a frame-aligned
 * capture and reconstruct a 2D image:
 *   - The sensor's frame (~326x324 = 105624 beats) slightly exceeds the 104977-beat frame buffer,
 *     and continuous free-running storage overflows, so we do ONE bounded capture armed right at a
 *     frame boundary (busy-poll FVLDCNT, then arm). We get ~319 of 324 rows -- effectively the
 *     whole image, trimmed by a couple of rows top/bottom.
 *   - Reconstruction uses the in-band markers: align to the first sof (or first eol if we armed a
 *     few pixels late), then delimit rows by eol. Robust to exact arm timing.
 *
 * Output:
 *   - an ASCII-art thumbnail to the console for an immediate look, and
 *   - the full frame as hex rows between <<<PGM ...>>> markers, which scripts/camera_pgm_from_hex.py
 *     turns into a PGM/PNG.
 *
 * TEST_PATTERN=0 captures a live scene (default). Set 1 for the deterministic sensor test pattern.
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
#define OSPI_FRAMECNT   0x10
#define OSPI_LASTWIDTH  0x14
#define OSPI_LASTHEIGHT 0x18
#define OSPI_FLAGS      0x1c
#define OSPI_DATA       0x20
#define OSPI_CAPACITY   0x24
#define OSPI_PIXTARGET  0x28
#define OSPI_CAPCOUNT   0x2c
#define OSPI_PCLKCNT    0x30
#define OSPI_FVLDCNT    0x34
#define OSPI_LVLDCNT    0x38
#define OSPI_LASTPIX    0x3c
#define OSPI_CAPSTAT    0x40

#define CTRL_EN     (1u << 0)
#define CTRL_CONT   (1u << 1)
#define CTRL_CLEAR  (1u << 4)
#define CTRL_FLUSH  (1u << 5)
#define CTRL_ARM    (1u << 6)

#define DATA_VALID  (1u << 31)
#define DATA_EOF    (1u << 10)
#define DATA_EOL    (1u << 9)
#define DATA_SOF    (1u << 8)

#define MCLK_DIV    1

#define HM_ADDR             0x24
#define HM_MODEL_ID_H       0x0000
#define HM_MODEL_ID_L       0x0001
#define HM_MODE_SELECT      0x0100
#define HM_TEST_PATTERN     0x0601
#define HM_MODEL_ID         0x01B0
#define HM_MODE_STREAMING   0x01
#define TEST_PATTERN        0     /* 0 = live scene; 1 = sensor test pattern */

/* ---- Exposure / gain (HM01B0 AE + manual regs) ---- */
#define HM_GRP_HOLD     0x0104   /* write 1 to hold grouped params, 0 to release+apply next frame */
#define HM_FRAME_LEN_H  0x0340   /* frame length (VTS) in lines; caps how long integration can be */
#define HM_FRAME_LEN_L  0x0341
#define HM_INTG_H       0x0202   /* coarse integration (exposure) in lines, high byte */
#define HM_INTG_L       0x0203
#define HM_ANA_GAIN     0x0205   /* [6:4] analog gain code: 0=1x,1=2x,2=4x,3=8x,4=16x */
#define HM_DGAIN_H      0x020E   /* digital gain, 8.8: 0x0100 = 1.0x */
#define HM_DGAIN_L      0x020F
#define HM_AE_CTRL      0x2100   /* [0] AE enable */
#define HM_AE_TARGET    0x2101   /* AE target mean brightness (default 0x3C=60) */
#define HM_AE_MAX_INTG_H 0x2105
#define HM_AE_MAX_INTG_L 0x2106
#define HM_MAX_AGAIN    0x210B   /* AE max analog gain code */
#define HM_MAX_DGAIN    0x210D

#define DUMP_REGS       1        /* print current exposure regs */

/* Manual exposure: AE off, low analog gain (clean), brightness from long integration + modest
 * digital gain. The original AE image (analog 1x) had real contrast; cranking analog gain flattened
 * it. So keep analog at 1x and expose longer instead. */
#define MANUAL_EXPOSURE 1
#define MAN_INTG        0x0230   /* integration lines (~560; near frame length VTS ~0x0232=562) */
#define MAN_AGAIN       0x10     /* analog gain code: 0=1x, 0x10=2x (clean-ish) */
#define MAN_DGAIN       0x0300   /* digital gain 8.8: 0x0300 = 3.0x */

/* Reconstruction bounds. Buffer holds < one full 326x324 frame, so cap the capture just under it. */
#define MAX_W       340
#define MAX_H       324
#define CAP_PIXELS  104900        /* just under CAPACITY (104977) */

static uint8_t img[MAX_H][MAX_W];   /* reconstructed frame (in DDR bss) */

static inline void     w32(uintptr_t o, uint32_t v) { *(volatile uint32_t *)(OSPI_BASE + o) = v; }
static inline uint32_t r32(uintptr_t o) { return *(volatile uint32_t *)(OSPI_BASE + o); }

static int hm_wr(const struct device *bus, uint16_t reg, uint8_t val)
{
	uint8_t tx[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val };
	return i2c_write(bus, tx, sizeof(tx), HM_ADDR);
}
static int hm_rd(const struct device *bus, uint16_t reg, uint8_t *val)
{
	uint8_t rb[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
	return i2c_write_read(bus, HM_ADDR, rb, sizeof(rb), val, 1);
}

int main(void)
{
	printk("\n=== camera_image_fpga: HM01B0 full-frame capture + reconstruction ===\n");

	uint32_t cap = r32(OSPI_CAPACITY);
	if (cap == 0) { printk("FAIL: no OSPI capture peripheral\n"); return 0; }

	/* Enable capture block (MCLK) but keep storing gated: set PIXTARGET now so storing only
	 * happens once armed (avoids free-running fill/overflow during setup). */
	w32(OSPI_MCLKDIV, MCLK_DIV);
	w32(OSPI_PIXTARGET, CAP_PIXELS);
	w32(OSPI_CTRL, CTRL_EN);

	/* I2C + MODEL_ID (proves MCLK reaches the sensor). */
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(bus)) { printk("FAIL: i2c0 not ready\n"); return 0; }
	uint8_t hi = 0, lo = 0;
	if (hm_rd(bus, HM_MODEL_ID_H, &hi) || hm_rd(bus, HM_MODEL_ID_L, &lo) ||
	    (((uint16_t)hi << 8) | lo) != HM_MODEL_ID) {
		printk("FAIL: MODEL_ID (got 0x%02x%02x)\n", hi, lo); return 0;
	}
	printk("[cfg] MODEL_ID ok; TEST_PATTERN=%d\n", TEST_PATTERN);

	if (hm_wr(bus, HM_TEST_PATTERN, TEST_PATTERN)) { printk("FAIL: TEST_PATTERN write\n"); return 0; }
	if (hm_wr(bus, HM_MODE_SELECT, HM_MODE_STREAMING)) { printk("FAIL: MODE_SELECT write\n"); return 0; }
	k_msleep(200);   /* let auto-exposure settle for a live scene */

#if DUMP_REGS
	{
		static const uint16_t regs[] = {
			HM_FRAME_LEN_H, HM_FRAME_LEN_L, HM_INTG_H, HM_INTG_L, HM_ANA_GAIN, HM_DGAIN_H, HM_DGAIN_L,
			HM_AE_CTRL, HM_AE_TARGET, 0x2102, HM_AE_MAX_INTG_H, HM_AE_MAX_INTG_L,
			0x2107, 0x2108, 0x2109, 0x210A, HM_MAX_AGAIN, HM_MAX_DGAIN };
		printk("[regs]");
		for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
			uint8_t v = 0; hm_rd(bus, regs[i], &v);
			printk(" %04x=%02x", regs[i], v);
		}
		printk("\n");
	}
#endif

	/* Manual exposure: disable AE and program integration/gain deterministically. */
#if MANUAL_EXPOSURE
	hm_wr(bus, HM_AE_CTRL, 0x00);                 /* AE off */
	hm_wr(bus, HM_GRP_HOLD, 0x01);
	hm_wr(bus, HM_INTG_H, (MAN_INTG >> 8) & 0xff);
	hm_wr(bus, HM_INTG_L, MAN_INTG & 0xff);
	hm_wr(bus, HM_ANA_GAIN, MAN_AGAIN);
	hm_wr(bus, HM_DGAIN_H, (MAN_DGAIN >> 8) & 0xff);
	hm_wr(bus, HM_DGAIN_L, MAN_DGAIN & 0xff);
	hm_wr(bus, HM_GRP_HOLD, 0x00);
	k_msleep(400);                                /* a few frames for the new exposure to take */
	{
		uint8_t ih = 0, il = 0, ag = 0, dh = 0, dl = 0, ae = 0;
		hm_rd(bus, HM_AE_CTRL, &ae);
		hm_rd(bus, HM_INTG_H, &ih); hm_rd(bus, HM_INTG_L, &il);
		hm_rd(bus, HM_ANA_GAIN, &ag); hm_rd(bus, HM_DGAIN_H, &dh); hm_rd(bus, HM_DGAIN_L, &dl);
		printk("[expose] manual: AE=%d intg=0x%02x%02x again=0x%02x dgain=0x%02x%02x\n",
		       ae, ih, il, ag, dh, dl);
	}
#endif

	/* Verify the sensor is actually driving pixels/sync before we arm. */
	uint32_t p0 = r32(OSPI_PCLKCNT), f0 = r32(OSPI_FVLDCNT), l0 = r32(OSPI_LVLDCNT);
	k_msleep(50);
	if (r32(OSPI_PCLKCNT) == p0 || r32(OSPI_FVLDCNT) == f0 || r32(OSPI_LVLDCNT) == l0) {
		printk("FAIL: sensor not streaming (pclk/fvld/lvld not all advancing)\n"); return 0;
	}

	/* Frame-align: busy-poll FVLDCNT, then clear+flush+arm right at a frame boundary. */
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);
	uint32_t fv = r32(OSPI_FVLDCNT);
	uint32_t spins = 2000000;
	while (r32(OSPI_FVLDCNT) == fv && spins-- > 0) { /* tight spin -- catch the frame edge */ }
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);   /* empty buffer at the boundary */
	w32(OSPI_CTRL, CTRL_EN | CTRL_ARM);                  /* arm: storing starts ~now (frame top) */
	w32(OSPI_CTRL, CTRL_EN);

	int timeout_ms = 500;
	while (!(r32(OSPI_CAPSTAT) & 0x2) && timeout_ms-- > 0) { k_msleep(1); }
	uint32_t flags = r32(OSPI_FLAGS), capcnt = r32(OSPI_CAPCOUNT), fifo = r32(OSPI_FIFOCOUNT);
	printk("[cap] captured=%u fifo=%u flags=0x%02x%s lastWxH=%ux%u\n",
	       capcnt, fifo, flags, (flags & 0x02) ? " OVERFLOW" : "", r32(OSPI_LASTWIDTH), r32(OSPI_LASTHEIGHT));

	/* Drain + reconstruct. Align to first sof (or first eol if we armed a hair late), then rows by eol. */
	int started = 0, row = 0, col = 0, W = 0;
	uint32_t npix = 0, guard = CAP_PIXELS + 64;
	for (;;) {
		uint32_t d = r32(OSPI_DATA);
		if (!(d & DATA_VALID)) break;
		if (guard-- == 0) break;
		if (d & DATA_EOF) {
			if (started && row >= 4) break;    /* completed a frame's worth */
			started = 0; row = 0; col = 0;     /* tail of prior frame -- restart on next line */
			continue;
		}
		uint8_t px = (uint8_t)(d & 0xff);
		if (!started) {
			if (d & DATA_SOF) { started = 1; }         /* true frame start; store this pixel */
			else { if (d & DATA_EOL) started = 1;       /* align: next beat begins row 0 */
			       continue; }
		}
		if (row >= MAX_H) break;
		if (col < MAX_W) { img[row][col] = px; }
		col++;
		npix++;
		if (d & DATA_EOL) { if (col > W) { W = col; } row++; col = 0; }
	}
	int H = row;
	printk("[img] reconstructed %d x %d  (%u pixels)\n", W, H, npix);
	if (H < 4 || W < 4) { printk("FAIL: too few rows/cols reconstructed\n"); return 0; }

	/* ASCII-art thumbnail: downsample to <=72 cols, block-average, map to a brightness ramp. */
	{
		const char ramp[] = " .:-=+*#%@";
		const int NR = (int)sizeof(ramp) - 2;   /* index range 0..NR */
		int outW = W < 72 ? W : 72;
		int outH = 30;
		if (outH > H) outH = H;
		printk("[thumb] %dx%d preview:\n", outW, outH);
		for (int oy = 0; oy < outH; oy++) {
			char line[80];
			int r0 = oy * H / outH, r1 = (oy + 1) * H / outH; if (r1 <= r0) r1 = r0 + 1;
			for (int ox = 0; ox < outW; ox++) {
				int c0 = ox * W / outW, c1 = (ox + 1) * W / outW; if (c1 <= c0) c1 = c0 + 1;
				uint32_t sum = 0, n = 0;
				for (int y = r0; y < r1 && y < H; y++)
					for (int x = c0; x < c1 && x < W; x++) { sum += img[y][x]; n++; }
				uint32_t avg = n ? sum / n : 0;
				line[ox] = ramp[(avg * NR) / 255];
			}
			line[outW] = 0;
			printk("|%s|\n", line);
		}
	}

	/* Full frame as hex rows for host-side PGM/PNG conversion. */
	printk("<<<PGM %d %d>>>\n", W, H);
	for (int y = 0; y < H; y++) {
		char hexline[MAX_W * 2 + 2];
		int k = 0;
		for (int x = 0; x < W; x++) {
			static const char hx[] = "0123456789abcdef";
			hexline[k++] = hx[(img[y][x] >> 4) & 0xf];
			hexline[k++] = hx[img[y][x] & 0xf];
		}
		hexline[k] = 0;
		printk("%s\n", hexline);
	}
	printk("<<<END>>>\n");
	printk("[done] image dump complete\n");

	for (;;) { k_msleep(1000); }
	return 0;
}
