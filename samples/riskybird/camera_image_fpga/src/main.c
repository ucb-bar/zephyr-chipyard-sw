/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * camera_image_fpga -- capture and reconstruct HM01B0 frames over the FPGA OSPI core, with manual
 * exposure/gain control and an exposure-sweep mode for motion calibration.
 *
 * Pipeline: configure the sensor over I2C (SCCB), stream, then a frame-aligned bounded capture
 * through the OSPI capture core (ospi@10080000); reconstruct via the in-band sof/eol/eof markers.
 *
 * Exposure: AE is off; integration (0x0202/03, in lines) + analog gain (0x0205) + digital gain
 * (0x020e/0f) are programmed manually. Exposure time = integration_lines * line_time, where
 * line_time = frame_time/VTS ~= 59.3 us at MCLKDIV=1 (12.5 MHz MCLK -> ~6.25 MHz PCLK, 30 fps,
 * VTS 0x0232=562). So 560 lines ~= 33 ms (still scene); short integration freezes motion.
 *
 * SWEEP=1 runs a table of (intg,again,dgain), printing mean/min/max/spread per row (no hex dump) so
 * a good motion setting can be picked in one load. SWEEP=0 captures once at MAN_* and dumps the
 * frame as hex rows between <<<PGM W H>>>/<<<END>>> for scripts/camera_pgm_from_hex.py.
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
#define OSPI_CAPCOUNT   0x2c
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

#define MCLK_DIV       1
#define LINE_TIME_NS   59300      /* ~59.3 us/line at MCLKDIV=1 (measured) -- for exp-time reporting */

/* ---- HM01B0 SCCB registers ---- */
#define HM_ADDR         0x24
#define HM_MODEL_ID_H   0x0000
#define HM_MODE_SELECT  0x0100
#define HM_GRP_HOLD     0x0104
#define HM_INTG_H       0x0202
#define HM_INTG_L       0x0203
#define HM_ANA_GAIN     0x0205   /* code: 0=1x 0x10=2x 0x20=4x 0x30=8x 0x40=16x */
#define HM_DGAIN_H      0x020E   /* digital gain 8.8: 0x0100=1.0x */
#define HM_DGAIN_L      0x020F
#define HM_TEST_PATTERN 0x0601
#define HM_AE_CTRL      0x2100
#define HM_MODEL_ID     0x01B0

#define TEST_PATTERN    0        /* 0 = live scene */

/* ---- Reconstruction bounds ---- */
#define MAX_W       340
#define MAX_H       324
#define CAP_PIXELS  104900       /* just under the 104977-beat frame buffer */

/* ======================= exposure calibration =======================
 * SWEEP=1: characterize the table below (no hex dump). SWEEP=0: single capture at MAN_* + dump.
 * DBG=1: print OSPI counters inside capture_frame.
 *
 * MOTION-vs-LIGHT finding (2026-09-02, indoor scene lit by ONE ceiling lamp):
 *   - exposure_time = integration_lines * ~59.3 us  (560 lines ~= 33 ms, the 30fps max).
 *   - Changing integration WHILE streaming stalls the HM01B0's pixel clock -> must standby,
 *     reprogram, then re-stream (see main). Digital gain (0x020e/0f) has a ceiling ~0x0bff;
 *     above it the output goes black. Analog-gain codes: 0=1x 0x10=2x 0x20=4x 0x30=8x 0x40=16x.
 *   - In this dim scene, motion-safe short exposures (2-4 ms) are light-starved: the scene signal
 *     falls below the noise floor and gain only amplifies a flat pedestal -> no detail. Even 15 ms
 *     with 8x analog was flat, while 33 ms with LOW (2x) analog had good detail. Lesson: in low
 *     light, detail needs LONG integration + LOW analog gain; high gain buries low-contrast detail.
 *   - => A true motion calibration needs MORE LIGHT on the scene; then the SWEEP finds the shortest
 *     exposure with mean ~110 that keeps detail. Default below = the good still-scene config. */
#define SWEEP           0
#define DBG             0
#define MAN_INTG        560      /* ~33 ms -- best still-scene detail in one-lamp lighting */
#define MAN_AGAIN       0x10     /* 2x analog (low gain = clean, keeps low-contrast detail) */
#define MAN_DGAIN       0x0300   /* 3x digital */

/* {integration lines, analog-gain code, digital-gain 8.8}. Shorter integration = shorter exposure
 * (freezes motion) but needs more gain. */
struct expo { uint16_t intg; uint8_t again; uint16_t dgain; };
static const struct expo SWEEP_TBL[] __attribute__((unused)) = {
	{ 135, 0x30, 0x0500 },   /* ~8 ms   8x*5x */
	{  68, 0x30, 0x0b00 },   /* ~4 ms   8x*11x */
	{  40, 0x40, 0x0900 },   /* ~2.4 ms 16x*9x */
	{  34, 0x40, 0x0b00 },   /* ~2 ms   16x*11x */
	{  24, 0x40, 0x1000 },   /* ~1.4 ms 16x*16x */
	{  17, 0x40, 0x1800 },   /* ~1 ms   16x*24x */
};
/* ==================================================================== */

static uint8_t img[MAX_H][MAX_W];

static inline void     w32(uintptr_t o, uint32_t v) { *(volatile uint32_t *)(OSPI_BASE + o) = v; }
static inline uint32_t r32(uintptr_t o) { return *(volatile uint32_t *)(OSPI_BASE + o); }

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

static void set_exposure(const struct device *b, uint16_t intg, uint8_t again, uint16_t dgain)
{
	hm_wr(b, HM_AE_CTRL, 0x00);            /* AE off */
	hm_wr(b, HM_GRP_HOLD, 0x01);
	hm_wr(b, HM_INTG_H, (intg >> 8) & 0xff);
	hm_wr(b, HM_INTG_L, intg & 0xff);
	hm_wr(b, HM_ANA_GAIN, again);
	hm_wr(b, HM_DGAIN_H, (dgain >> 8) & 0xff);
	hm_wr(b, HM_DGAIN_L, dgain & 0xff);
	hm_wr(b, HM_GRP_HOLD, 0x00);
}

/* Frame-aligned bounded capture + reconstruct into img[][]. Fills *W,*H and pixel stats. */
static int capture_frame(int *pW, int *pH, uint32_t *pmean, uint8_t *pmin, uint8_t *pmax,
			 uint32_t *poverflow)
{
	/* Fully reset the core each call so a repeated capture never inherits stale armed/done/overflow
	 * state. FVLDCNT advances regardless of enable, so frame-align still works after a disable. */
	w32(OSPI_CTRL, 0);                                     /* stop frontend */
	w32(OSPI_PIXTARGET, CAP_PIXELS);
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);     /* enable + clear status + flush buffer */
	w32(OSPI_CTRL, CTRL_EN);
	uint32_t fv = r32(OSPI_FVLDCNT);
	uint32_t spins = 4000000;
	while (r32(OSPI_FVLDCNT) == fv && spins-- > 0) { /* spin to a frame boundary */ }
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);     /* flush at the boundary */
	w32(OSPI_CTRL, CTRL_EN | CTRL_ARM);                   /* arm */
	w32(OSPI_CTRL, CTRL_EN);
	/* Poll until the bounded capture has actually filled the buffer (robust to exposure/timing),
	 * rather than a fixed sleep. */
	uint32_t pclk0 = r32(OSPI_PCLKCNT), fvld0 = r32(OSPI_FVLDCNT), lvld0 = r32(OSPI_LVLDCNT);
	int t = 400;
	while (r32(OSPI_FIFOCOUNT) < (CAP_PIXELS - 4096) && t-- > 0) { k_msleep(1); }
	*poverflow = (r32(OSPI_FLAGS) >> 1) & 1;
#if DBG
	printk("[dbg] armed->fifo=%u capcnt=%u capstat=0x%x flags=0x%02x dpclk=%u dfvld=%u dlvld=%u\n",
	       r32(OSPI_FIFOCOUNT), r32(OSPI_CAPCOUNT), r32(OSPI_CAPSTAT), r32(OSPI_FLAGS),
	       r32(OSPI_PCLKCNT) - pclk0, r32(OSPI_FVLDCNT) - fvld0, r32(OSPI_LVLDCNT) - lvld0);
#else
	(void)pclk0; (void)fvld0; (void)lvld0;
#endif

	int started = 0, row = 0, col = 0, W = 0;
	uint32_t npix = 0, guard = CAP_PIXELS + 64, sum = 0;
	uint8_t mn = 0xff, mx = 0;
	for (;;) {
		uint32_t d = r32(OSPI_DATA);
		if (!(d & DATA_VALID)) break;
		if (guard-- == 0) break;
		if (d & DATA_EOF) { if (started && row >= 4) break; started = 0; row = 0; col = 0; continue; }
		uint8_t px = (uint8_t)(d & 0xff);
		if (!started) {
			if (d & DATA_SOF) started = 1;
			else { if (d & DATA_EOL) started = 1; continue; }
		}
		if (row >= MAX_H) break;
		if (col < MAX_W) img[row][col] = px;
		col++; npix++; sum += px;
		if (px < mn) mn = px;
		if (px > mx) mx = px;
		if (d & DATA_EOL) { if (col > W) W = col; row++; col = 0; }
	}
	*pW = W; *pH = row; *pmin = mn; *pmax = mx;
	*pmean = npix ? sum / npix : 0;
	return (int)npix;
}

static void dump_hex(int W, int H) __attribute__((unused));
static void dump_hex(int W, int H)
{
	printk("<<<PGM %d %d>>>\n", W, H);
	for (int y = 0; y < H; y++) {
		char hexline[MAX_W * 2 + 2];
		int k = 0;
		static const char hx[] = "0123456789abcdef";
		for (int x = 0; x < W; x++) {
			hexline[k++] = hx[(img[y][x] >> 4) & 0xf];
			hexline[k++] = hx[img[y][x] & 0xf];
		}
		hexline[k] = 0;
		printk("%s\n", hexline);
	}
	printk("<<<END>>>\n");
}

int main(void)
{
	printk("\n=== camera_image_fpga: HM01B0 capture (SWEEP=%d) ===\n", SWEEP);
	if (r32(OSPI_CAPACITY) == 0) { printk("FAIL: no OSPI capture peripheral\n"); return 0; }

	w32(OSPI_MCLKDIV, MCLK_DIV);
	w32(OSPI_PIXTARGET, CAP_PIXELS);
	w32(OSPI_CTRL, CTRL_EN);

	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(bus)) { printk("FAIL: i2c0 not ready\n"); return 0; }
	uint8_t hi = 0, lo = 0;
	if (hm_rd(bus, HM_MODEL_ID_H, &hi) || hm_rd(bus, 0x0001, &lo) ||
	    (((uint16_t)hi << 8) | lo) != HM_MODEL_ID) { printk("FAIL: MODEL_ID 0x%02x%02x\n", hi, lo); return 0; }

	hm_wr(bus, HM_TEST_PATTERN, TEST_PATTERN);
	hm_wr(bus, HM_MODE_SELECT, 0x01);   /* streaming */
	k_msleep(200);

	int W, H, npix; uint32_t mean, ovf; uint8_t mn, mx;

#if SWEEP
	printk("[sweep] line_time~%u ns; cols: intg(lines) exp(us) again dgain -> mean min max spread ovf\n",
	       LINE_TIME_NS);
	for (unsigned i = 0; i < sizeof(SWEEP_TBL) / sizeof(SWEEP_TBL[0]); i++) {
		struct expo e = SWEEP_TBL[i];
		/* Clean restart per entry: changing exposure while streaming throws transient black
		 * frames. Standby -> reprogram -> stream mirrors the known-good boot path. */
		hm_wr(bus, HM_MODE_SELECT, 0x00);   /* standby */
		set_exposure(bus, e.intg, e.again, e.dgain);
		hm_wr(bus, HM_MODE_SELECT, 0x01);   /* stream */
		k_msleep(700);   /* streaming needs to fully resume after an integration change */
		/* retry: a capture right after an exposure change can catch an empty frame */
		npix = 0;
		for (int tries = 0; tries < 3 && npix < 20000; tries++) {
			npix = capture_frame(&W, &H, &mean, &mn, &mx, &ovf);
			if (npix < 20000) k_msleep(120);
		}
		uint32_t exp_us = ((uint32_t)e.intg * LINE_TIME_NS) / 1000;
		printk("  intg=%-4u exp=%-6u again=0x%02x dgain=0x%04x -> mean=%-3u min=%-3u max=%-3u spread=%-3u ovf=%u (%dx%d)\n",
		       e.intg, exp_us, e.again, e.dgain, mean, mn, mx, (unsigned)(mx - mn), ovf, W, H);
	}
	printk("[sweep] done -- pick the shortest exp with mean ~110-140 and good spread, set MAN_* + SWEEP=0\n");
#else
	/* Clean restart: standby, reprogram exposure, stream, long settle. */
	hm_wr(bus, HM_MODE_SELECT, 0x00);
	set_exposure(bus, MAN_INTG, MAN_AGAIN, MAN_DGAIN);
	hm_wr(bus, HM_MODE_SELECT, 0x01);
	k_msleep(700);
	{
		uint32_t a = r32(OSPI_PCLKCNT), b = r32(OSPI_FVLDCNT);
		k_msleep(50);
		printk("[pre] streaming check: dpclk=%u dfvld=%u\n", r32(OSPI_PCLKCNT) - a, r32(OSPI_FVLDCNT) - b);
	}
	npix = capture_frame(&W, &H, &mean, &mn, &mx, &ovf);
	uint32_t exp_us = ((uint32_t)MAN_INTG * LINE_TIME_NS) / 1000;
	printk("[capture] intg=%u exp=%u us again=0x%02x dgain=0x%04x -> %dx%d mean=%u min=%u max=%u spread=%u ovf=%u\n",
	       MAN_INTG, exp_us, MAN_AGAIN, MAN_DGAIN, W, H, mean, mn, mx, (unsigned)(mx - mn), ovf);
	if (npix > 100) dump_hex(W, H);
#endif
	printk("[done]\n");
	for (;;) { k_msleep(1000); }
	return 0;
}
