/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * camera_capture_fpga -- HM01B0 pixel capture over the FPGA OSPI capture peripheral.
 *
 * Two halves, matching the bring-up the user asked for:
 *   1. Configure the sensor over I2C (SCCB @ 0x24): drive MCLK, verify MODEL_ID, put it into
 *      8-bit gated-DVP streaming with the internal test pattern. The HM01B0 powers up in 8-bit
 *      DVP with FVLD (frame valid) / LVLD (line valid) gating already the default -- so a correct
 *      config is: test pattern on (0x0601), then MODE_SELECT=streaming (0x0100), which is the
 *      "capture on I2C command" that starts the sensor clocking pixels out.
 *   2. Read the data lines: use the OSPI capture core (ospi@10080000) to grab one full frame into
 *      its block-RAM frame buffer, then drain it over MMIO and validate. The core samples
 *      D[7:0]/FVLD/LVLD in the PCLK domain and tags each beat with sof/eol/eof, so draining proves
 *      the parallel bus and the FVLD/LVLD framing end-to-end.
 *
 * The test pattern is used first on purpose: it makes the *sensor* generate deterministic,
 * varying pixel values, so a correct readout is distinguishable from a stuck/floating bus
 * (the classic "all pixels identical" failure). Switch TEST_PATTERN to 0 for a live image.
 *
 * Build (DroneFullDDR shell -- has OSPI + I2C + DDR):
 *   west build -b chipyard_riscv64 -d build_camcap samples/riskybird/camera_capture_fpga -- \
 *     -DDTC_OVERLAY_FILE="<elf>/overlays/fpga-common.overlay;<elf>/overlays/arty200t.overlay" \
 *     -DCONFIG_UART_HTIF=n -DCONFIG_UART_SIFIVE=y -DCONFIG_UART_SIFIVE_PORT_0=y \
 *     -DCONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=50000
 * Load: scripts/load_elf_to_soc.sh --console build_camcap/zephyr/zephyr.elf
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* ---- OSPI capture peripheral (ucbbar,ospi-hm01b0) register map, base from the generated DTS. ---- */
#define OSPI_BASE       0x10080000UL
#define OSPI_CTRL       0x00   /* [0]en [1]cont [2]irqEn ; w1 pulses [3]trig [4]clear [5]flush [6]arm */
#define OSPI_GEOM       0x04   /* [15:0]expWidth [31:16]expHeight */
#define OSPI_MCLKDIV    0x08   /* MCLK = sysclk/(2*(div+1)) */
#define OSPI_FIFOCOUNT  0x0c   /* buffered beats (RO) */
#define OSPI_FRAMECNT   0x10   /* completed frames (RO) */
#define OSPI_LASTWIDTH  0x14
#define OSPI_LASTHEIGHT 0x18
#define OSPI_FLAGS      0x1c   /* [0]dataValid [1]overflow [2]geomErr [3]sensorInt [4]busy [5]irqPend [6]bufFull */
#define OSPI_DATA       0x20   /* [31]valid [10]eof [9]eol [8]sof [7:0]data */
#define OSPI_CAPACITY   0x24   /* frame-buffer capacity in beats (RO) */
#define OSPI_PIXTARGET  0x28
#define OSPI_CAPCOUNT   0x2c
#define OSPI_PCLKCNT    0x30   /* PCLK rising edges (RO) -- 0 => no camera clock */
#define OSPI_FVLDCNT    0x34   /* FVLD rising edges (RO) -- 0 => no frame sync */
#define OSPI_LVLDCNT    0x38   /* LVLD rising edges (RO) -- 0 => no line sync */
#define OSPI_LASTPIX    0x3c
#define OSPI_CAPSTAT    0x40   /* [0]armed [1]captureDone */

#define CTRL_EN     (1u << 0)
#define CTRL_CONT   (1u << 1)
#define CTRL_TRIG   (1u << 3)
#define CTRL_CLEAR  (1u << 4)
#define CTRL_FLUSH  (1u << 5)
#define CTRL_ARM    (1u << 6)

#define DATA_VALID  (1u << 31)
#define DATA_EOF    (1u << 10)
#define DATA_EOL    (1u << 9)
#define DATA_SOF    (1u << 8)

/* sysclk 50 MHz; div=1 -> MCLK = 12.5 MHz (HM01B0 INCK ~6-27 MHz). */
#define MCLK_DIV    1

/* ---- HM01B0 SCCB ---- */
#define HM_ADDR             0x24
#define HM_MODEL_ID_H       0x0000
#define HM_MODEL_ID_L       0x0001
#define HM_MODE_SELECT      0x0100   /* 0=standby 1=streaming(continuous) 3=streaming N frames */
#define HM_TEST_PATTERN     0x0601   /* 0=off 1=walking/color-bar test pattern */
#define HM_MODEL_ID         0x01B0
#define HM_MODE_STREAMING   0x01
#define TEST_PATTERN        0x01     /* set 0 for a live image instead of the test pattern */

/* HM01B0 default geometry (QVGA-ish); also the OSPI core's default GEOM. */
#define FRAME_W  324
#define FRAME_H  244

/* Bounded-capture size: enough to sample many lines without risking a buffer overflow. */
#define CAP_PIXELS  4096

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
	printk("\n=== camera_capture_fpga: HM01B0 pixel capture over FPGA OSPI ===\n");

	/* [1] Capture peripheral present? */
	uint32_t cap = r32(OSPI_CAPACITY);
	printk("[1] OSPI capture @ 0x%08lx, frame-buffer capacity = %u beats\n", OSPI_BASE, cap);
	if (cap == 0) {
		printk("FAIL: capture peripheral did not answer -- shell lacks WithOspiCapture\n");
		return 0;
	}

	/* [2] Enable the capture block so MCLK is generated to the sensor.
	 * NOTE: the HM01B0 drives its output PCLK only while STREAMING, so we can't check PCLK yet --
	 * we first prove MCLK reaches the sensor via a successful I2C MODEL_ID read (needs the sensor
	 * clocked), then start streaming, then verify PCLK/FVLD/LVLD. */
	w32(OSPI_MCLKDIV, MCLK_DIV);
	w32(OSPI_CTRL, CTRL_EN);                       /* enable capture block (MCLK gen + frontend) */
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);
	w32(OSPI_CTRL, CTRL_EN);
	printk("[2] MCLKDIV=%u -> MCLK ~%u kHz (capture block enabled)\n",
	       MCLK_DIV, (unsigned)(50000000UL / (2UL * (MCLK_DIV + 1)) / 1000));

	/* [3] I2C + MODEL_ID -- also proves MCLK actually reaches the sensor. */
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(bus)) { printk("FAIL: i2c0 not ready\n"); return 0; }
	uint8_t hi = 0, lo = 0;
	if (hm_rd(bus, HM_MODEL_ID_H, &hi) || hm_rd(bus, HM_MODEL_ID_L, &lo)) {
		printk("FAIL: MODEL_ID read error (sensor NACK / unclocked -- MCLK not reaching sensor?)\n"); return 0;
	}
	uint16_t id = ((uint16_t)hi << 8) | lo;
	printk("[3] MODEL_ID = 0x%04x (expect 0x%04x)%s -- MCLK confirmed reaching sensor\n", id, HM_MODEL_ID,
	       id == HM_MODEL_ID ? "" : "  <-- unexpected");
	if (id != HM_MODEL_ID) { printk("FAIL: not an HM01B0\n"); return 0; }

	/* [4] Configure the sensor over I2C: 8-bit gated DVP is the power-on default; add the test
	 * pattern so pixel values are deterministic and non-constant. */
	if (hm_wr(bus, HM_TEST_PATTERN, TEST_PATTERN)) { printk("FAIL: TEST_PATTERN write NACK\n"); return 0; }
	printk("[4] TEST_PATTERN(0x0601)=0x%02x  (8-bit gated DVP = HM01B0 default)\n", TEST_PATTERN);

	/* [5] Capture-on-I2C-command: MODE_SELECT=streaming starts the sensor clocking pixels out.
	 * This is when the sensor begins driving PCLK/FVLD/LVLD/D. */
	if (hm_wr(bus, HM_MODE_SELECT, HM_MODE_STREAMING)) { printk("FAIL: MODE_SELECT write NACK\n"); return 0; }
	printk("[5] MODE_SELECT(0x0100)=streaming -- sensor now streaming\n");

	/* [6] Now that the sensor is streaming, verify the pixel clock + frame/line sync reach the FPGA.
	 * PCLK==0 here (but MODEL_ID above passed) would point to the PCLK input pin (W11), not MCLK. */
	uint32_t p0 = r32(OSPI_PCLKCNT), f0 = r32(OSPI_FVLDCNT), l0 = r32(OSPI_LVLDCNT);
	k_msleep(100);
	uint32_t p1 = r32(OSPI_PCLKCNT), f1 = r32(OSPI_FVLDCNT), l1 = r32(OSPI_LVLDCNT);
	printk("[6] PCLK edges %u->%u, FVLD %u->%u, LVLD %u->%u over 100 ms\n", p0, p1, f0, f1, l0, l1);
	if (p1 == p0) { printk("FAIL: no PCLK activity while streaming -- check the PCLK input pin (W11)\n"); return 0; }
	if (f1 == f0) { printk("FAIL: no frame-valid activity (PCLK present but no frame starts -- FVLD pin?)\n"); return 0; }
	if (l1 == l0) { printk("FAIL: no line-valid activity (frames start but no line -- LVLD pin?)\n"); return 0; }

	/* [7] Bounded capture: grab a fixed number of pixels (no overflow -- the frame buffer holds
	 * far more than this), so we get a clean sample of the data lines. */
	w32(OSPI_GEOM, (uint32_t)FRAME_W | ((uint32_t)FRAME_H << 16));
	w32(OSPI_PIXTARGET, CAP_PIXELS);
	w32(OSPI_CTRL, CTRL_EN | CTRL_CLEAR | CTRL_FLUSH);   /* clear status + empty buffer */
	w32(OSPI_CTRL, CTRL_EN | CTRL_ARM);                  /* enable + arm (w1 arm pulse) */
	w32(OSPI_CTRL, CTRL_EN);
	int timeout_ms = 500;
	while (!(r32(OSPI_CAPSTAT) & 0x2) && timeout_ms-- > 0) {   /* wait for captureDone */
		k_msleep(1);
	}
	uint32_t flags = r32(OSPI_FLAGS);
	printk("[7] bounded capture: target=%u captured=%u fifo=%u flags=0x%02x%s%s\n",
	       CAP_PIXELS, r32(OSPI_CAPCOUNT), r32(OSPI_FIFOCOUNT), flags,
	       (flags & 0x02) ? " OVERFLOW" : "", (flags & 0x04) ? " GEOMERR" : "");

	/* [8] Drain + analyze the data lines. The key diagnostics for the parallel bus:
	 *   orAll  = OR of every pixel  -> a 0 bit here is a data line that NEVER went high (stuck low
	 *                                  / unwired / wrong pin). A healthy 8-bit bus reaches 0xff.
	 *   andAll = AND of every pixel -> a 1 bit here is a data line stuck HIGH. Healthy = 0x00. */
	uint32_t hist[256] = {0};
	uint32_t npix = 0, nsof = 0, neol = 0, neof = 0;
	uint8_t first[32]; uint32_t nfirst = 0;
	uint8_t pmin = 0xff, pmax = 0x00, orAll = 0x00, andAll = 0xff;
	uint32_t guard = CAP_PIXELS + 64;
	for (;;) {
		uint32_t d = r32(OSPI_DATA);
		if (!(d & DATA_VALID)) break;              /* buffer empty */
		if (d & DATA_SOF) nsof++;
		if (d & DATA_EOL) neol++;
		if (d & DATA_EOF) { neof++; continue; }    /* EOF marker carries no pixel */
		uint8_t px = (uint8_t)(d & 0xff);
		hist[px]++; npix++;
		orAll |= px; andAll &= px;
		if (px < pmin) pmin = px;
		if (px > pmax) pmax = px;
		if (nfirst < sizeof(first)) first[nfirst++] = px;
		if (guard-- == 0) break;
	}
	uint32_t distinct = 0;
	for (int i = 0; i < 256; i++) { if (hist[i]) distinct++; }

	printk("[8] drained: pixels=%u sof=%u eol=%u eof=%u\n", npix, nsof, neol, neof);
	printk("    range 0x%02x..0x%02x, distinct=%u, lastPix=0x%02x\n",
	       pmin, pmax, distinct, r32(OSPI_LASTPIX) & 0xff);
	printk("    data-line activity: OR=0x%02x AND=0x%02x  (OR<0xff => some D[] never toggles)\n",
	       orAll, andAll);
	printk("    per-bit toggled D7..D0:");
	for (int b = 7; b >= 0; b--) { printk(" D%d=%c", b, (orAll & (1u << b)) ? '1' : '0'); }
	printk("\n    first %u pixels:", nfirst);
	for (uint32_t i = 0; i < nfirst; i++) { printk(" %02x", first[i]); }
	printk("\n");

	/* [9] Verdict. */
	if (npix == 0) {
		printk("[9] FAIL: sync present but zero pixels sampled\n");
	} else if (distinct <= 1) {
		printk("[9] FAIL: every pixel identical (0x%02x) -- data bus stuck/floating\n", pmin);
	} else if (orAll != 0xff) {
		printk("[9] PARTIAL: %u px, %u distinct, but OR=0x%02x -- data line(s) not toggling; check DVP pin map\n",
		       npix, distinct, orAll);
	} else {
		printk("[9] PASS: %u px, %u distinct, all 8 data lines toggle -- parallel bus + DVP framing OK\n",
		       npix, distinct);
	}

	for (;;) { k_msleep(1000); }
	return 0;
}
