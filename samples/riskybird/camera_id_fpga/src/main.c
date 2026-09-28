/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * camera_id_fpga -- confirm the HM01B0 camera is alive on I2C once the FPGA drives its clock.
 *
 * The HM01B0 clocks its entire digital core (SCCB/I2C interface included) from the external
 * master clock. Nothing on the ESP drives that clock; only the FPGA OSPI capture peripheral does.
 * So this test must run on an OSPI-periphery shell (RocketArty200TOspiSpadConfig / DroneFullDDR):
 *   1. Program the OSPI MCLKDIV so MCLK runs at a sensor-legal rate.
 *   2. Read the model-ID register over I2C (16-bit reg address) and check it == 0x01B0.
 * An address ACK alone (what i2c_scanner sees) does NOT prove the sensor is clocked; a correct
 * model-ID readback does.
 *
 * Build (scratchpad OSPI shell):
 *   west build -b chipyard_riscv64 -d build_camid samples/riskybird/camera_id_fpga -- \
 *     -DDTC_OVERLAY_FILE="<elf>/overlays/fpga-common.overlay;<elf>/overlays/arty200t-scratchpad.overlay" \
 *     -DCONFIG_UART_HTIF=n -DCONFIG_UART_SIFIVE=y -DCONFIG_UART_SIFIVE_PORT_0=y \
 *     -DCONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=50000
 * Load: scripts/load_elf_to_soc.sh --console build_camid/zephyr/zephyr.elf
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* OSPI (ucbbar,ospi-hm01b0) register map -- base from the generated DTS (ospi@10080000). */
#define OSPI_BASE     0x10080000UL
#define OSPI_CTRL     0x00   /* [0]=enable [1]=continuous [2]=irqEnable */
#define OSPI_MCLKDIV  0x08   /* MCLK = sysclk / (2*(mclkDiv+1)) */

/* sysclk 50 MHz; mclkDiv=1 -> MCLK = 50/(2*2) = 12.5 MHz (HM01B0 INCK range ~6-27 MHz). */
#define MCLK_DIV      1

/* HM01B0 SCCB (I2C) */
#define HM01B0_ADDR       0x24
#define HM01B0_MODEL_ID_H 0x0000
#define HM01B0_MODEL_ID_L 0x0001
#define HM01B0_MODEL_ID   0x01B0

static inline void mmio_w(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline uint32_t mmio_r(uintptr_t a) { return *(volatile uint32_t *)a; }

/* 16-bit register address, 8-bit data read (SCCB style). */
static int hm_rd8(const struct device *bus, uint16_t reg, uint8_t *out)
{
	uint8_t rb[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
	return i2c_write_read(bus, HM01B0_ADDR, rb, sizeof(rb), out, 1);
}

int main(void)
{
	printk("\n=== camera_id_fpga: HM01B0 bring-up over FPGA I2C ===\n");

	/* 1. Drive MCLK to the sensor. */
	mmio_w(OSPI_BASE + OSPI_MCLKDIV, MCLK_DIV);
	mmio_w(OSPI_BASE + OSPI_CTRL, 0x1);            /* enable capture block (MCLK gen) */
	uint32_t rdiv = mmio_r(OSPI_BASE + OSPI_MCLKDIV);
	printk("MCLKDIV=%u -> MCLK ~= %u kHz (readback %u)\n",
	       MCLK_DIV, (unsigned)(50000000UL / (2UL * (MCLK_DIV + 1)) / 1000), rdiv);
	k_msleep(50);   /* let the sensor's internal clock settle before SCCB */

	/* 2. I2C bus up? */
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(bus)) {
		printk("FAIL: i2c0 not ready\n");
		return 0;
	}

	/* 3. Read the model ID. */
	uint8_t hi = 0, lo = 0;
	int r1 = hm_rd8(bus, HM01B0_MODEL_ID_H, &hi);
	int r2 = hm_rd8(bus, HM01B0_MODEL_ID_L, &lo);
	if (r1 || r2) {
		printk("FAIL: model-ID read err (h=%d l=%d) -- sensor NACK/unclocked?\n", r1, r2);
		return 0;
	}
	uint16_t id = ((uint16_t)hi << 8) | lo;
	printk("HM01B0 MODEL_ID = 0x%04x (expect 0x%04x) -> %s\n",
	       id, HM01B0_MODEL_ID, (id == HM01B0_MODEL_ID) ? "PASS -- camera alive + clocked" : "MISMATCH");

	/* Keep the core busy so the console session stays open. */
	for (;;) {
		k_msleep(1000);
	}
	return 0;
}
