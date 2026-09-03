/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * fpga_uart_telem -- Rocket-side half of the FPGA->ESP->WiFi telemetry bridge test.
 *
 * The DroneFullDDR shell exposes a 2nd SiFive UART at 0x1002_1000 wired to the ESP:
 *   FPGA uart1 TXD = F14 -> ESP U0RXD (GPIO17)
 *   FPGA uart1 RXD = E13 <- ESP U0TXD (GPIO16)
 * The ESP runs samples/riskybird/fpga_telem_bridge, which forwards whatever it reads
 * on its UART0 out over the SoftAP as UDP broadcast to 192.168.4.255:14550 (framed on
 * newlines). So anything this app writes to uart1 should surface at the ground station.
 *
 * We talk to uart1 by raw MMIO (SiFive uart0 reg map) rather than instantiating a 2nd
 * Zephyr driver port -- polling TX needs no IRQ mapping and no DT node, which keeps this
 * bring-up test decoupled from the shell's PLIC source numbering. Status/echo of each line
 * also goes to the console (uart0 -> /dev/ttyUSB1) so the test is observable with or without
 * the ESP attached.
 *
 * Build (scratchpad or DDR shell both have uart1 on DroneFullDDR):
 *   west build -b chipyard_riscv64 -d build_uarttelem samples/riskybird/fpga_uart_telem -- \
 *     -DDTC_OVERLAY_FILE="<elf>/overlays/fpga-common.overlay;<elf>/overlays/arty200t.overlay" \
 *     -DCONFIG_UART_HTIF=n -DCONFIG_UART_SIFIVE=y -DCONFIG_UART_SIFIVE_PORT_0=y \
 *     -DCONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=50000
 * Load: scripts/load_elf_to_soc.sh --console build_uarttelem/zephyr/zephyr.elf
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdio.h>

/* SiFive UART0 register map (offsets from the peripheral base). */
#define UART1_BASE   0x10021000UL
#define UART_TXDATA  0x00   /* [31] = TX FIFO full (RO on read); [7:0] = byte to send */
#define UART_TXCTRL  0x08   /* [0] = txen, [1] = nstop, [18:16] = tx watermark        */
#define UART_DIV     0x18   /* baud divisor: div = f_pclk / f_baud - 1                 */

/* Peripheral clock: the DroneFull/OSPI shells run the uncore at 50 MHz (matches
 * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=50000 * RTC_CLOCK_DIVIDER_VALUE=1000). */
#define PCLK_HZ      50000000UL
#define BAUD         115200UL
#define UART_DIVISOR ((PCLK_HZ / BAUD) - 1UL)   /* 50e6/115200 - 1 = 433 */

static inline void mmio_w(uintptr_t addr, uint32_t val)
{
	*(volatile uint32_t *)addr = val;
}
static inline uint32_t mmio_r(uintptr_t addr)
{
	return *(volatile uint32_t *)addr;
}

static void uart1_init(void)
{
	mmio_w(UART1_BASE + UART_DIV, UART_DIVISOR);
	mmio_w(UART1_BASE + UART_TXCTRL, 0x1);   /* txen=1, 1 stop bit, watermark 0 */
}

static void uart1_putc(char c)
{
	/* TXDATA[31] is the "FIFO full" flag; spin until there is room. */
	while (mmio_r(UART1_BASE + UART_TXDATA) & 0x80000000UL) {
		/* busy-wait */
	}
	mmio_w(UART1_BASE + UART_TXDATA, (uint32_t)(uint8_t)c);
}

static void uart1_puts(const char *s)
{
	for (; *s; ++s) {
		uart1_putc(*s);
	}
}

int main(void)
{
	printk("fpga_uart_telem: uart1 @ 0x%08lx, div=%lu (%lu baud @ %lu Hz)\n",
	       UART1_BASE, UART_DIVISOR, BAUD, PCLK_HZ);
	uart1_init();
	printk("fpga_uart_telem: streaming telemetry on uart1 -> ESP -> WiFi (UDP :14550)\n");

	uint32_t seq = 0;
	for (;;) {
		uint32_t ms = k_uptime_get_32();
		char line[96];
		/* Newline-terminated so the ESP bridge frames + broadcasts one datagram per line. */
		int n = snprintf(line, sizeof(line),
				 "RBTELEM seq=%u t=%ums hb=alive fpga=rocket\n", seq, ms);
		if (n > 0) {
			uart1_puts(line);
		}
		/* Echo to the console so we can confirm TX even before the ESP is listening. */
		printk("tx[%u] %ums\n", seq, ms);
		seq++;
		k_msleep(100);   /* 10 Hz telemetry */
	}
	return 0;
}
