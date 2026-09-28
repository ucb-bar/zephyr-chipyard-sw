/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * See telem_uart.h. Body compiled only when ROSE_UART_TELEM=1 (FPGA target); otherwise empty stubs
 * so the ESP/co-sim builds are unaffected.
 *
 * The uart1 link to the ESP bridge is bidirectional:
 *   TX (F14 -> ESP U0RXD) : newline-framed telemetry + snapshot chunks (telem_uart_line)
 *   RX (E13 <- ESP U0TXD) : newline-framed commands relayed from the GCS :14551 uplink
 *                           (telem_uart_poll_cmd)
 */
#include "telem_uart.h"
#include <stddef.h>
#include <stdint.h>

#if ROSE_UART_TELEM

#include <zephyr/kernel.h>   /* k_mutex -- serialize uart1 TX across threads */


/* SiFive uart0 register map, applied to the 2nd UART instance (the ESP link). */
#define U1_BASE   0x10021000UL
#define U1_TXDATA 0x00   /* [31]=TX FIFO full,  [7:0]=data (write)  */
#define U1_RXDATA 0x04   /* [31]=RX FIFO empty, [7:0]=data (read)   */
#define U1_TXCTRL 0x08   /* [0]=txen */
#define U1_RXCTRL 0x0C   /* [0]=rxen */
#define U1_DIV    0x18   /* baud divisor = f_pclk/f_baud - 1 */
#define PCLK_HZ   50000000UL
/* 921600: fast enough that a downsampled camera snapshot streams in a fraction of a second and the
 * ~100 Hz telemetry never back-pressures the loop. DIV=53 -> 925926 baud (+0.47%, within tol). */
#define BAUD      921600UL

static inline void u1_w(uintptr_t o, uint32_t v) { *(volatile uint32_t *)(U1_BASE + o) = v; }
static inline uint32_t u1_r(uintptr_t o) { return *(volatile uint32_t *)(U1_BASE + o); }

/* UART TX DMA (RocketArty200TDroneFullDDRDmaUartConfig): a TL-master engine that streams a DDR buffer
 * into uart1 TXDATA with FIFO-full flow control, so the CPU kicks-and-waits instead of spinning on the
 * full bit for every byte. Register block mirrors the OSPI camera DMA. Enabled with -DROSE_UART_DMA=1
 * (needs the DmaUart bitstream); otherwise the per-byte CPU-poll path below is used. */
#ifndef ROSE_UART_DMA
#define ROSE_UART_DMA 0
#endif
#define UDMA_BASE     0x10022000UL
#define UDMA_ADDR_LO  0x44
#define UDMA_ADDR_HI  0x48
#define UDMA_LEN      0x4C
#define UDMA_CTRL     0x50   /* [0]=EN [1]=START(w1p) [3]=CLEAR(w1p) */
#define UDMA_STATUS   0x54   /* [0]=BUSY [1]=DONE [2]=ERROR */
#define UDMA_EN       (1u << 0)
#define UDMA_START    (1u << 1)
#define UDMA_CLEAR    (1u << 3)
#define UDMA_BUSY     (1u << 0)
#define UDMA_DONE     (1u << 1)
static inline void ud_w(uintptr_t o, uint32_t v) { *(volatile uint32_t *)(UDMA_BASE + o) = v; }
static inline uint32_t ud_r(uintptr_t o) { return *(volatile uint32_t *)(UDMA_BASE + o); }

/* Serializes uart1 TX so a full line writes atomically. Without it the control loop's RBT telemetry
 * and the camera thread's IMG snapshot chunks interleave byte-by-byte on the shared TX FIFO and both
 * come out corrupted. */
static struct k_mutex tx_mutex;

void telem_uart_init(void)
{
	k_mutex_init(&tx_mutex);
	u1_w(U1_DIV, (PCLK_HZ / BAUD) - 1UL);   /* 53 @ 921600 */
	u1_w(U1_TXCTRL, 0x1);                    /* txen */
	u1_w(U1_RXCTRL, 0x1);                    /* rxen -- command uplink from the ESP bridge */
}

#if ROSE_UART_DMA
void telem_uart_line(const char *s)
{
	size_t n = 0;
	while (s[n]) { n++; }
	if (n == 0) { return; }

	k_mutex_lock(&tx_mutex, K_FOREVER);
	/* Order the line's stores before the DMA reads it (the TL master is not L1-D-coherent by
	 * contract; on this Rocket the FBUS master path is coherent in practice -- same as the camera
	 * DMA -- but the fence keeps the kick correctly ordered regardless). */
	__asm__ volatile ("fence" ::: "memory");
	ud_w(UDMA_ADDR_LO, (uint32_t)((uintptr_t)s & 0xffffffffu));
	ud_w(UDMA_ADDR_HI, (uint32_t)(((uint64_t)(uintptr_t)s) >> 32));
	ud_w(UDMA_LEN, (uint32_t)n);
	ud_w(UDMA_CTRL, UDMA_EN | UDMA_START);
	/* Wait for the engine, YIELDING rather than busy-spinning on the per-byte full bit: the whole
	 * point of the DMA is to free the CPU during the ~baud-limited transfer. A bounded fallback
	 * avoids a wedge if the engine never completes. */
	for (uint32_t g = 0; g < 200000u; g++) {
		if (ud_r(UDMA_STATUS) & UDMA_DONE) { break; }
		if (!(ud_r(UDMA_STATUS) & UDMA_BUSY)) { break; }
		k_yield();
	}
	ud_w(UDMA_CTRL, UDMA_EN | UDMA_CLEAR);   /* clear DONE/ERROR, keep the engine enabled */
	k_mutex_unlock(&tx_mutex);
}
#else
void telem_uart_line(const char *s)
{
	k_mutex_lock(&tx_mutex, K_FOREVER);
	for (; *s; ++s) {
		/* Bounded wait for FIFO room: if the ESP relay back-pressures (its RX FIFO full, e.g. mid
		 * snapshot burst), DROP the rest of the line instead of spinning forever -- a stuck spin
		 * here would hold tx_mutex and stall the control loop. ~2M iters @ 50 MHz is tens of ms,
		 * far longer than one 921600-baud byte, so normal flow never trips it. */
		uint32_t spin = 0;
		while (u1_r(U1_TXDATA) & 0x80000000UL) {
			if (++spin > 2000000u) { goto done; }
		}
		u1_w(U1_TXDATA, (uint32_t)(uint8_t)*s);
	}
done:
	k_mutex_unlock(&tx_mutex);
}
#endif

/* Drain the RX FIFO into a line buffer; return a complete NUL-terminated, newline-stripped command
 * line when one arrives, else NULL. Non-blocking: reads only what's already in the FIFO. Overlong
 * lines (no newline within the buffer) are flushed to avoid a stuck partial. */
static volatile uint32_t g_rx_bytes;   /* total bytes seen on uart1 RX (RX-link liveness diag) */
uint32_t telem_uart_rx_count(void) { return g_rx_bytes; }

const char *telem_uart_poll_cmd(void)
{
	static char buf[96];
	static size_t len;
	uint32_t r;

	while (!((r = u1_r(U1_RXDATA)) & 0x80000000UL)) {   /* while RX FIFO not empty */
		char c = (char)(r & 0xFF);
		g_rx_bytes++;
		if (c == '\n' || c == '\r') {
			if (len == 0) {
				continue;               /* skip blank lines / CRLF pairs */
			}
			buf[len] = '\0';
			len = 0;
			return buf;
		}
		if (len < sizeof(buf) - 1) {
			buf[len++] = c;
		} else {
			len = 0;                    /* overlong -> drop and resync on next newline */
		}
	}
	return NULL;
}

#else  /* ROSE_UART_TELEM == 0 */

void telem_uart_init(void) {}
void telem_uart_line(const char *s) { (void)s; }
const char *telem_uart_poll_cmd(void) { return NULL; }
uint32_t telem_uart_rx_count(void) { return 0; }

#endif
