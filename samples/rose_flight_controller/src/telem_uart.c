/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * See telem_uart.h. Body compiled only when ROSE_UART_TELEM=1 (FPGA target); otherwise empty stubs
 * so the ESP/co-sim builds are unaffected.
 */
#include "telem_uart.h"

#if ROSE_UART_TELEM

#include <stdint.h>

/* SiFive uart0 register map, applied to the 2nd UART instance (the ESP link). */
#define U1_BASE   0x10021000UL
#define U1_TXDATA 0x00   /* [31]=TX FIFO full */
#define U1_TXCTRL 0x08   /* [0]=txen */
#define U1_DIV    0x18   /* baud divisor = f_pclk/f_baud - 1 */
#define PCLK_HZ   50000000UL
#define BAUD      115200UL

static inline void u1_w(uintptr_t o, uint32_t v) { *(volatile uint32_t *)(U1_BASE + o) = v; }
static inline uint32_t u1_r(uintptr_t o) { return *(volatile uint32_t *)(U1_BASE + o); }

void telem_uart_init(void)
{
	u1_w(U1_DIV, (PCLK_HZ / BAUD) - 1UL);   /* 433 */
	u1_w(U1_TXCTRL, 0x1);                    /* txen */
}

void telem_uart_line(const char *s)
{
	for (; *s; ++s) {
		while (u1_r(U1_TXDATA) & 0x80000000UL) { /* wait for FIFO room */ }
		u1_w(U1_TXDATA, (uint32_t)(uint8_t)*s);
	}
}

#else  /* ROSE_UART_TELEM == 0 */

void telem_uart_init(void) {}
void telem_uart_line(const char *s) { (void)s; }

#endif
