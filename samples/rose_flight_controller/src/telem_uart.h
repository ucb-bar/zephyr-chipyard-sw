/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART telemetry downlink for the FPGA flight controller. When the FC runs on the FPGA Rocket, the
 * ESP32 acts purely as a wireless bridge (samples/riskybird/fpga_telem_bridge): the FC writes
 * newline-framed telemetry lines to the 2nd SiFive UART (uart1 @ 0x1002_1000, TXD=F14 -> ESP U0RXD)
 * and the ESP relays them over its SoftAP as UDP broadcast. Raw MMIO (SiFive uart0 reg map), polled
 * TX -- no Zephyr driver instance / IRQ mapping needed. Enabled with -DROSE_UART_TELEM=1.
 */
#ifndef ROSE_TELEM_UART_H_
#define ROSE_TELEM_UART_H_

#ifndef ROSE_UART_TELEM
#define ROSE_UART_TELEM 0
#endif
#ifndef ROSE_UART_TELEM_DIV
#define ROSE_UART_TELEM_DIV 20   /* emit every Nth control tick (~2 kHz loop / 20 = ~100 Hz) */
#endif

#ifdef __cplusplus
extern "C" {
#endif

void telem_uart_init(void);          /* configure uart1 (115200 @ 50 MHz peripheral clock) */
void telem_uart_line(const char *s); /* blocking-polled write of a NUL-terminated line to uart1 */

#ifdef __cplusplus
}
#endif

#endif /* ROSE_TELEM_UART_H_ */
