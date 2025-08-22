/*
 * SPDX-License-Identifier: Apache-2.0
 * Minimal test app: echo back ID + four test floats for each drone
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
// #include <sys/printk.h>
#include <string.h>

#define UART_NODE      DT_NODELABEL(uart0)
#define HDR0           0xDE
#define HDR1           0xAD
#define HDR2           0xBE
#define HDR3           0xEF

static const uint8_t HEADER[4] = { HDR0, HDR1, HDR2, HDR3 };

int main(void)
{
    const struct device *uart = DEVICE_DT_GET(UART_NODE);
    if (!device_is_ready(uart)) {
        // printk("UART not ready\n");
        return 1;
    }

    uint8_t rx;
    size_t match_idx = 0;

    while (1) {
        /* 1) SYNC on header */
        while (match_idx < sizeof(HEADER)) {
            if (uart_poll_in(uart, &rx) == 0) {
                if (rx == HEADER[match_idx]) {
                    match_idx++;
                } else {
                    /* partial-match fallback: if this byte is HDR0, stay at 1 */
                    match_idx = (rx == HEADER[0]) ? 1 : 0;
                }
            }
            /* else timeout: retry */
        }

        /* 2) Read num_drones */
        uint8_t num_drones = 0;
        while (uart_poll_in(uart, &rx) != 0) { /* wait */ }
        num_drones = rx;

        /* 3) Skip over incoming state data: num_drones × 12 floats */
        size_t bytes_to_skip = num_drones * 12 * sizeof(float);
        for (size_t i = 0; i < bytes_to_skip; i++) {
            while (uart_poll_in(uart, &rx) != 0) { /* wait */ }
        }

        /* 4) For each drone, send back [HEADER][id][0,1,2,3] */
        for (uint8_t id = 0; id < num_drones; id++) {
            /* send header */
            for (int i = 0; i < 4; i++) {
                uart_poll_out(uart, HEADER[i]);
            }
            /* send drone id */
            uart_poll_out(uart, id);

            /* send four test floats: 0.0f, 1.0f, 2.0f, 3.0f little-endian */
            float actions[5] = { 0.0f, 1.0f, 2.0f, 3.0f, 0.0f };
            for (int j = 0; j < 5; j++) {
                uint8_t *pb = (uint8_t *)&actions[j];
                for (int b = 0; b < sizeof(float); b++) {
                    uart_poll_out(uart, pb[b]);
                }
            }
        }

        /* reset sync state */
        match_idx = 0;
    }
    return 0;
}
