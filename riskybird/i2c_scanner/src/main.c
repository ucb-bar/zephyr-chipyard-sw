/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/reboot.h>
#include <stdio.h>

#define I2C_NODE DT_NODELABEL(i2c0)

int main(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c_dev)) {
        printf("I2C: Device not ready.\n");
        return 1;
    }

    printf("Starting I2C scan on %s...\n", i2c_dev->name);

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        /*
         * Try to write zero bytes. If the device ACKs, it's present.
         */
        int ret = i2c_write(i2c_dev, NULL, 0, addr);
        if (ret == 0) {
            printf("Found device at 0x%02X\n", addr);
        }
        k_msleep(10); // short delay between probes
    }

    printf("I2C scan complete.\n");

    return 0;
}
