/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * PMW3901 Optical Flow Sensor Test
 * 
 * This sample initializes and tests the PMW3901 optical flow sensor
 * connected via SPI.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include "pmw3901.h"
#include <zephyr/devicetree.h>

/* Get SPI device from device tree */
#if IS_ENABLED(CONFIG_SPI_BITBANG)
#define SPI_NODE DT_NODELABEL(pmw3901_spi)
#else
#define SPI_NODE DT_NODELABEL(spi2)
#endif

/* GPIO device - ESP32C6 uses gpio0 */
#define GPIO_NODE DT_NODELABEL(gpio0)

/* GPIO pin numbers - defined in overlay */
#define CS_GPIO_PIN    19
#define RESET_GPIO_PIN 2
#define LED_GPIO_PIN   3

/* Device configuration and data structures - initialized at runtime */
static struct pmw3901_config pmw3901_cfg;
static struct pmw3901_data pmw3901_data;

/* Create a simple device structure */
static const struct device pmw3901_device = {
    .name = "PMW3901",
    .config = &pmw3901_cfg,
    .data = &pmw3901_data,
};

#define PMW3901_DEV (&pmw3901_device)

/* Initialize PMW3901 configuration at runtime */
static int pmw3901_init_config(void)
{
    const struct device *spi_dev;
    const struct device *gpio_dev;

    /* Get devices - these should exist if device tree is correct */
    if (!DT_NODE_EXISTS(SPI_NODE)) {
        printf("ERROR: SPI2 node does not exist in device tree\n");
        return -ENODEV;
    }
    if (!DT_NODE_EXISTS(GPIO_NODE)) {
        printf("ERROR: GPIO0 node does not exist in device tree\n");
        return -ENODEV;
    }

    spi_dev = DEVICE_DT_GET(SPI_NODE);
    gpio_dev = DEVICE_DT_GET(GPIO_NODE);

    if (spi_dev == NULL || gpio_dev == NULL) {
        printf("ERROR: Failed to get device pointers\n");
        return -ENODEV;
    }

    pmw3901_cfg.spi.bus = spi_dev;
    /* PMW3901 uses SPI mode 3 (CPOL=1, CPHA=1) */
    pmw3901_cfg.spi.config.operation =
        SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
    pmw3901_cfg.spi.config.frequency = 2000000;  /* 2MHz */
    pmw3901_cfg.spi.config.slave = 0;

    pmw3901_cfg.cs_gpio.port = gpio_dev;
    pmw3901_cfg.cs_gpio.pin = CS_GPIO_PIN;
    pmw3901_cfg.cs_gpio.dt_flags = GPIO_ACTIVE_LOW;

    /* riskybird v3: the PMW3901 (IC1) NRESET is tied high through R37 (no GPIO reset), and its
     * MOTION line goes to the FPGA SoM (JB2.73), not the ESP32-C6 -- so there is no reset/LED GPIO
     * on this board. Leave both NULL; the driver guards on .port and does a SOFTWARE power-on-reset
     * (reg 0x3A=0x5A) + polled reads, so the chip-ID connection test still works over SPI alone.
     * SPI itself is a shared bus (ESP via 100R R63-66, FPGA via 47R R27-29/34) -- the FPGA must be
     * off/high-Z for the ESP to master it (true during bring-up). */
    pmw3901_cfg.reset_gpio.port = NULL;
    pmw3901_cfg.led_gpio.port = NULL;

    return 0;
}

int main(void)
{
    int ret;
    motionBurst_t motion;
    int16_t deltaX, deltaY;

    printf("PMW3901 Optical Flow Sensor Test\n");
    printf("================================\n\n");

    /* Initialize PMW3901 configuration */
    ret = pmw3901_init_config();
    if (ret != 0) {
        printf("ERROR: Failed to initialize PMW3901 configuration (ret: %d)\n", ret);
        return 1;
    }

    /* Check if SPI and GPIO devices are ready */
    if (!device_is_ready(pmw3901_cfg.spi.bus)) {
        printf("ERROR: SPI device is not ready\n");
        return 1;
    }
    if (!device_is_ready(pmw3901_cfg.cs_gpio.port)) {
        printf("ERROR: GPIO device is not ready\n");
        return 1;
    }

    printf("SPI and GPIO devices ready\n\n");

    /* Initialize the PMW3901 sensor. Retry in a loop: pmw3901_init() prints the chip ID on every
     * attempt (0x49/0xB6 = alive, 0xFF = open/not-wetted, 0x00 = MISO low), so the connection
     * status is ALWAYS visible on the serial monitor no matter when it attaches -- no need to catch
     * a one-shot boot print or fight the USB-JTAG reset. */
    printf("Initializing PMW3901 (looping chip-ID probe until connected)...\n");
    while (pmw3901_init(PMW3901_DEV) != 0) {
        k_msleep(700);
    }
    printf("PMW3901 initialized successfully.\n\n");

    /* --- throughput benchmark: time back-to-back motion-burst reads (SPI @ 2 MHz) --- */
    {
        const int N = 2000;
        uint32_t c0 = k_cycle_get_32();
        for (int i = 0; i < N; i++) {
            pmw3901_read_motion_burst(PMW3901_DEV, &motion);
        }
        uint32_t us = k_cyc_to_us_floor32(k_cycle_get_32() - c0);
        printf("BENCH: %d motion-burst reads in %u us -> %.2f us/read, %.0f Hz max (SPI 2MHz)\n\n",
               N, us, (double)us / N, N * 1e6 / (double)us);
    }

    /* Continuously read motion data */
    printf("Entering continuous motion read loop...\n");
    printf("Format: deltaX, deltaY, SQUAL, Shutter\n\n");

    while (1) {
        /* Read motion burst */
        ret = pmw3901_read_motion_burst(PMW3901_DEV, &motion);
        if (ret != 0) {
            printf("ERROR: Failed to read motion burst (ret: %d)\n", ret);
        } else {
            printf("Motion: deltaX=%5d, deltaY=%5d, SQUAL=%3u, Shutter=%5u",
                   motion.deltaX, motion.deltaY, motion.squal, motion.shutter);
            
            if (motion.motionOccured) {
                printf(" [MOTION]");
            }
            printf("\n");
        }

        /* Also read motion count for comparison */
        ret = pmw3901_read_motion_count(PMW3901_DEV, &deltaX, &deltaY);
        if (ret == 0) {
            printf("  Count: deltaX=%5d, deltaY=%5d\n", deltaX, deltaY);
        }

        k_msleep(100); /* Poll every 100ms */
    }

    return 0;
}
