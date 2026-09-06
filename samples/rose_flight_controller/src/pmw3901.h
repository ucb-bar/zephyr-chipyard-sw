/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * PMW3901 Optical Flow Sensor Driver
 * Ported from ESP-IDF to Zephyr
 */

#ifndef PMW3901_H
#define PMW3901_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>

typedef struct motionBurst_s {
    union {
        uint8_t motion;
        struct {
            uint8_t frameFrom0    : 1;
            uint8_t runMode       : 2;
            uint8_t reserved1     : 1;
            uint8_t rawFrom0      : 1;
            uint8_t reserved2     : 2;
            uint8_t motionOccured : 1;
        };
    };

    uint8_t observation;
    int16_t deltaX;
    int16_t deltaY;

    uint8_t squal;

    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;
} __attribute__((packed)) motionBurst_t;

struct pmw3901_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec cs_gpio;
    struct gpio_dt_spec reset_gpio;  /* Optional: CamRST pin (active low) */
    struct gpio_dt_spec led_gpio;    /* Optional: LED_N pin */
};

struct pmw3901_data {
    int64_t last_read_time_us;
};

int pmw3901_init(const struct device *dev);
int pmw3901_read_motion_burst(const struct device *dev, motionBurst_t *motion);
int pmw3901_read_motion_count(const struct device *dev, int16_t *deltaX, int16_t *deltaY);
uint8_t pmw3901_register_read(const struct device *dev, uint8_t reg);
int pmw3901_register_write(const struct device *dev, uint8_t reg, uint8_t value);

#endif // PMW3901_H
