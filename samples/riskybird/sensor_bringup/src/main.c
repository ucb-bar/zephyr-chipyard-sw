/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Sensor Bringup: I2C scan with GPIO control
 * 
 * This sample configures ADS7128 GPIO6, then performs I2C scans
 * with GPIO6 in different states to help identify devices that
 * may be enabled/disabled by GPIO control.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include "ads7128.h"

#define I2C_NODE DT_NODELABEL(i2c0)
#define ADS7128_I2C_ADDR 0x17

/* ADS7128 I2C command opcodes (Table 9 in datasheet, section 8.5.1/8.5.2) */
#define ADS7128_CMD_REG_WRITE 0x08  /* 0000 1000b: Single register write */
#define ADS7128_CMD_REG_READ  0x10  /* 0001 0000b: Single register read  */

/* Helper function to read a register */
static int ads7128_read_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t *data)
{
    uint8_t tx[2] = { ADS7128_CMD_REG_READ, reg_addr };
    int ret = i2c_write(i2c_dev, tx, sizeof(tx), ADS7128_I2C_ADDR);
    if (ret != 0) {
        return ret;
    }
    return i2c_read(i2c_dev, data, 1, ADS7128_I2C_ADDR);
}

/* Helper function to write a register */
static int ads7128_write_reg(const struct device *i2c_dev, uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[3] = { ADS7128_CMD_REG_WRITE, reg_addr, data };
    return i2c_write(i2c_dev, buf, sizeof(buf), ADS7128_I2C_ADDR);
}

/* Configure a channel as GPIO output */
static int ads7128_config_gpio_output(const struct device *i2c_dev, uint8_t channel)
{
    int ret;
    uint8_t pin_cfg, gpio_cfg, gpo_drive_cfg;
    
    if (channel > 7) {
        return -EINVAL;
    }
    
    ret = ads7128_read_reg(i2c_dev, PIN_CFG_ADDRESS, &pin_cfg);
    if (ret != 0) return ret;
    pin_cfg |= (1U << channel);
    ret = ads7128_write_reg(i2c_dev, PIN_CFG_ADDRESS, pin_cfg);
    if (ret != 0) return ret;
    
    ret = ads7128_read_reg(i2c_dev, GPIO_CFG_ADDRESS, &gpio_cfg);
    if (ret != 0) return ret;
    gpio_cfg |= (1 << channel);
    ret = ads7128_write_reg(i2c_dev, GPIO_CFG_ADDRESS, gpio_cfg);
    if (ret != 0) return ret;
    
    ret = ads7128_read_reg(i2c_dev, GPO_DRIVE_CFG_ADDRESS, &gpo_drive_cfg);
    if (ret != 0) return ret;
    gpo_drive_cfg |= (1U << channel);
    ret = ads7128_write_reg(i2c_dev, GPO_DRIVE_CFG_ADDRESS, gpo_drive_cfg);
    
    return ret;
}

/* Write a digital value to a GPIO channel */
static int ads7128_write_gpio(const struct device *i2c_dev, uint8_t channel, uint8_t value)
{
    int ret;
    uint8_t gpo_value;
    
    if (channel > 7) {
        return -EINVAL;
    }
    
    ret = ads7128_read_reg(i2c_dev, GPO_VALUE_ADDRESS, &gpo_value);
    if (ret != 0) {
        return ret;
    }
    
    if (value) {
        gpo_value |= (1 << channel);
    } else {
        gpo_value &= ~(1 << channel);
    }
    
    return ads7128_write_reg(i2c_dev, GPO_VALUE_ADDRESS, gpo_value);
}

/* Perform I2C bus scan */
static void i2c_scan(const struct device *i2c_dev)
{
    printf("Scanning I2C bus...\n");
    
    int found_count = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        int ret = i2c_write(i2c_dev, NULL, 0, addr);
        if (ret == 0) {
            printf("  Found device at 0x%02X\n", addr);
            found_count++;
        }
        k_msleep(10);
    }
    
    if (found_count == 0) {
        printf("  No devices found.\n");
    } else {
        printf("  Total: %d device(s) found.\n", found_count);
    }
}

int main(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
    int ret;

    printf("Sensor Bringup: I2C Scan with GPIO Control\n");
    printf("==========================================\n\n");

    /* Check if I2C device is ready */
    if (!device_is_ready(i2c_dev)) {
        printf("ERROR: I2C device %s is not ready.\n", i2c_dev->name);
        return 1;
    }
    printf("I2C device ready: %s\n\n", i2c_dev->name);

    /* Configure ADS7128 GPIO1, GPIO2, GPIO3, GPIO4, and GPIO6 as outputs */
    printf("Configuring ADS7128 GPIO1, GPIO2, GPIO3, GPIO4, and GPIO6 as outputs...\n");
    for (int gpio = 1; gpio <= 4; gpio++) {
        ret = ads7128_config_gpio_output(i2c_dev, gpio);
        if (ret != 0) {
            printf("ERROR: Failed to configure GPIO%d (ret: %d)\n", gpio, ret);
            return 1;
        }
    }
    ret = ads7128_config_gpio_output(i2c_dev, 6);
    if (ret != 0) {
        printf("ERROR: Failed to configure GPIO6 (ret: %d)\n", ret);
        return 1;
    }
    printf("GPIO1-4 and GPIO6 configured successfully.\n\n");

    /* Set GPIO1, GPIO2, GPIO3, GPIO4, and GPIO6 to LOW and perform first scan */
    printf("Setting GPIO1, GPIO2, GPIO3, GPIO4, and GPIO6 to LOW...\n");
    for (int gpio = 1; gpio <= 4; gpio++) {
        ret = ads7128_write_gpio(i2c_dev, gpio, 0);
        if (ret != 0) {
            printf("ERROR: Failed to set GPIO%d LOW (ret: %d)\n", gpio, ret);
            return 1;
        }
    }
    ret = ads7128_write_gpio(i2c_dev, 6, 0);
    if (ret != 0) {
        printf("ERROR: Failed to set GPIO6 LOW (ret: %d)\n", ret);
        return 1;
    }
    k_msleep(100); /* Allow time for GPIO to settle */
    
    printf("\n--- I2C Scan #1 (GPIO1-4, GPIO6 = LOW) ---\n");
    i2c_scan(i2c_dev);
    
    /* Set GPIO6 to HIGH and perform second scan (GPIO1-4 remain LOW) */
    printf("\nSetting GPIO6 to HIGH (GPIO1-4 remain LOW)...\n");
    ret = ads7128_write_gpio(i2c_dev, 6, 1);
    if (ret != 0) {
        printf("ERROR: Failed to set GPIO6 HIGH (ret: %d)\n", ret);
        return 1;
    }
    k_msleep(100); /* Allow time for GPIO to settle */
    
    printf("\n--- I2C Scan #2 (GPIO1-4 = LOW, GPIO6 = HIGH) ---\n");
    i2c_scan(i2c_dev);

    /* Give the ToF sensor a moment after enabling via GPIO6 */
    k_msleep(100);

    /* Now that GPIO6 is HIGH, the ToF sensor should be available */
    const struct device *tof_dev = DEVICE_DT_GET(DT_ALIAS(tof0));
    
    /* Check if ToF sensor device is ready */
    if (!device_is_ready(tof_dev)) {
        printf("ERROR: ToF sensor device (VL53L1X) is not ready.\n");
        return 1;
    }
    printf("ToF sensor device ready: %s\n\n", tof_dev->name);

    /* Continuously read distance measurements from the VL53L1X */
    printf("\nEntering VL53L1X continuous distance read loop...\n");

    struct sensor_value distance;
    double dist_mm;

    while (1) {
        ret = sensor_sample_fetch(tof_dev);
        if (ret < 0) {
            printf("ERROR: sensor_sample_fetch failed (%d)\n", ret);
        } else {
            ret = sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &distance);
            if (ret < 0) {
                printf("ERROR: sensor_channel_get failed (%d)\n", ret);
            } else {
                /* Distance is reported in meters; print directly using float printf support */
                dist_mm = sensor_value_to_double(&distance);
                printf("VL53L1X distance: %.1f mm\n", dist_mm);
            }
        }

        k_msleep(500); /* Poll every 500 ms */
    }
}
