/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * ADS7128 I2C ADC Expander Connection Test
 * 
 * This sample validates that the ADS7128 I2C expander is properly
 * connected and accessible on the I2C bus at address 0x17.
 * 
 * The ADS7128 is an 8-channel, 12-bit ADC with I²C interface.
 * Reference: ADS7128 datasheet (SBAS868A)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
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
    /* Per datasheet 8.5.1.1: Single Register Read
     * S | Addr(W) | A | 0001 0000b | A | RegAddr | A | Sr | Addr(R) | A | Data | NACK | P
     *
     * Implemented as two separate I2C operations:
     *  1) i2c_write(): [CMD_REG_READ, reg_addr]
     *  2) i2c_read():  1 byte
     */
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
    /* Per datasheet 8.5.2.1: Single Register Write
     * S | Addr(W) | A | 0000 1000b | A | RegAddr | A | Data | A | P
     */
    uint8_t buf[3] = { ADS7128_CMD_REG_WRITE, reg_addr, data };
    return i2c_write(i2c_dev, buf, sizeof(buf), ADS7128_I2C_ADDR);
}

/**
 * Configure a channel as GPIO output
 * @param i2c_dev I2C device handle
 * @param channel Channel number (0-7)
 * @return 0 on success, negative error code on failure
 */
static int ads7128_config_gpio_output(const struct device *i2c_dev, uint8_t channel)
{
    int ret;
    uint8_t pin_cfg, gpio_cfg, gpo_drive_cfg;
    
    if (channel > 7) {
        return -EINVAL;
    }
    
    /* Read current PIN_CFG register */
    ret = ads7128_read_reg(i2c_dev, PIN_CFG_ADDRESS, &pin_cfg);
    if (ret != 0) {
        return ret;
    }
    
    /* Set this channel as GPIO (1 = GPIO, 0 = analog input) */
    pin_cfg |= (1U << channel);
    ret = ads7128_write_reg(i2c_dev, PIN_CFG_ADDRESS, pin_cfg);
    if (ret != 0) {
        return ret;
    }
    
    /* Read current GPIO_CFG register */
    ret = ads7128_read_reg(i2c_dev, GPIO_CFG_ADDRESS, &gpio_cfg);
    if (ret != 0) {
        return ret;
    }
    
    /* Set channel as output (bit for channel) */
    gpio_cfg |= (1 << channel);
    ret = ads7128_write_reg(i2c_dev, GPIO_CFG_ADDRESS, gpio_cfg);
    if (ret != 0) {
        return ret;
    }
    
    /* Read current GPO_DRIVE_CFG register */
    ret = ads7128_read_reg(i2c_dev, GPO_DRIVE_CFG_ADDRESS, &gpo_drive_cfg);
    if (ret != 0) {
        return ret;
    }
    
    /* Configure as push-pull output (1 = push-pull, 0 = open-drain) */
    gpo_drive_cfg |= (1U << channel);
    ret = ads7128_write_reg(i2c_dev, GPO_DRIVE_CFG_ADDRESS, gpo_drive_cfg);
    
    return ret;
}

/**
 * Write a digital value to a GPIO channel
 * @param i2c_dev I2C device handle
 * @param channel Channel number (0-7)
 * @param value 0 for low, non-zero for high
 * @return 0 on success, negative error code on failure
 */
static int ads7128_write_gpio(const struct device *i2c_dev, uint8_t channel, uint8_t value)
{
    int ret;
    uint8_t gpo_value;
    
    if (channel > 7) {
        return -EINVAL;
    }
    
    /* Read current GPO_VALUE register */
    ret = ads7128_read_reg(i2c_dev, GPO_VALUE_ADDRESS, &gpo_value);
    if (ret != 0) {
        return ret;
    }
    
    /* Set or clear the bit for this channel */
    if (value) {
        gpo_value |= (1 << channel);
    } else {
        gpo_value &= ~(1 << channel);
    }
    
    /* Write back the updated value */
    ret = ads7128_write_reg(i2c_dev, GPO_VALUE_ADDRESS, gpo_value);
    
    return ret;
}

/**
 * Read the current GPIO output value
 * @param i2c_dev I2C device handle
 * @param channel Channel number (0-7)
 * @param value Pointer to store the value (0 or 1)
 * @return 0 on success, negative error code on failure
 */
static int ads7128_read_gpio(const struct device *i2c_dev, uint8_t channel, uint8_t *value)
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
    
    *value = (gpo_value >> channel) & 0x01;
    return 0;
}

int main(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
    uint8_t reg_data;
    int ret;

    printf("ADS7128 I2C Expander Connection Test\n");
    printf("====================================\n\n");

    /* Check if I2C device is ready */
    if (!device_is_ready(i2c_dev)) {
        printf("ERROR: I2C device %s is not ready.\n", i2c_dev->name);
        return 1;
    }
    printf("I2C device ready: %s\n", i2c_dev->name);

    /* Read and print SYSTEM_STATUS register (0x00) */
    ret = ads7128_read_reg(i2c_dev, SYSTEM_STATUS_ADDRESS, &reg_data);
    if (ret != 0) {
        printf("ERROR: Failed to read SYSTEM_STATUS register (ret: %d)\n", ret);
    } else {
        printf("SYSTEM_STATUS = 0x%02X\n", reg_data);
    }

    /* Debug: read PIN/GPIO configuration *before* any changes */
    uint8_t pin_cfg_before = 0, gpio_cfg_before = 0;
    uint8_t drive_cfg_before = 0, gpo_val_before = 0;
    (void)ads7128_read_reg(i2c_dev, PIN_CFG_ADDRESS, &pin_cfg_before);
    (void)ads7128_read_reg(i2c_dev, GPIO_CFG_ADDRESS, &gpio_cfg_before);
    (void)ads7128_read_reg(i2c_dev, GPO_DRIVE_CFG_ADDRESS, &drive_cfg_before);
    (void)ads7128_read_reg(i2c_dev, GPO_VALUE_ADDRESS, &gpo_val_before);
    printf("DEBUG BEFORE CFG: PIN_CFG=0x%02X, GPIO_CFG=0x%02X, GPO_DRIVE_CFG=0x%02X, GPO_VALUE=0x%02X\n",
           pin_cfg_before, gpio_cfg_before, drive_cfg_before, gpo_val_before);

    /* Test 1: Try to write zero bytes (basic I2C presence check) */
    printf("\nTest 1: Basic I2C presence check...\n");
    ret = i2c_write(i2c_dev, NULL, 0, ADS7128_I2C_ADDR);
    if (ret != 0) {
        printf("ERROR: ADS7128 not responding at address 0x%02X (ret: %d)\n", 
               ADS7128_I2C_ADDR, ret);
        printf("       Check I2C connections and address configuration.\n");
        return 1;
    }
    printf("       PASS: Device ACKed at address 0x%02X\n", ADS7128_I2C_ADDR);

    /* Test 2: Read GENERAL_CFG register */
    printf("\nTest 2: Reading GENERAL_CFG register (0x01)...\n");
    ret = ads7128_read_reg(i2c_dev, GENERAL_CFG_ADDRESS, &reg_data);
    if (ret != 0) {
        printf("ERROR: Failed to read GENERAL_CFG register (ret: %d)\n", ret);
        return 1;
    }
    printf("       PASS: Read GENERAL_CFG = 0x%02X\n", reg_data);

    /* Test 3: Read DATA_CFG register */
    printf("\nTest 3: Reading DATA_CFG register (0x02)...\n");
    ret = ads7128_read_reg(i2c_dev, DATA_CFG_ADDRESS, &reg_data);
    if (ret != 0) {
        printf("ERROR: Failed to read DATA_CFG register (ret: %d)\n", ret);
        return 1;
    }
    printf("       PASS: Read DATA_CFG = 0x%02X\n", reg_data);

    /* Test 4: Read OSR_CFG register */
    printf("\nTest 4: Reading OSR_CFG register (0x03)...\n");
    ret = ads7128_read_reg(i2c_dev, OSR_CFG_ADDRESS, &reg_data);
    if (ret != 0) {
        printf("ERROR: Failed to read OSR_CFG register (ret: %d)\n", ret);
        return 1;
    }
    printf("       PASS: Read OSR_CFG = 0x%02X\n", reg_data);

    /* Test 5: Read OPMODE_CFG register */
    printf("\nTest 5: Reading OPMODE_CFG register (0x04)...\n");
    ret = ads7128_read_reg(i2c_dev, OPMODE_CFG_ADDRESS, &reg_data);
    if (ret != 0) {
        printf("ERROR: Failed to read OPMODE_CFG register (ret: %d)\n", ret);
        return 1;
    }
    printf("       PASS: Read OPMODE_CFG = 0x%02X\n", reg_data);

    /* Test 6: Read PIN_CFG register */
    printf("\nTest 6: Reading PIN_CFG register (0x05)...\n");
    ret = ads7128_read_reg(i2c_dev, PIN_CFG_ADDRESS, &reg_data);
    if (ret != 0) {
        printf("ERROR: Failed to read PIN_CFG register (ret: %d)\n", ret);
        return 1;
    }
    printf("       PASS: Read PIN_CFG = 0x%02X\n", reg_data);

    /* All tests passed */
    printf("\n====================================\n");
    printf("ADS7128 connected successfully!\n");
    printf("All register read tests passed.\n");
    printf("Device is ready for use.\n");
    printf("====================================\n");

    /* GPIO Toggle Example */
    printf("\n====================================\n");
    printf("GPIO Toggle Example\n");
    printf("====================================\n");
    printf("Configuring GPIO1 as output...\n");
    
    ret = ads7128_config_gpio_output(i2c_dev, 1);
    if (ret != 0) {
        printf("ERROR: Failed to configure GPIO1 as output (ret: %d)\n", ret);
        return 1;
    }
    printf("GPIO1 configured as output successfully.\n");

    /* Debug: dump key GPIO-related configuration registers AFTER config */
    uint8_t dbg_pin_cfg = 0, dbg_gpio_cfg = 0, dbg_drive_cfg = 0, dbg_gpo_val = 0;
    (void)ads7128_read_reg(i2c_dev, PIN_CFG_ADDRESS, &dbg_pin_cfg);
    (void)ads7128_read_reg(i2c_dev, GPIO_CFG_ADDRESS, &dbg_gpio_cfg);
    (void)ads7128_read_reg(i2c_dev, GPO_DRIVE_CFG_ADDRESS, &dbg_drive_cfg);
    (void)ads7128_read_reg(i2c_dev, GPO_VALUE_ADDRESS, &dbg_gpo_val);
    printf("DEBUG AFTER CFG:  PIN_CFG=0x%02X, GPIO_CFG=0x%02X, GPO_DRIVE_CFG=0x%02X, GPO_VALUE=0x%02X\n",
           dbg_pin_cfg, dbg_gpio_cfg, dbg_drive_cfg, dbg_gpo_val);
    
    printf("\nStarting GPIO1 toggle loop (every 1 second)...\n");
    printf("Press Ctrl+C to stop.\n\n");
    
    uint8_t gpio_state = 0;
    uint8_t read_back;
    
    while (1) {
        /* Toggle GPIO1 */
        gpio_state = !gpio_state;
        ret = ads7128_write_gpio(i2c_dev, 1, gpio_state);
        if (ret != 0) {
            printf("ERROR: Failed to write GPIO1 (ret: %d)\n", ret);
            k_msleep(5000);
            continue;
        }
        
        /* Read back to verify */
        ret = ads7128_read_gpio(i2c_dev, 1, &read_back);
        if (ret == 0) {
            printf("GPIO1: %s (read back: %s)\n", 
                   gpio_state ? "HIGH" : "LOW",
                   read_back ? "HIGH" : "LOW");
        } else {
            printf("GPIO1: %s (read back failed)\n", 
                   gpio_state ? "HIGH" : "LOW");
        }
        
        k_msleep(5000);
    }

    return 0;
}
