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
#include <zephyr/drivers/sensor/vl53l1x.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include "ads7128.h"
#include "pmw3901.h"

#define I2C_NODE DT_NODELABEL(i2c0)
#define SPI_NODE DT_NODELABEL(spi2)
#define GPIO_NODE DT_NODELABEL(gpio0)
#define ADS7128_I2C_ADDR 0x17
#define VL53L1X_OLD_ADDR 0x29
#define VL53L1X_NEW_ADDR 0x30
#define VL53L1X_I2C_SLAVE_ADDR_REG 0x01  /* Register to change I2C address */

/* PMW3901 pins (riskybird PCB) */
#define PMW3901_CS_GPIO_PIN    19
#define PMW3901_RESET_GPIO_PIN 2

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

/* Reprogram VL53L1X I2C address from old_addr to new_addr
 * The sensor must be powered (XSHUT high) to accept I2C commands.
 * We mirror the ST API (VL53L1_SetDeviceAddress): write the new 7-bit
 * address to register VL53L1_I2C_SLAVE__DEVICE_ADDRESS (0x0001).
 * No XSHUT cycling is required for the change to take effect.
 */
static int vl53l1x_change_address(const struct device *i2c_dev, 
                                   uint8_t old_addr, uint8_t new_addr)
{
    uint8_t write_buf[3];
    uint8_t reg_addr[2];
    uint8_t read_back;
    int ret;
    uint8_t gpo_value;
    int i;

    printf("Reprogramming VL53L1X from 0x%02X to 0x%02X...\n", old_addr, new_addr);

    /* Ensure XSHUT is HIGH (sensor must be powered to accept I2C commands) */
    ret = ads7128_read_reg(i2c_dev, GPO_VALUE_ADDRESS, &gpo_value);
    if (ret != 0) {
        printf("ERROR: Failed to read GPO_VALUE (ret: %d)\n", ret);
        return ret;
    }
    if (!(gpo_value & (1 << 6))) {
        printf("ERROR: XSHUT (GPIO6) must be HIGH to reprogram address\n");
        return -EINVAL;
    }

    /* Wait for sensor to be ready after power-on (boot time + I2C ready) */
    k_msleep(50);

    /* Verify sensor responds at old address before attempting to reprogram */
    printf("Verifying sensor at old address 0x%02X...\n", old_addr);
    for (i = 0; i < 5; i++) {
        ret = i2c_write(i2c_dev, NULL, 0, old_addr);
        if (ret == 0) {
            printf("Sensor confirmed at 0x%02X\n", old_addr);
            break;
        }
        k_msleep(10);
    }
    if (ret != 0) {
        printf("WARNING: Sensor not responding at 0x%02X (ret: %d), continuing anyway...\n", old_addr, ret);
    }

    /* VL53L1X uses 16-bit register addresses, MSB first, then LSB, then data
     * Format matches VL53L1_WriteMulti: buffer[0]=MSB, buffer[1]=LSB, buffer[2]=data
     */
    write_buf[0] = (VL53L1X_I2C_SLAVE_ADDR_REG >> 8) & 0xFF;  /* Register MSB (0x00) */
    write_buf[1] = VL53L1X_I2C_SLAVE_ADDR_REG & 0xFF;          /* Register LSB (0x8A) */
    write_buf[2] = new_addr;                                   /* New address value */

    /* Write to the sensor at its current address (old_addr) */
    printf("Writing new address 0x%02X to register 0x%04X...\n", new_addr, VL53L1X_I2C_SLAVE_ADDR_REG);
    ret = i2c_write(i2c_dev, write_buf, sizeof(write_buf), old_addr);
    if (ret != 0) {
        printf("ERROR: Failed to write new address (ret: %d)\n", ret);
        printf("  Transaction: [0x%02X 0x%02X 0x%02X] to addr 0x%02X\n", 
               write_buf[0], write_buf[1], write_buf[2], old_addr);
        return ret;
    }

    k_msleep(10); /* Small delay for address change to take effect */

    /* Read back 0x0001 at old address to confirm write */
    reg_addr[0] = write_buf[0];
    reg_addr[1] = write_buf[1];
    ret = i2c_write_read(i2c_dev, old_addr, reg_addr, sizeof(reg_addr),
                         &read_back, 1);
    if (ret != 0) {
        printf("WARNING: Failed to read back 0x%04X at 0x%02X (ret: %d)\n",
               VL53L1X_I2C_SLAVE_ADDR_REG, old_addr, ret);
    } else {
        printf("Read-back at old addr: 0x%02X (expected 0x%02X)\n",
               read_back, new_addr);
    }

    /* Try reading 0x0001 at new address without cycling XSHUT */
    ret = i2c_write_read(i2c_dev, new_addr, reg_addr, sizeof(reg_addr),
                         &read_back, 1);
    if (ret != 0) {
        printf("WARNING: Failed to read back 0x%04X at new addr 0x%02X (ret: %d)\n",
               VL53L1X_I2C_SLAVE_ADDR_REG, new_addr, ret);
    } else {
        printf("Read-back after XSHUT at new addr: 0x%02X\n", read_back);
    }

    printf("VL53L1X address reprogrammed sequence complete.\n");
    return 0;
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

/* PMW3901 device/config instantiated at runtime (bring-up oriented) */
static struct pmw3901_config pmw3901_cfg;
static struct pmw3901_data pmw3901_data;
static const struct device pmw3901_device = {
	.name = "PMW3901",
	.config = &pmw3901_cfg,
	.data = &pmw3901_data,
};

#define PMW3901_DEV (&pmw3901_device)

static int pmw3901_init_config(void)
{
	if (!DT_NODE_EXISTS(SPI_NODE) || !DT_NODE_EXISTS(GPIO_NODE)) {
		printf("PMW3901: missing SPI/GPIO devicetree nodes\n");
		return -ENODEV;
	}

	const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
	const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);
	if (spi_dev == NULL || gpio_dev == NULL) {
		return -ENODEV;
	}

	pmw3901_cfg.spi.bus = spi_dev;
	/* PMW3901 uses SPI mode 3 (CPOL=1, CPHA=1) */
	pmw3901_cfg.spi.config.operation =
		SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA |
		SPI_TRANSFER_MSB;
	pmw3901_cfg.spi.config.frequency = 2000000; /* 2 MHz bring-up */
	pmw3901_cfg.spi.config.slave = 0;

	pmw3901_cfg.cs_gpio.port = gpio_dev;
	pmw3901_cfg.cs_gpio.pin = PMW3901_CS_GPIO_PIN;
	pmw3901_cfg.cs_gpio.dt_flags = GPIO_ACTIVE_LOW;

	pmw3901_cfg.reset_gpio.port = gpio_dev;
	pmw3901_cfg.reset_gpio.pin = PMW3901_RESET_GPIO_PIN;
	pmw3901_cfg.reset_gpio.dt_flags = GPIO_ACTIVE_LOW;

	return 0;
}

int main(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
    int ret;

    printf("Sensor Bringup: I2C Scan with GPIO Control\n");
    printf("==========================================\n\n");

    /* Initialize PMW3901 optical flow sensor before doing I2C scans */
    printf("Initializing PMW3901 (optical flow)...\n");
    ret = pmw3901_init_config();
    if (ret != 0) {
        printf("WARNING: PMW3901 config init failed (ret: %d), continuing\n\n", ret);
    } else if (!device_is_ready(pmw3901_cfg.spi.bus) || !device_is_ready(pmw3901_cfg.cs_gpio.port)) {
        printf("WARNING: PMW3901 SPI/GPIO not ready, continuing\n\n");
    } else {
        ret = pmw3901_init(PMW3901_DEV);
        if (ret != 0) {
            printf("WARNING: PMW3901 init failed (ret: %d), continuing\n\n", ret);
        } else {
            printf("PMW3901 initialized.\n\n");
        }
    }

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

    /* Reprogram VL53L1X address from 0x29 to 0x30 */
    ret = vl53l1x_change_address(i2c_dev, VL53L1X_OLD_ADDR, VL53L1X_NEW_ADDR);
    if (ret != 0) {
        printf("ERROR: Failed to reprogram VL53L1X address (ret: %d)\n", ret);
        printf("Continuing anyway - sensor may already be at new address.\n");
    }
    k_msleep(100); /* Allow time for sensor to stabilize after address change */

    /* Verify sensor is at new address */
    printf("Verifying VL53L1X at address 0x%02X...\n", VL53L1X_NEW_ADDR);
    ret = i2c_write(i2c_dev, NULL, 0, VL53L1X_NEW_ADDR);
    if (ret != 0) {
        printf("WARNING: VL53L1X not found at address 0x%02X (ret: %d)\n",
               VL53L1X_NEW_ADDR, ret);
        printf("Checking old address 0x%02X...\n", VL53L1X_OLD_ADDR);
        ret = i2c_write(i2c_dev, NULL, 0, VL53L1X_OLD_ADDR);
        if (ret == 0) {
            printf("Sensor still at old address. Address reprogramming may have failed.\n");
        }
    } else {
        printf("VL53L1X confirmed at address 0x%02X\n\n", VL53L1X_NEW_ADDR);
    }

    /* Wait a bit for the sensor to be ready after address change.
     * Note: We do NOT power cycle here as that would reset the address back to default.
     */
    printf("Waiting for sensor to stabilize after address change...\n");
    k_msleep(50);

    /* Verify sensor is still responding at new address before initializing */
    printf("Verifying sensor is responding at address 0x%02X...\n", VL53L1X_NEW_ADDR);
    ret = i2c_write(i2c_dev, NULL, 0, VL53L1X_NEW_ADDR);
    if (ret != 0) {
        printf("WARNING: Sensor not responding at 0x%02X (ret: %d), continuing anyway...\n", 
               VL53L1X_NEW_ADDR, ret);
    } else {
        printf("Sensor confirmed responding at 0x%02X\n", VL53L1X_NEW_ADDR);
    }

    /* Initialize the sensor now that it's powered and address is reprogrammed */
    const struct device *tof_dev = DEVICE_DT_GET(DT_ALIAS(tof0));
    
    printf("\nInitializing VL53L1X sensor after address reprogramming...\n");
    ret = vl53l1x_reinit(tof_dev);
    if (ret != 0) {
        printf("ERROR: Failed to initialize VL53L1X sensor (ret: %d)\n", ret);
        /* Error code -134 (0xFFFFFF7A) is from ST HAL library.
         * This could indicate sensor not ready, I2C issue, or invalid parameters.
         * Common VL53L1 error codes: 0=SUCCESS, -1=INVALID_PARAMS, -4=TIME_OUT, -13=REF_SPAD_INIT
         */
        printf("Error code %d (0x%08X) from ST HAL library.\n", ret, (unsigned int)ret);
        printf("Make sure:\n");
        printf("  - Sensor is powered (GPIO6 HIGH)\n");
        printf("  - Sensor is at address 0x%02X\n", VL53L1X_NEW_ADDR);
        printf("  - Sensor has had enough time after address change (50ms+)\n");
        printf("  - I2C bus is functioning correctly\n");
        return 1;
    }
    printf("VL53L1X sensor initialized successfully.\n\n");

    /* Enable VL53L5 power/enable via GPIO1 and perform a third I2C scan */
    printf("Enabling VL53L5 via ADS7128 GPIO1 (set HIGH)...\n");
    ret = ads7128_write_gpio(i2c_dev, 1, 1);
    if (ret != 0) {
        printf("ERROR: Failed to set GPIO1 HIGH (ret: %d)\n", ret);
        return 1;
    }
    k_msleep(100); /* Allow time for VL53L5 to power up / enable */

    printf("\n--- I2C Scan #3 (GPIO1 = HIGH, GPIO2-4 = LOW, GPIO6 = HIGH) ---\n");
    i2c_scan(i2c_dev);

    /* Continuously read distance measurements from the VL53L1X */
    printf("\nEntering VL53L1X continuous distance read loop...\n");
    printf("Note: If the driver failed initialization during boot, it may not recover\n");
    printf("automatically. The power cycle should help, but if errors persist, the\n");
    printf("driver may need to be modified to support deferred initialization.\n\n");

    struct sensor_value distance;
    double dist_mm;
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 10;
    motionBurst_t flow_motion;
    bool pmw_ok = (ret == 0);

    while (1) {
        ret = sensor_sample_fetch(tof_dev);
        if (ret < 0) {
            consecutive_errors++;
            if (consecutive_errors <= MAX_CONSECUTIVE_ERRORS) {
                printf("ERROR: sensor_sample_fetch failed (%d) [attempt %d/%d]\n", 
                       ret, consecutive_errors, MAX_CONSECUTIVE_ERRORS);
            }
            if (consecutive_errors == MAX_CONSECUTIVE_ERRORS) {
                printf("\nERROR: Too many consecutive failures. The driver may need to be reinitialized.\n");
                printf("The sensor is at address 0x%02X and GPIO6 is HIGH, but the driver\n", VL53L1X_NEW_ADDR);
                printf("initialized during boot when the sensor wasn't ready.\n");
                printf("Try power-cycling the board or modifying the driver to support runtime initialization.\n");
                return 1;
            }
        } else {
            consecutive_errors = 0; /* Reset error counter on success */
            ret = sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &distance);
            if (ret < 0) {
                printf("ERROR: sensor_channel_get failed (%d)\n", ret);
            } else {
                /* Distance is reported in meters; print directly using float printf support */
                dist_mm = sensor_value_to_double(&distance);
                if (pmw_ok && (pmw3901_read_motion_burst(PMW3901_DEV, &flow_motion) == 0)) {
                    printf("VL53L1X: %.1f mm | PMW3901: dX=%d dY=%d SQUAL=%u Shutter=%u\n",
                           dist_mm, flow_motion.deltaX, flow_motion.deltaY,
                           flow_motion.squal, flow_motion.shutter);
                } else {
                    printf("VL53L1X: %.1f mm | PMW3901: (no data)\n", dist_mm);
                }
            }
        }

        k_msleep(500); /* Poll every 500 ms */
    }
}
