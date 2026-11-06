/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * PMW3901 Optical Flow Sensor - bring-up implementation
 */

#include "pmw3901.h"

#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

/*
 * gpio_dt_spec values are LOGICAL, not raw electrical levels:
 * - value=1 means "active" (asserted)
 * - value=0 means "inactive" (deasserted)
 *
 * Since our CS is configured GPIO_ACTIVE_LOW, asserting CS drives the pin low.
 */
static void cs_deassert(const struct device *dev)
{
	const struct pmw3901_config *config = dev->config;
	(void)gpio_pin_set_dt(&config->cs_gpio, 0);
}

static void cs_assert(const struct device *dev)
{
	const struct pmw3901_config *config = dev->config;
	(void)gpio_pin_set_dt(&config->cs_gpio, 1);
}

static void busy_wait_us(int microseconds)
{
	k_busy_wait(microseconds);
}

uint8_t pmw3901_register_read(const struct device *dev, uint8_t reg)
{
	const struct pmw3901_config *config = dev->config;
	uint8_t data = 0;
	uint8_t tx_data = reg & ~0x80u; /* MSB=0 for read */
	uint8_t dummy = 0;

	struct spi_buf tx_buf = { .buf = &tx_data, .len = 1 };
	struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

	struct spi_buf rx_buf = { .buf = &data, .len = 1 };
	struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

	cs_assert(dev);
	busy_wait_us(50);

	/* Send register address */
	(void)spi_write_dt(&config->spi, &tx_bufs);
	busy_wait_us(500);

	/* Read data (send dummy byte) */
	tx_buf.buf = &dummy;
	(void)spi_transceive_dt(&config->spi, &tx_bufs, &rx_bufs);

	busy_wait_us(50);
	cs_deassert(dev);
	k_msleep(1);

	return data;
}

int pmw3901_register_write(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct pmw3901_config *config = dev->config;
	uint8_t data[2] = { (uint8_t)(reg | 0x80u), value }; /* MSB=1 for write */

	struct spi_buf tx_buf = { .buf = data, .len = 2 };
	struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

	cs_assert(dev);
	busy_wait_us(50);

	(void)spi_write_dt(&config->spi, &tx_bufs);

	busy_wait_us(50);
	cs_deassert(dev);
	k_msleep(1);

	return 0;
}

static void init_registers(const struct device *dev)
{
	/* Known-good init sequence (ported from ESP-IDF implementations) */
	pmw3901_register_write(dev, 0x7F, 0x00);
	pmw3901_register_write(dev, 0x61, 0xAD);
	pmw3901_register_write(dev, 0x7F, 0x03);
	pmw3901_register_write(dev, 0x40, 0x00);
	pmw3901_register_write(dev, 0x7F, 0x05);
	pmw3901_register_write(dev, 0x41, 0xB3);
	pmw3901_register_write(dev, 0x43, 0xF1);
	pmw3901_register_write(dev, 0x45, 0x14);
	pmw3901_register_write(dev, 0x5B, 0x32);
	pmw3901_register_write(dev, 0x5F, 0x34);
	pmw3901_register_write(dev, 0x7B, 0x08);
	pmw3901_register_write(dev, 0x7F, 0x06);
	pmw3901_register_write(dev, 0x44, 0x1B);
	pmw3901_register_write(dev, 0x40, 0xBF);
	pmw3901_register_write(dev, 0x4E, 0x3F);
	pmw3901_register_write(dev, 0x7F, 0x08);
	pmw3901_register_write(dev, 0x65, 0x20);
	pmw3901_register_write(dev, 0x6A, 0x18);
	pmw3901_register_write(dev, 0x7F, 0x09);
	pmw3901_register_write(dev, 0x4F, 0xAF);
	pmw3901_register_write(dev, 0x5F, 0x40);
	pmw3901_register_write(dev, 0x48, 0x80);
	pmw3901_register_write(dev, 0x49, 0x80);
	pmw3901_register_write(dev, 0x57, 0x77);
	pmw3901_register_write(dev, 0x60, 0x78);
	pmw3901_register_write(dev, 0x61, 0x78);
	pmw3901_register_write(dev, 0x62, 0x08);
	pmw3901_register_write(dev, 0x63, 0x50);
	pmw3901_register_write(dev, 0x7F, 0x0A);
	pmw3901_register_write(dev, 0x45, 0x60);
	pmw3901_register_write(dev, 0x7F, 0x00);
	pmw3901_register_write(dev, 0x4D, 0x11);
	pmw3901_register_write(dev, 0x55, 0x80);
	pmw3901_register_write(dev, 0x74, 0x1F);
	pmw3901_register_write(dev, 0x75, 0x1F);
	pmw3901_register_write(dev, 0x4A, 0x78);
	pmw3901_register_write(dev, 0x4B, 0x78);
	pmw3901_register_write(dev, 0x44, 0x08);
	pmw3901_register_write(dev, 0x45, 0x50);
	pmw3901_register_write(dev, 0x64, 0xFF);
	pmw3901_register_write(dev, 0x65, 0x1F);
	pmw3901_register_write(dev, 0x7F, 0x14);
	pmw3901_register_write(dev, 0x65, 0x67);
	pmw3901_register_write(dev, 0x66, 0x08);
	pmw3901_register_write(dev, 0x63, 0x70);
	pmw3901_register_write(dev, 0x7F, 0x15);
	pmw3901_register_write(dev, 0x48, 0x48);
	pmw3901_register_write(dev, 0x7F, 0x07);
	pmw3901_register_write(dev, 0x41, 0x0D);
	pmw3901_register_write(dev, 0x43, 0x14);
	pmw3901_register_write(dev, 0x4B, 0x0E);
	pmw3901_register_write(dev, 0x45, 0x0F);
	pmw3901_register_write(dev, 0x44, 0x42);
	pmw3901_register_write(dev, 0x4C, 0x80);
	pmw3901_register_write(dev, 0x7F, 0x10);
	pmw3901_register_write(dev, 0x5B, 0x02);
	pmw3901_register_write(dev, 0x7F, 0x07);
	pmw3901_register_write(dev, 0x40, 0x41);
	pmw3901_register_write(dev, 0x70, 0x00);

	k_msleep(10);

	pmw3901_register_write(dev, 0x32, 0x44);
	pmw3901_register_write(dev, 0x7F, 0x07);
	pmw3901_register_write(dev, 0x40, 0x40);
	pmw3901_register_write(dev, 0x7F, 0x06);
	pmw3901_register_write(dev, 0x62, 0xF0);
	pmw3901_register_write(dev, 0x63, 0x00);
	pmw3901_register_write(dev, 0x7F, 0x0D);
	pmw3901_register_write(dev, 0x48, 0xC0);
	pmw3901_register_write(dev, 0x6F, 0xD5);
	pmw3901_register_write(dev, 0x7F, 0x00);
	pmw3901_register_write(dev, 0x5B, 0xA0);
	pmw3901_register_write(dev, 0x4E, 0xA8);
	pmw3901_register_write(dev, 0x5A, 0x50);
	pmw3901_register_write(dev, 0x40, 0x80);

	pmw3901_register_write(dev, 0x7F, 0x00);
	pmw3901_register_write(dev, 0x5A, 0x10);
	pmw3901_register_write(dev, 0x54, 0x00);
}

int pmw3901_init(const struct device *dev)
{
	const struct pmw3901_config *config = dev->config;
	struct pmw3901_data *data = dev->data;
	uint8_t chip_id, inv_chip_id;

	if (!spi_is_ready_dt(&config->spi)) {
		printf("PMW3901: SPI not ready\n");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->cs_gpio)) {
		printf("PMW3901: CS GPIO not ready\n");
		return -ENODEV;
	}

	/* Configure CS pin (inactive) */
	(void)gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);

	/* Optional reset pin (active-low) */
	if (config->reset_gpio.port) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			printf("PMW3901: reset GPIO not ready\n");
			return -ENODEV;
		}
		(void)gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
		/* assert then deassert */
		(void)gpio_pin_set_dt(&config->reset_gpio, 1);
		k_msleep(10);
		(void)gpio_pin_set_dt(&config->reset_gpio, 0);
		k_msleep(10);
	}

	k_msleep(40);

	/* CS toggle sequence */
	cs_deassert(dev);
	k_msleep(2);
	cs_assert(dev);
	k_msleep(2);
	cs_deassert(dev);
	k_msleep(2);
	cs_assert(dev);
	k_msleep(2);

	chip_id = pmw3901_register_read(dev, 0x00);
	inv_chip_id = pmw3901_register_read(dev, 0x5F);
	printf("PMW3901 chip ID: 0x%02X, inverted: 0x%02X\n", chip_id, inv_chip_id);

	if (chip_id == 0x49 || inv_chip_id == 0xB6) {
		/* Power on reset */
		pmw3901_register_write(dev, 0x3A, 0x5A);
		k_msleep(5);

		/* Initialize registers */
		init_registers(dev);

		data->last_read_time_us = k_uptime_get() * 1000;
		return 0;
	}

	printf("PMW3901: invalid chip ID\n");
	return -ENODEV;
}

int pmw3901_read_motion_burst(const struct device *dev, motionBurst_t *motion)
{
	const struct pmw3901_config *config = dev->config;
	uint8_t address = 0x16;
	uint8_t rx_buffer[sizeof(motionBurst_t)] = {0};

	struct spi_buf tx_buf = { .buf = &address, .len = 1 };
	struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

	struct spi_buf rx_buf = { .buf = rx_buffer, .len = sizeof(motionBurst_t) };
	struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

	cs_assert(dev);
	k_msleep(1);

	(void)spi_write_dt(&config->spi, &tx_bufs);
	k_msleep(1);

	(void)spi_read_dt(&config->spi, &rx_bufs);

	cs_deassert(dev);
	k_msleep(1);

	memcpy(motion, rx_buffer, sizeof(motionBurst_t));

	/* Fix endianness for shutter value */
	uint16_t real_shutter = (motion->shutter >> 8) & 0x0FF;
	real_shutter |= (motion->shutter & 0x0FF) << 8;
	motion->shutter = real_shutter;

	return 0;
}

int pmw3901_read_motion_count(const struct device *dev, int16_t *deltaX, int16_t *deltaY)
{
	(void)pmw3901_register_read(dev, 0x02);
	*deltaY = -1 * (((int16_t)pmw3901_register_read(dev, 0x04) << 8) |
			pmw3901_register_read(dev, 0x03));
	*deltaX = -1 * (((int16_t)pmw3901_register_read(dev, 0x06) << 8) |
			pmw3901_register_read(dev, 0x05));
	return 0;
}

