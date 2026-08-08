/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motor speed control via PWM for 4 motors (GPIOs 20, 21, 22, 23).
 * UART: 's' start at current %%, 'x'/' '/'q' stop, 0-100 = set throttle %%.
 * Uses UART1 when motor_uart alias is defined, else console UART.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <stdlib.h>

static const struct pwm_dt_spec pwm_motor0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0));
static const struct pwm_dt_spec pwm_motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));
static const struct pwm_dt_spec pwm_motor2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor2));
static const struct pwm_dt_spec pwm_motor3 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor3));

#if DT_NODE_EXISTS(DT_ALIAS(motor_uart))
#define UART_NODE DT_ALIAS(motor_uart)
#else
#define UART_NODE DT_CHOSEN(zephyr_console)
#endif
static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

/* 20 kHz period (matches overlay), 50 us in nanoseconds */
#define MOTOR_PERIOD_NSEC (50U * 1000U)

#define THROTTLE_BUF_LEN 4
static uint8_t throttle_pct = 10; /* 0-100, default 10% */
static char throttle_buf[THROTTLE_BUF_LEN];
static int throttle_buf_len;

static int set_throttle(uint8_t pct)
{
	uint32_t pulse = (MOTOR_PERIOD_NSEC * (uint32_t)pct) / 100U;
	int ret;

	ret = pwm_set_dt(&pwm_motor0, MOTOR_PERIOD_NSEC, pulse);
	if (ret) return ret;
	ret = pwm_set_dt(&pwm_motor1, MOTOR_PERIOD_NSEC, pulse);
	if (ret) return ret;
	ret = pwm_set_dt(&pwm_motor2, MOTOR_PERIOD_NSEC, pulse);
	if (ret) return ret;
	ret = pwm_set_dt(&pwm_motor3, MOTOR_PERIOD_NSEC, pulse);
	if (ret) return ret;
	throttle_pct = pct;
	return 0;
}

static void apply_throttle_number(void)
{
	if (throttle_buf_len == 0) return;
	throttle_buf[throttle_buf_len] = '\0';
	unsigned long val = strtoul(throttle_buf, NULL, 10);
	if (val > 100U) val = 100U;
	if (set_throttle((uint8_t)val) == 0) {
		printk("Throttle %u%%\n", (unsigned int)(uint8_t)val);
	}
	throttle_buf_len = 0;
}

static void process_uart_byte(uint8_t byte)
{
	if (byte >= '0' && byte <= '9') {
		if (throttle_buf_len < THROTTLE_BUF_LEN - 1) {
			throttle_buf[throttle_buf_len++] = (char)byte;
		}
		return;
	}
	/* Non-digit: apply any pending number then handle command */
	apply_throttle_number();

	switch (byte) {
	case 's':
	case 'S':
		if (set_throttle(throttle_pct) == 0) {
			printk("Motors START (%u%%)\n", throttle_pct);
		}
		break;
	case 'x':
	case 'X':
	case ' ':
	case 'q':
	case 'Q':
		if (set_throttle(0) == 0) {
			printk("Motors STOP\n");
		}
		break;
	case '\r':
	case '\n':
		/* already applied in apply_throttle_number */
		break;
	default:
		break;
	}
}

int main(void)
{
	int ret;
	uint8_t byte;

	printk("PWM motor control - 4 motors, UART throttle\n");
	printk("Commands: s = start, x/space/q = stop, 0-100 = throttle %%\n");

	if (!pwm_is_ready_dt(&pwm_motor0) || !pwm_is_ready_dt(&pwm_motor1) ||
	    !pwm_is_ready_dt(&pwm_motor2) || !pwm_is_ready_dt(&pwm_motor3)) {
		printk("Error: One or more PWM devices not ready\n");
		return 0;
	}

	if (!device_is_ready(uart_dev)) {
		printk("Error: UART (motor commands) not ready\n");
		return 0;
	}
#if DT_NODE_EXISTS(DT_ALIAS(motor_uart))
	printk("Motor commands on UART1 (GPIO12 TX / GPIO13 RX, 115200)\n");
#else
	printk("Motor commands on console (this serial, 115200)\n");
#endif

	printk("Motors on channels %d,%d,%d,%d, period %u nsec\n",
	       pwm_motor0.channel, pwm_motor1.channel,
	       pwm_motor2.channel, pwm_motor3.channel,
	       MOTOR_PERIOD_NSEC);

	/* Start with motors stopped (0% throttle) */
	(void)set_throttle(0);

	while (1) {
		/* Poll UART for start/stop commands */
		if (uart_poll_in(uart_dev, &byte) == 0) {
			process_uart_byte(byte);
		}
		k_msleep(10);
	}
	return 0;
}
