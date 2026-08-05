/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motor speed control via PWM for 1 motor (GPIO 21).
 * Starts automatically at full throttle on boot.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec pwm_motor = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor));

/* 20 kHz period (matches overlay), 50 us in nanoseconds */
#define MOTOR_PERIOD_NSEC (50U * 1000U)
/* Full throttle: 100% duty */
#define MOTOR_PULSE_NSEC  MOTOR_PERIOD_NSEC

int main(void)
{
	int ret;

	printk("PWM motor speed control - 1 motor (full throttle)\n");

	if (!pwm_is_ready_dt(&pwm_motor)) {
		printk("Error: PWM device %s is not ready\n", pwm_motor.dev->name);
		return 0;
	}

	printk("Motor on channel %d, period %u nsec, pulse %u nsec\n",
	       pwm_motor.channel, MOTOR_PERIOD_NSEC, MOTOR_PULSE_NSEC);

	ret = pwm_set_dt(&pwm_motor, MOTOR_PERIOD_NSEC, MOTOR_PULSE_NSEC);
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
		return 0;
	}

	printk("Motor set to full throttle (100%%).\n");

	while (1) {
		k_msleep(1000);
	}
	return 0;
}
