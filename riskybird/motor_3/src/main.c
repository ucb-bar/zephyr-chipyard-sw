/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motor speed control via PWM for 3 motors (GPIOs 20, 21, 22).
 * Same 20 kHz, high duty as the working single-motor setup.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec pwm_motor0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0));
static const struct pwm_dt_spec pwm_motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));
static const struct pwm_dt_spec pwm_motor2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor2));

/* 20 kHz period (matches overlay), 50 us in nanoseconds */
#define MOTOR_PERIOD_NSEC (50U * 1000U)
/* High throttle: 95% duty */
#define MOTOR_PULSE_NSEC  ((MOTOR_PERIOD_NSEC * 95U) / 100U)

int main(void)
{
	int ret;

	printk("PWM motor speed control - 3 motors (high throttle)\n");

	if (!pwm_is_ready_dt(&pwm_motor0) || !pwm_is_ready_dt(&pwm_motor1) ||
	    !pwm_is_ready_dt(&pwm_motor2)) {
		printk("Error: One or more PWM devices not ready\n");
		return 0;
	}

	printk("Motors on channels %d,%d,%d, period %u nsec, pulse %u nsec\n",
	       pwm_motor0.channel, pwm_motor1.channel, pwm_motor2.channel,
	       MOTOR_PERIOD_NSEC, MOTOR_PULSE_NSEC);

	ret = pwm_set_dt(&pwm_motor0, MOTOR_PERIOD_NSEC, MOTOR_PULSE_NSEC);
	if (ret) {
		printk("Error %d: motor 0 (GPIO20)\n", ret);
		return 0;
	}

	ret = pwm_set_dt(&pwm_motor1, MOTOR_PERIOD_NSEC, MOTOR_PULSE_NSEC);
	if (ret) {
		printk("Error %d: motor 1 (GPIO21)\n", ret);
		return 0;
	}

	ret = pwm_set_dt(&pwm_motor2, MOTOR_PERIOD_NSEC, MOTOR_PULSE_NSEC);
	if (ret) {
		printk("Error %d: motor 2 (GPIO22)\n", ret);
		return 0;
	}

	printk("All 3 motors at high throttle.\n");

	while (1) {
		k_msleep(1000);
	}
	return 0;
}
