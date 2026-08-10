/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * riskybird v3 — Actuator (motor) bring-up
 *
 * Spins each of the four brushed motors one at a time, briefly, at a low PWM
 * duty. Motors are low-side N-FET driven (Q1-Q4 = SI2302); PWM the gate
 * (active-high: higher duty = faster).
 *
 *   Motor  Gate net   ESP32-C6 GPIO   LEDC ch
 *   M1     /MOTOR1     GPIO21          CH0
 *   M2     /MOTOR2     GPIO20          CH1
 *   M3     /MOTOR3     GPIO23          CH2
 *   M4     /MOTOR4     GPIO22          CH3
 *
 * SAFETY: run with PROPELLERS REMOVED. Motors run off +BATT, so the board must
 * be battery-powered (or a bench supply on the +BATT input) for them to spin.
 *
 * Tune SPIN_DUTY_PCT if a motor doesn't start (too low) or spins too hard.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <stdio.h>

#define LEDC_NODE DT_NODELABEL(ledc0)

/* 20 kHz (inaudible); low duty — just enough to spin, NOT flight thrust. */
#define PWM_PERIOD_NS   50000u   /* 20 kHz */
#define SPIN_DUTY_PCT   10u      /* low spin */
#define SPIN_MS         1000u    /* ~1 s per motor */
#define GAP_MS          400u     /* pause between motors */

struct motor {
	const char *name;
	uint32_t channel;
};

static const struct motor motors[] = {
	{ "M1 (GPIO21)", 0 },
	{ "M2 (GPIO20)", 1 },
	{ "M3 (GPIO23)", 2 },
	{ "M4 (GPIO22)", 3 },
};
#define NUM_MOTORS ARRAY_SIZE(motors)

int main(void)
{
	const struct device *pwm = DEVICE_DT_GET(LEDC_NODE);
	const uint32_t pulse_ns =
		(uint32_t)((uint64_t)PWM_PERIOD_NS * SPIN_DUTY_PCT / 100u);
	int ret;

	printf("\nriskybird actuator bring-up (sequential motor spin)\n");
	printf("*** REMOVE PROPELLERS ***  duty=%u%%  freq=%u Hz  %u ms each\n\n",
	       SPIN_DUTY_PCT, 1000000000u / PWM_PERIOD_NS, SPIN_MS);

	if (!device_is_ready(pwm)) {
		printf("ERROR: LEDC PWM device not ready\n");
		return 1;
	}

	/* Force everything off before we begin. */
	for (int i = 0; i < NUM_MOTORS; i++) {
		(void)pwm_set(pwm, motors[i].channel, PWM_PERIOD_NS, 0, 0);
	}
	k_msleep(300);

	/* One pass: spin each motor briefly, in sequence, then stop. */
	for (int i = 0; i < NUM_MOTORS; i++) {
		printf("  %s  ON  (%u%%)\n", motors[i].name, SPIN_DUTY_PCT);
		ret = pwm_set(pwm, motors[i].channel, PWM_PERIOD_NS, pulse_ns, 0);
		if (ret != 0) {
			printf("  ERROR: pwm_set(ch%u) = %d\n",
			       motors[i].channel, ret);
		}
		k_msleep(SPIN_MS);

		/* Always stop this motor before moving to the next. */
		(void)pwm_set(pwm, motors[i].channel, PWM_PERIOD_NS, 0, 0);
		printf("  %s  off\n", motors[i].name);
		k_msleep(GAP_MS);
	}

	printf("\nPass complete — all motors off. Reset the board to run again.\n");
	return 0;
}
