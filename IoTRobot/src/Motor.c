/*
 * Motor.c
 *
 *  Created on: Dec 13, 2015
 *      Author: terry
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "mraa.h"

mraa_pwm_context pwm_context = NULL;;

void motor_init(void) {
	// 0 14 20 21
	pwm_context = mraa_pwm_init(0);
	if (pwm_context == NULL)
		printf("PWM init error.\n");
	mraa_pwm_period_ms(pwm_context, 5);
	mraa_pwm_write(pwm_context, 0.0);
	mraa_pwm_enable(pwm_context, 1);
}

void motor_release(void) {
	mraa_pwm_enable(pwm_context, 0);
	mraa_pwm_close(pwm_context);
}

void motor_run(int persent) {
	float orr = persent / 100.0f;
	printf("%.2f\n", orr);
	mraa_pwm_pulsewidth_us(pwm_context, orr * 2000 + 500);
}
