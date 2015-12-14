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
	// 20 PWM0
	// 14 PWM1
	// 0 PWM2
	// 21 PWM3
	int pin = 20;
	mraa_gpio_context gpio;
	gpio = mraa_gpio_init(pin);
	mraa_gpio_dir(gpio, MRAA_GPIO_OUT);
	mraa_gpio_write(gpio, 1);
	pwm_context = mraa_pwm_init(pin);
	if (pwm_context == NULL)
		printf("%d PWM init error.\n", pin);
	mraa_pwm_period_us(pwm_context, 2500);
	mraa_pwm_write(pwm_context, 0.2);
	mraa_pwm_enable(pwm_context, 1);
}

void motor_release(void) {
	mraa_pwm_enable(pwm_context, 0);
	mraa_pwm_close(pwm_context);
}

void motor_run(float persent) {
//	printf("%.2f\n", persent);
//	float motor_persent = 0.2 + persent * 0.8;
//	mraa_pwm_write(pwm_context, motor_persent);
	float servo_persent = 0.2 + persent * 0.8;
	mraa_pwm_write(pwm_context, servo_persent);
}
