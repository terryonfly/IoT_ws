/*
 * I2CBus.c
 *
 *  Created on: 2015年12月16日
 *      Author: terry
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "mraa.h"

#include "I2CBus.h"

#define I2C_BUS_ADDR 		0x06

mraa_i2c_context i2c_context;

void i2cbus_init(void) {
	i2c_context = mraa_i2c_init(I2C_BUS_ADDR);
	mraa_i2c_frequency(i2c_context, MRAA_I2C_STD);
}

void i2cbus_release(void) {
	mraa_i2c_stop(i2c_context);
}

mraa_i2c_context i2cbus_get_instance(void) {
	if (i2c_context == NULL) {
		i2cbus_init();
	}
	return i2c_context;
}
