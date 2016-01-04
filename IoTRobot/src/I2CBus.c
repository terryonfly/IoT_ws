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

mraa_i2c_context i2c_context_mpu;
mraa_i2c_context i2c_context_pca;

void i2cbus_init(void) {
	i2c_context_mpu = mraa_i2c_init(0x06);
	mraa_i2c_frequency(i2c_context_mpu, MRAA_I2C_FAST);
	i2c_context_pca = mraa_i2c_init(0x01);
	mraa_i2c_frequency(i2c_context_pca, MRAA_I2C_FAST);
}

void i2cbus_release(void) {
	mraa_i2c_stop(i2c_context_mpu);
	mraa_i2c_stop(i2c_context_pca);
}

mraa_i2c_context i2cbus_get_instance_mpu(void) {
	if (i2c_context_mpu == NULL) {
		i2cbus_init();
	}
	return i2c_context_mpu;
}

mraa_i2c_context i2cbus_get_instance_pca(void) {
	if (i2c_context_pca == NULL) {
		i2cbus_init();
	}
	return i2c_context_pca;
}
