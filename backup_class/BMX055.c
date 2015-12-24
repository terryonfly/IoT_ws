/*
 * BMX055.c
 *
 *  Created on: Dec 23, 2015
 *      Author: terry
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "mraa.h"

#include "BMX055.h"
#include "I2CBus.h"

#define BMX055_ADDR		0x69

void bmx_init(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, BMX055_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", BMX055_ADDR);

	uint8_t who_am_i_val = mraa_i2c_read_byte_data(i2c_context, 0x00);
	printf("BMX055 who_am_i_val = 0x%02x\n", who_am_i_val);

//	mraa_result_t ret = mraa_i2c_write_byte_data(i2c_context, 0x01, 0x00);
//	printf("ret = %d\n", ret);
//
//	who_am_i_val = mraa_i2c_read_byte_data(i2c_context, 0x00);
//	printf("HMC5883L who_am_i_val = 0x%02x\n", who_am_i_val);
}

void bmx_release(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, BMX055_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", BMX055_ADDR);
}

void bmx_run() {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, BMX055_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", BMX055_ADDR);
}
