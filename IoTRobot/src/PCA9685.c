/*
 * Servo.c
 *
 *  Created on: Dec 15, 2015
 *      Author: terry
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "mraa.h"

#include "PCA9685.h"
#include "I2CBus.h"

#include "StatusReport.h"

#define PCA9685_ADDR		0x40

void pca_init(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance_pca();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, PCA9685_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", PCA9685_ADDR);

	// Sleep = 0
	uint8_t mode1_val = mraa_i2c_read_byte_data(i2c_context, 0x00);
	mode1_val &= 0xEF;
	mraa_i2c_write_byte_data(i2c_context, mode1_val, 0x00);
	// update_rate = 333Hz
	mraa_i2c_write_byte_data(i2c_context, 0x12, 0xFE);
	// 0 on
	mraa_i2c_write_byte_data(i2c_context, 0x00, 0x06);
	mraa_i2c_write_byte_data(i2c_context, 0x00, 0x07);
	// 1 on
	mraa_i2c_write_byte_data(i2c_context, 0x00, 0x0A);
	mraa_i2c_write_byte_data(i2c_context, 0x00, 0x0B);
	// 2 on
	mraa_i2c_write_byte_data(i2c_context, 0x00, 0x0E);
	mraa_i2c_write_byte_data(i2c_context, 0x00, 0x0F);
	// 3 on
	mraa_i2c_write_byte_data(i2c_context, 0x00, 0x12);
	mraa_i2c_write_byte_data(i2c_context, 0x00, 0x13);
}

void pca_release(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance_pca();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, PCA9685_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", PCA9685_ADDR);
}

void pca_run(float left_angle, float right_angle, float left_power, float right_power) {
	mraa_i2c_context i2c_context = i2cbus_get_instance_pca();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, PCA9685_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", PCA9685_ADDR);

	sync_action(left_angle, right_angle, left_power, right_power);

	uint16_t on_val_0 = 800 + left_angle * 950;
	uint8_t *on_pdata_0 = ((uint8_t *)&on_val_0);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_0 ++, 0x08);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_0 ++, 0x09);

	uint16_t on_val_1 = 800 + (1 - right_angle) * 950;
	uint8_t *on_pdata_1 = ((uint8_t *)&on_val_1);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_1 ++, 0x0C);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_1 ++, 0x0D);

	uint16_t on_val_2 = 1312 + left_power * 500;// 1365
	uint8_t *on_pdata_2 = ((uint8_t *)&on_val_2);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_2 ++, 0x10);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_2 ++, 0x11);

	uint16_t on_val_3 = 950 + right_power * 500;//1003
	uint8_t *on_pdata_3 = ((uint8_t *)&on_val_3);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_3 ++, 0x14);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_3 ++, 0x15);
}
