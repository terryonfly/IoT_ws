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
#include "../Common/I2CBus.h"
#include "../StatusReport/StatusReport.h"

#define PCA9685_ADDR		0x40

uint8_t pca_who_am_i = 0x00;

#define LEFT_ANGLE_FRONT 	1682
#define LEFT_ANGLE_MIDDLE 	1261
#define LEFT_ANGLE_BACK 	857
#define RIGHT_ANGLE_FRONT 	844
#define RIGHT_ANGLE_MIDDLE 	1247
#define RIGHT_ANGLE_BACK 	1672

float left_angle_front_plus = LEFT_ANGLE_FRONT - LEFT_ANGLE_MIDDLE;
float left_angle_back_plus = LEFT_ANGLE_MIDDLE - LEFT_ANGLE_BACK;
float right_angle_front_plus = RIGHT_ANGLE_FRONT - RIGHT_ANGLE_MIDDLE;
float right_angle_back_plus = RIGHT_ANGLE_MIDDLE - RIGHT_ANGLE_BACK;

#define LEFT_POWER_OFF 		900
#define LEFT_POWER_MIN 		1373
#define LEFT_POWER_MAX 		1630// 1693
#define RIGHT_POWER_OFF 	900
#define RIGHT_POWER_MIN 	1011
#define RIGHT_POWER_MAX 	1345// 1442

float left_power_plus = LEFT_POWER_MAX - LEFT_POWER_MIN;
float right_power_plus = RIGHT_POWER_MAX - RIGHT_POWER_MIN;

void pca_init(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance_pca();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, PCA9685_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", PCA9685_ADDR);

	pca_who_am_i = mraa_i2c_read_byte_data(i2c_context, 0x00);
	if (pca_who_am_i == 0x00) {
		printf("PCA is not connected.\n");
		return;
	}

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

	pca_run();
	usleep(2 * 1000 * 1000);
}

void pca_release(void) {
	if (pca_who_am_i == 0x00) return;
	mraa_i2c_context i2c_context = i2cbus_get_instance_pca();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, PCA9685_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", PCA9685_ADDR);
}

void pca_run() {
	if (pca_who_am_i == 0x00) return;
	mraa_i2c_context i2c_context = i2cbus_get_instance_pca();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, PCA9685_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", PCA9685_ADDR);

	sync_action(left_angle, right_angle, left_power, right_power);

	uint16_t on_val_0 = LEFT_ANGLE_MIDDLE;
	if (left_angle > 0)
		on_val_0 += left_angle_front_plus * left_angle / 45.0;
	if (left_angle < 0)
		on_val_0 += left_angle_back_plus * left_angle / 45.0;
	uint8_t *on_pdata_0 = ((uint8_t *)&on_val_0);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_0 ++, 0x08);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_0 ++, 0x09);

	uint16_t on_val_1 = RIGHT_ANGLE_MIDDLE;
	if (right_angle > 0)
		on_val_1 += right_angle_front_plus * right_angle / 45.0;
	if (right_angle < 0)
		on_val_1 += right_angle_back_plus * right_angle / 45.0;
	uint8_t *on_pdata_1 = ((uint8_t *)&on_val_1);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_1 ++, 0x0C);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_1 ++, 0x0D);

	uint16_t on_val_2 = LEFT_POWER_OFF;
	if (left_power > 0) {
		if (left_power > 100)
			on_val_2 = LEFT_POWER_MAX;
		else
			on_val_2 = LEFT_POWER_MIN + left_power * left_power_plus / 100.0;
	}
	uint8_t *on_pdata_2 = ((uint8_t *)&on_val_2);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_2 ++, 0x10);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_2 ++, 0x11);

	uint16_t on_val_3 = RIGHT_POWER_OFF;
	if (right_power > 0) {
		if (right_power > 100)
			on_val_3 = RIGHT_POWER_MAX;
		else
			on_val_3 = RIGHT_POWER_MIN + right_power * right_power_plus / 100.0;
	}
	uint8_t *on_pdata_3 = ((uint8_t *)&on_val_3);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_3 ++, 0x14);
	mraa_i2c_write_byte_data(i2c_context, *on_pdata_3 ++, 0x15);
}
