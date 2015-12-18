/*
 * MAX21100.c
 *
 *  Created on: 2015年12月16日
 *      Author: terry
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "mraa.h"

#include "MAX21100.h"
#include "I2CBus.h"

#define MAX21100_ADDR		0x58
#define HMC5983_ADDR 		0x1E

void max_bank_select(mraa_i2c_context i2c_context, uint8_t bank) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	if (bank < 0x00 || bank > 0x02) return;
	uint8_t bank_select_val = mraa_i2c_read_byte_data(i2c_context, 0x22);// BANK_SELECT
	bank_select_val &= 0xF0;
	bank_select_val |= bank;
	printf("bank_select = %02x\n", bank_select_val);
	mraa_i2c_write_byte_data(i2c_context, bank_select_val, 0x22);// BANK_SELECT
}

void max_init_bypass(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);
	// accelero normal + gyro normal mode
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b01111111, 0x00);// POWER_CFG
	// SELF_TEST = disabled
	// SNS_LPF_CFG = 250Hz
	// SNS_DOUT_FSC = 2000dps
	mraa_i2c_write_byte_data(i2c_context, 0b00110100, 0x01);// GYRO_CFG1
	// SNS_GYR_OIS_LPF = 0
	// SNS_DOUT_CFG = 0
	// SNS_ODR = 250Hz
	mraa_i2c_write_byte_data(i2c_context, 0b00000101, 0x02);// GYRO_CFG2
	// SNS_ACC_FSC = 2g
	// ACC_SELF_TEST = desabled
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b11000111, 0x04);// PWR_ACC_CFG
	// SNS_ACC_HPF_CFG = ODR / 400
	// SNS_ACC_LPF_CFG = ODR / 9
	// SNS_ACC_ODR = 250Hz
	mraa_i2c_write_byte_data(i2c_context, 0b00100011, 0x05);// ACC_CFG1
	// BYP_EN = bypass
	// RW_SEL = write
	// SNGLE_EN = disable single R or single W
	// DR_RST_MODE = STATUS - DATA_READY
	// COARSE_TEMP = fine
	// TEMP_EN = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b10001001, 0x13);// DR_CFG
 }

void max_init_master(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);
	// accelero normal + gyro normal mode
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b01111111, 0x00);// POWER_CFG
	// SELF_TEST = disabled
	// SNS_LPF_CFG = 250Hz
	// SNS_DOUT_FSC = 2000dps
	mraa_i2c_write_byte_data(i2c_context, 0b00110100, 0x01);// GYRO_CFG1
	// SNS_GYR_OIS_LPF = 0
	// SNS_DOUT_CFG = 0
	// SNS_ODR = 250Hz
	mraa_i2c_write_byte_data(i2c_context, 0b00000101, 0x02);// GYRO_CFG2
	// SNS_ACC_FSC = 2g
	// ACC_SELF_TEST = desabled
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b11000111, 0x04);// PWR_ACC_CFG
	// SNS_ACC_HPF_CFG = ODR / 400
	// SNS_ACC_LPF_CFG = ODR / 9
	// SNS_ACC_ODR = 250Hz
	mraa_i2c_write_byte_data(i2c_context, 0b00100011, 0x05);// ACC_CFG1
	// SNS_MAG_ODR = ACC_ODR / 16
	// SNS_ACC_LPF_CFG = disabled
	mraa_i2c_write_byte_data(i2c_context, 0b00001000, 0x06);// ACC_CFG2
	// MAG_EN = enabled
	// MAG_SWAP = MSB First
	// MAG_SAFE = Reg On
	// MAG_GRP = Group Even
	// I2C_STD = 400kHz
	// MAG_I2C_LEN = 6
	mraa_i2c_write_byte_data(i2c_context, 0b10000110, 0x07);// MAG_SLV_CFG
	mraa_i2c_write_byte_data(i2c_context, HMC5983_ADDR, 0x08);// MAG_SLV_ADD
	mraa_i2c_write_byte_data(i2c_context, 0x03, 0x09);// MAG_SLV_REG
	// MAG_CH_MAP : [Xi, Yi, Zi] <= [A, C, B]
	// INV_Z = 0
	// INV_Y = 0
	// INV_X = 0
	mraa_i2c_write_byte_data(i2c_context, 0b00001000, 0x0A);// MAG_MAP_REG
	// BYP_EN = I2C_MASTER active
	// RW_SEL = write
	// SNGLE_EN = disable single R or single W
	// DR_RST_MODE = STATUS - DATA_READY
	// COARSE_TEMP = fine
	// TEMP_EN = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b00001001, 0x13);// DR_CFG
}

void hmc_init(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, HMC5983_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", HMC5983_ADDR);
	// enable temperature sensor = 0
	// number of samples averaged per measurement output = 1
	// Data Output Rate = 220Hz
	// Measurement Configuration = Normal
	mraa_i2c_write_byte_data(i2c_context, 0b00011100, 0x00);// Configuration Register A
	// Gain Configuration = 1090LSb/Gauss
	mraa_i2c_write_byte_data(i2c_context, 0b00100000, 0x01);// Configuration Register B
	// High Speed mode(3400 kHz) = 0
	// Lowest power mode = 0
	// SPI serial interface mode = 4-wire SPI interface
	// Mode Select = Continuous-Measurement Mode
	mraa_i2c_write_byte_data(i2c_context, 0b00000000, 0x02);// Mode Register
}

void max_init(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);

	uint8_t who_am_i_val = mraa_i2c_read_byte_data(i2c_context, 0x20);
	printf("MAX who_am_i_val = 0x%02x\n", who_am_i_val);

	max_init_bypass(i2c_context);
	hmc_init(i2c_context);
	max_init_master(i2c_context);
	max_bank_select(i2c_context, 2);
	// FUS_ODR = SNS_ODR / 1
	// FUS_GR_HD = 1
	// FUS_AUTO_MODE = 1
	// FUS_MODE = Gryo + Accelerometer + Magnetometer
	// FUS_EN = 1
	mraa_i2c_write_byte_data(i2c_context, 0b00011001, 0x1C);// FUS_CFG_0
}

void max_release(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
}

void max_run() {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);

	max_bank_select(i2c_context, 0);
	uint8_t status = mraa_i2c_read_byte_data(i2c_context, 0x23);

	int8_t bluk_data[20];
	int actual = mraa_i2c_read_bytes_data(i2c_context, 0x24, (uint8_t *)bluk_data, 20);
	printf("=== [%d] === 0x%02x\n", actual, status);

	float gyr_x = (float)(
			bluk_data[0] << 8 |
			bluk_data[1]);
	float gyr_y = (float)(
			bluk_data[2] << 8 |
			bluk_data[3]);
	float gyr_z = (float)(
			bluk_data[4] << 8 |
			bluk_data[5]);
	float acc_x = (float)(
			bluk_data[6] << 8 |
			bluk_data[7]);
	float acc_y = (float)(
			bluk_data[8] << 8 |
			bluk_data[9]);
	float acc_z = (float)(
			bluk_data[10] << 8 |
			bluk_data[11]);
	float mag_x = (float)(
			bluk_data[12] << 8 |
			bluk_data[13]);
	float mag_y = (float)(
			bluk_data[14] << 8 |
			bluk_data[15]);
	float mag_z = (float)(
			bluk_data[16] << 8 |
			bluk_data[17]);
	float temp = (float)(
			bluk_data[18] << 8 |
			bluk_data[19]);
	printf("gyr : %.2f, %.2f, %.2f\n", gyr_x, gyr_y, gyr_z);
	printf("acc : %.2f, %.2f, %.2f\n", acc_x, acc_y, acc_z);
	printf("mag : %.2f, %.2f, %.2f\n", mag_x, mag_y, mag_z);
	printf("temp : %.2f degC\n", temp);

	max_bank_select(i2c_context, 2);
	int8_t quaternion_bluk_data[28];
	actual = mraa_i2c_read_bytes_data(i2c_context, 0x00, (uint8_t *)quaternion_bluk_data, 28);
	printf("==%d==\n", actual);
	int16_t quat[4];
	quat[0] =
			quaternion_bluk_data[0] << 8 |
			quaternion_bluk_data[1];
	quat[1] =
			quaternion_bluk_data[2] << 8 |
			quaternion_bluk_data[3];
	quat[2] =
			quaternion_bluk_data[4] << 8 |
			quaternion_bluk_data[5];
	quat[3] =
			quaternion_bluk_data[6] << 8 |
			quaternion_bluk_data[7];
	float norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
	norm = 1.0f/norm;

	float quaternion[4];
	quaternion[0] = (float) quat[0] * norm;
	quaternion[1] = (float) quat[1] * norm;
	quaternion[2] = (float) quat[2] * norm;
	quaternion[3] = (float) quat[3] * norm;
	printf("qua : %.2f, %.2f, %.2f, %.2f\n", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
}

