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
//	printf("bank_select = %02x\n", bank_select_val);
	mraa_i2c_write_byte_data(i2c_context, bank_select_val, 0x22);// BANK_SELECT
}

void max_init_bypass(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);
	// powerdown mode
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b00000111, 0x00);// POWER_CFG
	// SELF_TEST = disabled
	// SNS_LPF_CFG = 2kHz
	// SNS_DOUT_FSC = 2000dps
	mraa_i2c_write_byte_data(i2c_context, 0b00100000, 0x01);// GYRO_CFG1
	// SNS_GYR_OIS_LPF = 1
	// SNS_DOUT_CFG = 0
	// SNS_ODR = 4kHz
	mraa_i2c_write_byte_data(i2c_context, 0b00100001, 0x02);// GYRO_CFG2
	// TODO : GYRO_CFG3
	// SNS_ACC_FSC = 4g
	// ACC_SELF_TEST = desabled
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b10000111, 0x04);// PWR_ACC_CFG
	// SNS_ACC_HPF_CFG = ODR / 50
	// SNS_ACC_LPF_CFG = ODR / 3
	// SNS_ACC_ODR = 2kHz
	mraa_i2c_write_byte_data(i2c_context, 0b11110000, 0x05);// ACC_CFG1
	// SNS_MAG_ODR = ACC_ODR / 16
	// SNS_ACC_LPF_CFG = disabled
	mraa_i2c_write_byte_data(i2c_context, 0b00001001, 0x06);// ACC_CFG2
	// accelero normal + gyro normal mode
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b01111111, 0x00);// POWER_CFG
	// BYP_EN = bypass
	// RW_SEL = write
	// SNGLE_EN = disable single R or single W
	// DR_RST_MODE = ALL - DATA_READY
	// COARSE_TEMP = fine
	// TEMP_EN = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b10000001, 0x13);// DR_CFG
 }

void max_init_master(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);
//	// powerdown mode
//	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
//	mraa_i2c_write_byte_data(i2c_context, 0b00000111, 0x00);// POWER_CFG
	// accelero normal + gyro normal mode
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b01111111, 0x00);// POWER_CFG
	// SELF_TEST = disabled
	// SNS_LPF_CFG = 2kHz
	// SNS_DOUT_FSC = 2000dps
	mraa_i2c_write_byte_data(i2c_context, 0b00100000, 0x01);// GYRO_CFG1
	// SNS_GYR_OIS_LPF = 1
	// SNS_DOUT_CFG = 0
	// SNS_ODR = 4kHz
	mraa_i2c_write_byte_data(i2c_context, 0b00100001, 0x02);// GYRO_CFG2
	// TODO : GYRO_CFG3
	// SNS_ACC_FSC = 4g
	// ACC_SELF_TEST = desabled
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b10000111, 0x04);// PWR_ACC_CFG
	// SNS_ACC_HPF_CFG = ODR / 50
	// SNS_ACC_LPF_CFG = ODR / 3
	// SNS_ACC_ODR = 2kHz
	mraa_i2c_write_byte_data(i2c_context, 0b11110000, 0x05);// ACC_CFG1
	// SNS_MAG_ODR = ACC_ODR / 32
	// SNS_ACC_LPF_CFG = disabled
	mraa_i2c_write_byte_data(i2c_context, 0b00001011, 0x06);// ACC_CFG2
	// MAG_EN = enabled
	// MAG_SWAP = MSB First
	// MAG_SAFE = Reg On
	// MAG_GRP = Group Even -> bit4
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
	// DR_RST_MODE = ALL - DATA_READY
	// COARSE_TEMP = fine
	// TEMP_EN = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b00000001, 0x13);// DR_CFG
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
	// High Speed mode(3400 kHz) = 1
	// Lowest power mode = 0
	// SPI serial interface mode = 4-wire SPI interface
	// Mode Select = Continuous-Measurement Mode
	mraa_i2c_write_byte_data(i2c_context, 0b10000000, 0x02);// Mode Register
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

	int8_t bluk_data[20];
	int actual = mraa_i2c_read_bytes_data(i2c_context, 0x24, (uint8_t *)bluk_data, 20);
//	printf("=== [%d] ===\n", actual);

	int16_t gyr_x =
			bluk_data[0] << 8 |
			bluk_data[1];
	int16_t gyr_y =
			bluk_data[2] << 8 |
			bluk_data[3];
	int16_t gyr_z =
			bluk_data[4] << 8 |
			bluk_data[5];
	int16_t acc_x =
			bluk_data[6] << 8 |
			bluk_data[7];
	int16_t acc_y =
			bluk_data[8] << 8 |
			bluk_data[9];
	int16_t acc_z =
			bluk_data[10] << 8 |
			bluk_data[11];
	int16_t mag_x =
			bluk_data[12] << 8 |
			bluk_data[13];
	int16_t mag_y =
			bluk_data[14] << 8 |
			bluk_data[15];
	int16_t mag_z =
			bluk_data[16] << 8 |
			bluk_data[17];
	int16_t temp =
			bluk_data[18] << 8 |
			bluk_data[19];
//	float temperature = temp / 256.0f;
//	printf("gyr : %5d, %5d, %5d\n", gyr_x, gyr_y, gyr_z);
//	printf("acc : %5d, %5d, %5d\n", acc_x, acc_y, acc_z);
//	printf("mag : %5d, %5d, %5d\n", mag_x, mag_y, mag_z);
//	printf("temp : %.2f degC\n", temperature);

	max_bank_select(i2c_context, 2);
	int8_t quaternion_bluk_data[28];
	actual = mraa_i2c_read_bytes_data(i2c_context, 0x00, (uint8_t *)quaternion_bluk_data, 28);
//	printf("==%d==\n", actual);
	mraa_i2c_read_byte_data(i2c_context, 0x23);//******************
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

	float qx, qy, qz, qw;
	qx = (float) quat[0] * norm;
	qy = (float) quat[1] * norm;
	qz = (float) quat[2] * norm;
	qw = (float) quat[3] * norm;
//	printf("qua : %.2f, %.2f, %.2f, %.2f\n", qx, qy, qz, qw);

	float angle_x = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
	float angle_y = asin(2 * (qw * qy - qz * qx));
	float angle_z = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
//	float angle_x = atan2(mag_z, mag_y);
//	float angle_y = atan2(mag_x, mag_z);
//	float angle_z = atan2(mag_y, mag_x);

	angle_x = angle_x * 180 / M_PI;
	while (angle_x < 0) angle_x += 360.0f;
	angle_y = angle_y * 180 / M_PI;
	while (angle_y < 0) angle_y += 360.0f;
	angle_z = angle_z * 180 / M_PI;
	while (angle_z < 0) angle_z += 360.0f;

//	angle_x = 0.0;
//	angle_y = 0.0;
//	angle_z = 0.0;

	printf("%6.2f %6.2f %6.2f\n", angle_x, angle_y, angle_z);

	unsigned char msg[12];
	int c_i = 0;
	unsigned char *pdata;
	int i;

	pdata = ((unsigned char *)&angle_x);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	pdata = ((unsigned char *)&angle_y);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	pdata = ((unsigned char *)&angle_z);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	tcpserver_send(msg, 12);
}

