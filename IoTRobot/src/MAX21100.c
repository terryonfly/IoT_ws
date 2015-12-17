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

uint8_t Gscale = GFS_250DPS;  // Gyro full scale
uint8_t Godr = GODR_250Hz;    // Gyro sample rate
uint8_t Gbw = GBW_22Hz;       // Gyro bandwidth
uint8_t Ascale = AFS_2G;      // Accel full scale
uint8_t Aodr = AODR_250Hz;    // Accel sample rate
uint8_t Abw = ABW_div9;       // Accel bandwidth, accel sample rate divided by ABW_divx
uint8_t MSodr = MODR_div16;   // Select magnetometer ODR as Aodr/MODR_divx
uint8_t powerSelect = 0x00;   // no DSYNC enable or usage
uint8_t powerMode = accelnormalgyronormalmode;  // specify power mode for accel + gyro

// Initialize the MAX21100 for bypass mode operations (read from magnetometer directly via microcontroller
void initbypassMAX21100(mraa_i2c_context i2c_context) {
	// Enable power select to control power mode from DSYNC (enable = 0x80, disable = 0x00)
	// choose power mode (accelnormal_gyronormal = 0xF in bits 6:3
	// Enable all axes (z = bit 2, y = bit 1, x = bit 0)
	mraa_i2c_write_byte_data(i2c_context, powerSelect | powerMode << 3 | 0x07, MAX21100_POWER_CFG);

	// Configure gyro
	// Select gyro bandwidth (bits 5:2) and gyro full scale (bits 1:0)
	mraa_i2c_write_byte_data(i2c_context, Gbw << 2 | Gscale, MAX21100_GYRO_CFG1);
	// Select gyro ODR (bits 1:0)
	mraa_i2c_write_byte_data(i2c_context, Godr, MAX21100_GYRO_CFG2);

	// Configure the accelerometer
	// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
	mraa_i2c_write_byte_data(i2c_context, Ascale << 6 | 0x07, MAX21100_PWR_ACC_CFG);
	// Select accel band width (bits 5:4) and accel ODR (bits 3:0)
	mraa_i2c_write_byte_data(i2c_context, Abw << 4 | Aodr, MAX21100_ACC_CFG1);

	// Data ready configuration
	// Enable bypass mode to read magnetometer directly from microcontroller (bit 7 = 1)
	// Clear data ready bits when status register is read (bits 3:2 = 10)
	// Enable fine temperature mode, enable temperature sensor (bits 1:0 = 01)
	mraa_i2c_write_byte_data(i2c_context, 0x80 | 0x08 | 0x01, MAX21100_DR_CFG);
 }

// Initialize the MAX21100 for master mode operations (read from magnetometer from MAX21100 master)
void initmasterMAX21100(mraa_i2c_context i2c_context) {
	// Enable power select to control power mode from DSYNC (enable = 0x80, disable = 0x00)
	// choose power mode (accelnormal_gyronormal = 0xFF in bits 6:3
	// Enable all axes (z = bit 2, y = bit 1, x = bit 0)
	mraa_i2c_write_byte_data(i2c_context, powerSelect | powerMode << 3 | 0x07, MAX21100_POWER_CFG);

	// Configure gyro
	// Select gyro bandwidth (bits 5:2) and gyro full scale (bits 1:0)
	mraa_i2c_write_byte_data(i2c_context, Gbw << 2 | Gscale, MAX21100_GYRO_CFG1);
	// Select gyro ODR (bits 1:0)
	mraa_i2c_write_byte_data(i2c_context, Godr, MAX21100_GYRO_CFG2);

	// Configure the accelerometer
	// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
	mraa_i2c_write_byte_data(i2c_context, Ascale << 6 | 0x07, MAX21100_PWR_ACC_CFG);
	// Select accel band width (bits 5:4) and accel ODR (bits 3:0)
	mraa_i2c_write_byte_data(i2c_context, Abw << 4 | Aodr, MAX21100_ACC_CFG1);

	// Configure magnetometer in slave mode
	mraa_i2c_write_byte_data(i2c_context, MSodr << 1, MAX21100_ACC_CFG2);           // slave magnetometer ODR
	// magnetometer slave enable (bit 7 = 1), byte order swap (LSB first, bit 6 = 1), byte length = 6
	mraa_i2c_write_byte_data(i2c_context, 0x80 | 0x40 | 0x06, MAX21100_MAG_SLV_CFG);
	mraa_i2c_write_byte_data(i2c_context, LIS3MDL_ADDR, MAX21100_MAG_SLV_ADD);  // magnetometer slave address
	mraa_i2c_write_byte_data(i2c_context, LIS3MDL_OUT_X_L, MAX21100_MAG_SLV_REG);  // magnetometer slave first data register
	mraa_i2c_write_byte_data(i2c_context, 0x00, MAX21100_MAG_MAP_REG);             // magnetometer has inverted x,y axes wrt the MAX21100

	// Data ready configuration
	// Enable master mode to read magnetometer from MAX21100 (bit 7 = 0, default)
	// Clear data ready bits when status register is read (bits 3:2 = 10)
	// Enable fine temperature mode, enable temperature sensor (bits 1:0 = 01)
	mraa_i2c_write_byte_data(i2c_context, 0x08 | 0x01, MAX21100_DR_CFG);
}

void check_i2c_addrs(mraa_i2c_context i2c_context) {
	int i;
	for (i = 0; i < 255; i++) {
		if (mraa_i2c_address(i2c_context, i) != MRAA_SUCCESS) {
			printf("can not found 0x%02x sensor\n", i);
		} else {
			uint8_t who_am_i_val = mraa_i2c_read_byte_data(i2c_context, 0x20);
			printf("MAX 0x%02x who_am_i_val = 0x%02x\n", i, who_am_i_val);
		}
	}
}

void max_bank_select(mraa_i2c_context i2c_context, uint8_t bank) {
	if (bank < 0x00 || bank > 0x02) return;
	uint8_t bank_select_val = mraa_i2c_read_byte_data(i2c_context, MAX21100_BANK_SELECT);
	bank_select_val &= 0xF0;
	bank_select_val |= bank;
	printf("bank_select = %02x\n", bank_select_val);
	mraa_i2c_write_byte_data(i2c_context, bank_select_val, MAX21100_BANK_SELECT);
}

void max_init(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);

	uint8_t who_am_i_val = mraa_i2c_read_byte_data(i2c_context, 0x20);
	printf("MAX who_am_i_val = 0x%02x\n", who_am_i_val);

	max_bank_select(i2c_context, 0);
	mraa_i2c_write_byte_data(i2c_context, 0x07, 0x00);// MAX21100_POWER_CFG
//	mraa_i2c_write_byte_data(i2c_context, 0x10, 0x01);// MAX21100_GYRO_CFG1 10Hz BW
//	mraa_i2c_write_byte_data(i2c_context, 0x01, 0x02);// MAX21100_GYRO_CFG2 4KHz ODR
	mraa_i2c_write_byte_data(i2c_context, 0xC7, 0x04);
	mraa_i2c_write_byte_data(i2c_context, 0x17, 0x00);// MAX21100_POWER_CFG
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

	while (mraa_i2c_read_byte_data(i2c_context, 35) == 0x00) {
		printf("---\n");
		usleep(1000);
	}

	int8_t bluk_data[20];
	int actual = mraa_i2c_read_bytes_data(i2c_context, 0x24, bluk_data, 20);
	printf("=== [%d] ===\n", actual);

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
}

