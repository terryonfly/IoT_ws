/*
 * TenDofSensor.c
 *
 *  Created on: 2015年12月14日
 *      Author: terry
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "mraa.h"

#include "TenDofSensor.h"

#define I2C_BUS 		0x06
#define L3G4200D_ADDR 	0x69
#define ADXL345_ADDR 	0x53
#define HMC5883L_ADDR 	0x1E
#define BMP085_ADDR 	0x77

mraa_i2c_context i2c_context;

mraa_result_t tendof_i2c_write_byte_data(mraa_i2c_context dev, const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t data) {
	if (mraa_i2c_address(i2c_context, dev_addr) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", dev_addr);
	mraa_result_t ret = mraa_i2c_write_byte_data(i2c_context, data, reg_addr);
	printf("write to 0x%02x 0x%02x -> 0x%02x %s\n", dev_addr, reg_addr, data, (ret == MRAA_SUCCESS) ? "SUCCESS" : "FAIL");
	return ret;
}

uint8_t tendof_i2c_read_byte_data(mraa_i2c_context dev, const uint8_t dev_addr, const uint8_t reg_addr) {
	if (mraa_i2c_address(i2c_context, dev_addr) != MRAA_SUCCESS)
			printf("can not found 0x%02x sensor\n", dev_addr);
	uint8_t data = mraa_i2c_read_byte_data(i2c_context, reg_addr);
	printf("read from 0x%02x 0x%02x <- 0x%02x\n", dev_addr, reg_addr, data);
	return data;
}

void tendof_init(void) {
	i2c_context = mraa_i2c_init(I2C_BUS);
	mraa_i2c_frequency(i2c_context, MRAA_I2C_STD);
	usleep(100 * 1000);
	tendof_i2c_write_byte_data(i2c_context, L3G4200D_ADDR, L3G_CTRL_REG1, 0x0F);
	tendof_i2c_write_byte_data(i2c_context, ADXL345_ADDR, 0x2D, 0x08);
	tendof_i2c_write_byte_data(i2c_context, HMC5883L_ADDR, 0x02, 0x00);
}

void tendof_release(void) {
	mraa_i2c_stop(i2c_context);
}

void tendof_run(void) {
	// Gyroscope
	if (mraa_i2c_address(i2c_context, L3G4200D_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", L3G4200D_ADDR);
	int16_t gyr_x, gyr_y, gyr_z;
	uint8_t gyr_data[6];
	int count = mraa_i2c_read_bytes_data(i2c_context, L3G_OUT_X_L, gyr_data, 6);
	gyr_x = gyr_data[1] << 8 |
			gyr_data[0];
	gyr_y = gyr_data[3] << 8 |
			gyr_data[2];
	gyr_z = gyr_data[5] << 8 |
			gyr_data[4];

//	printf("read count %d\n", count);
//	printf("%d %d %d\n", gyr_x, gyr_y, gyr_z);

	// Accelerometer
	int16_t acc_x, acc_y, acc_z;
	acc_x = tendof_i2c_read_byte_data(i2c_context, ADXL345_ADDR, 0x43) << 8 |
			tendof_i2c_read_byte_data(i2c_context, ADXL345_ADDR, 0x44);
	acc_y = tendof_i2c_read_byte_data(i2c_context, ADXL345_ADDR, 0x45) << 8 |
			tendof_i2c_read_byte_data(i2c_context, ADXL345_ADDR, 0x46);
	acc_z = tendof_i2c_read_byte_data(i2c_context, ADXL345_ADDR, 0x47) << 8 |
			tendof_i2c_read_byte_data(i2c_context, ADXL345_ADDR, 0x48);

//	printf("%d %d %d\n", acc_x, acc_y, acc_z);

	// Magnetometer
	int16_t mag_x, mag_y, mag_z;
	mag_x = tendof_i2c_read_byte_data(i2c_context, HMC5883L_ADDR, 0x03) |
			tendof_i2c_read_byte_data(i2c_context, HMC5883L_ADDR, 0x04) << 8;
	mag_y = tendof_i2c_read_byte_data(i2c_context, HMC5883L_ADDR, 0x05) |
			tendof_i2c_read_byte_data(i2c_context, HMC5883L_ADDR, 0x06) << 8;
	mag_z = tendof_i2c_read_byte_data(i2c_context, HMC5883L_ADDR, 0x07) |
			tendof_i2c_read_byte_data(i2c_context, HMC5883L_ADDR, 0x08) << 8;

//	printf("%d %d %d\n", mag_x, mag_y, mag_z);
}
