/*
 * MPU9250.c
 *
 *  Created on: Dec 12, 2015
 *      Author: terry
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "mraa.h"

#include "MPU9250.h"

#define I2C_BUS 		0x06

mraa_i2c_context i2c_context;

void mpu_init(void) {
	i2c_context = mraa_i2c_init(I2C_BUS);
	mraa_i2c_frequency(i2c_context, MRAA_I2C_FAST);
}

void mpu_release(void) {
	mraa_i2c_stop(i2c_context);
}

#define MPU9250_ADDR 	0x68
#define AK8963_ADDR 	0x0C

void mpu_run(void) {
	if (mraa_i2c_address(i2c_context, MPU9250_ADDR) != MRAA_SUCCESS)
		printf("can not found MPU9250 sensor\n");

	int16_t acc_x, acc_y, acc_z;
	acc_x = mraa_i2c_read_byte_data(i2c_context, 0x3B) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x3C);
	acc_y = mraa_i2c_read_byte_data(i2c_context, 0x3D) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x3E);
	acc_z = mraa_i2c_read_byte_data(i2c_context, 0x3F) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x40);

//	printf("%d %d %d\n", acc_x, acc_y, acc_z);

	int16_t gyr_x, gyr_y, gyr_z;
	gyr_x = mraa_i2c_read_byte_data(i2c_context, 0x43) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x44);
	gyr_y = mraa_i2c_read_byte_data(i2c_context, 0x45) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x46);
	gyr_z = mraa_i2c_read_byte_data(i2c_context, 0x47) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x48);

//	printf("%d %d %d\n", gyr_x, gyr_y, gyr_z);

	if (mraa_i2c_address(i2c_context, AK8963_ADDR) != MRAA_SUCCESS)
		printf("can not found AK8963 sensor\n");

	int16_t mag_x, mag_y, mag_z;
	mag_x = mraa_i2c_read_byte_data(i2c_context, 0x03) |
			mraa_i2c_read_byte_data(i2c_context, 0x04) << 8;
	mag_y = mraa_i2c_read_byte_data(i2c_context, 0x05) |
			mraa_i2c_read_byte_data(i2c_context, 0x06) << 8;
	mag_z = mraa_i2c_read_byte_data(i2c_context, 0x07) |
			mraa_i2c_read_byte_data(i2c_context, 0x08) << 8;

//	printf("%d %d %d\n", mag_x, mag_y, mag_z);


	/* ===================== */
	float accfx, accfy, accfz;
	accfx = -acc_y / 16256.0;
	accfy = acc_z / 16256.0;
	accfz = acc_x / 16256.0;

//	printf("%.2f %.2f %.2f\n", accfx, accfy, accfz);

	float anglex, angley, anglez;
	anglex = atan2(accfz, accfy) * 180 / M_PI - 180.0;
	anglex = -anglex;
	while (anglex < 0) anglex += 360;
	while (anglex > 360) anglex -= 360;
	angley = 0.0;
	anglez = atan2(accfy, accfx) * 180 / M_PI + 90.0;
	anglez = -anglez;
	while (anglez < 0) anglez += 360;
	while (anglez > 360) anglez -= 360;

//	printf("%.2f %.2f %.2f\n", anglex, angley, anglez);

	unsigned char msg[12];
	int c_i = 0;
	unsigned char *pdata;
	int i;

	float angle_x = anglex;
	pdata = ((unsigned char *)&angle_x);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	float angle_y = angley;
	pdata = ((unsigned char *)&angle_y);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	float angle_z = anglez;
	pdata = ((unsigned char *)&angle_z);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	tcpserver_send(msg, 12);
}

/*
#define L3G4200D_ADDR 	0x68
#define ADXL345_ADDR 	0x0C
#define HMC5883L_ADDR 	0x0C
#define BMP085_ADDR 	0x0C

void mpu_run(void) {
	if (mraa_i2c_address(i2c_context, L3G4200D_ADDR) != MRAA_SUCCESS)
		printf("can not found L3G4200D sensor\n");

	int16_t acc_x, acc_y, acc_z;
	acc_x = mraa_i2c_read_byte_data(i2c_context, 0x3B) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x3C);
	acc_y = mraa_i2c_read_byte_data(i2c_context, 0x3D) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x3E);
	acc_z = mraa_i2c_read_byte_data(i2c_context, 0x3F) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x40);

//	printf("%d %d %d\n", acc_x, acc_y, acc_z);

	if (mraa_i2c_address(i2c_context, ADXL345_ADDR) != MRAA_SUCCESS)
		printf("can not found ADXL345 sensor\n");

	int16_t gyr_x, gyr_y, gyr_z;
	gyr_x = mraa_i2c_read_byte_data(i2c_context, 0x43) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x44);
	gyr_y = mraa_i2c_read_byte_data(i2c_context, 0x45) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x46);
	gyr_z = mraa_i2c_read_byte_data(i2c_context, 0x47) << 8 |
			mraa_i2c_read_byte_data(i2c_context, 0x48);

//	printf("%d %d %d\n", gyr_x, gyr_y, gyr_z);

	if (mraa_i2c_address(i2c_context, HMC5883L_ADDR) != MRAA_SUCCESS)
		printf("can not found HMC5883L sensor\n");

	int16_t mag_x, mag_y, mag_z;
	mag_x = mraa_i2c_read_byte_data(i2c_context, 0x03) |
			mraa_i2c_read_byte_data(i2c_context, 0x04) << 8;
	mag_y = mraa_i2c_read_byte_data(i2c_context, 0x05) |
			mraa_i2c_read_byte_data(i2c_context, 0x06) << 8;
	mag_z = mraa_i2c_read_byte_data(i2c_context, 0x07) |
			mraa_i2c_read_byte_data(i2c_context, 0x08) << 8;

//	printf("%d %d %d\n", mag_x, mag_y, mag_z);
}
*/
