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
#include "I2CBus.h"
#include "TCPServer.h"

SensorData sensor_data;

//#define FIND_MAG_RANGE
#ifdef FIND_MAG_RANGE
int16_t x_min = 0x7FF8;
int16_t x_max = 0x7FF8;
int16_t y_min = 0x7FF8;
int16_t y_max = 0x7FF8;
int16_t z_min = 0x7FF8;
int16_t z_max = 0x7FF8;
int16_t x_mid = 0;
int16_t y_mid = 0;
int16_t z_mid = 0;
#endif

mraa_result_t mpu_i2c_write_byte_data(mraa_i2c_context i2c_context, const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t data) {
	if (mraa_i2c_address(i2c_context, dev_addr) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", dev_addr);
	mraa_result_t ret = mraa_i2c_write_byte_data(i2c_context, data, reg_addr);
	printf("write to 0x%02x 0x%02x -> 0x%02x %s\n", dev_addr, reg_addr, data, (ret == MRAA_SUCCESS) ? "SUCCESS" : "FAIL");
	return ret;
}

uint8_t mpu_i2c_read_byte_data(mraa_i2c_context i2c_context, const uint8_t dev_addr, const uint8_t reg_addr) {
	if (mraa_i2c_address(i2c_context, dev_addr) != MRAA_SUCCESS)
			printf("can not found 0x%02x sensor\n", dev_addr);
	uint8_t data = mraa_i2c_read_byte_data(i2c_context, reg_addr);
//	printf("read from 0x%02x 0x%02x <- 0x%02x\n", dev_addr, reg_addr, data);
	return data;
}

void mpu_set_bypass(mraa_i2c_context i2c_context, uint8_t status) {
	printf("%s\n", status ? "into bypass mode" : "into master mode");
    uint8_t pincfg;
    uint8_t usrctl;
    pincfg = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG);
    usrctl = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_USER_CTRL);
    if (status) {
        usrctl &= 0xDF;
        pincfg |= 0x02;
    } else {
        usrctl |= 0x20;
        pincfg &= 0xFD;
    }
    mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_USER_CTRL, usrctl);
    mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, pincfg);
}

void mpu_init(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL) {
		perror("Err : i2c_context = NULL");
		return;
	}
	// Init MPU9250
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_1, 0x80);// H_RESET = 1
	usleep(1 * 1000);
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_1, 0x01);// CLKSEL = 1
	usleep(1 * 1000);
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_2, 0x00);// Enable Acc & Gyro
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_CONFIG, 0x07);
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG, MPU_GyrFS_2000dps);// +-2000dps
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_ACCEL_CONFIG, MPU_AccFS_2g);// +-2G
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_ACCEL_CONFIG_2, 0x00);// Set Acc Data Rates
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, 0x30);// LATCH_INT_EN = 1, INT_ANYRD_2CLEAR = 1
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_I2C_MST_CTRL, 0x0D);// I2C Speed 400 kHz

//	mpu_set_bypass(i2c_context, 1);// Into bypass mode
//	mpu_i2c_write_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_CNTL1, 0x16);
//	uint8_t x_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAX);
//	uint8_t y_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAY);
//	uint8_t z_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAZ);
//	sensor_data.magnet.x_gain = ((x_asa - 128) * 0.5 / 128.f + 1);
//	sensor_data.magnet.y_gain = ((y_asa - 128) * 0.5 / 128.f + 1);
//	sensor_data.magnet.z_gain = ((z_asa - 128) * 0.5 / 128.f + 1);
//	printf("asa xyz : %f %f %f\n", sensor_data.magnet.x_gain, sensor_data.magnet.y_gain, sensor_data.magnet.z_gain);

	mpu_set_bypass(i2c_context, 0);// Into master mode
	// Set Slave to Read AK8963
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_ADDR, 0x80 | AK8963_I2C_ADDR);// AK8963_I2C_ADDR (Read | 0x0C)
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_REG, AK8963_ST1);
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_CTRL, 0x89);// Enable 9 bytes read 10001001
//	sensor_data.magnet.x_offset = 196;
//	sensor_data.magnet.y_offset = 320;
//	sensor_data.magnet.z_offset = -190;
	sensor_data.magnet.x_offset = 9;
	sensor_data.magnet.y_offset = 320;
	sensor_data.magnet.z_offset = -73;

	sensor_data.gyro.x_integral = 0.0;
	sensor_data.gyro.y_integral = 0.0;
	sensor_data.gyro.z_integral = 0.0;
}

void mpu_release(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL) {
		perror("Err : i2c_context = NULL");
		return;
	}
}

void mpu_run(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL) {
		perror("Err : i2c_context = NULL");
		return;
	}
	if (mraa_i2c_address(i2c_context, MPU9250_I2C_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MPU9250_I2C_ADDR);

	// Accelerometer
	sensor_data.accel.x_raw = (int16_t)((uint16_t)
			mraa_i2c_read_byte_data(i2c_context, MPU9250_ACCEL_XOUT_H) << 8 |
			mraa_i2c_read_byte_data(i2c_context, MPU9250_ACCEL_XOUT_L));
	sensor_data.accel.y_raw = (int16_t)((uint16_t)
			mraa_i2c_read_byte_data(i2c_context, MPU9250_ACCEL_YOUT_H) << 8 |
			mraa_i2c_read_byte_data(i2c_context, MPU9250_ACCEL_YOUT_L));
	sensor_data.accel.z_raw = (int16_t)((uint16_t)
			mraa_i2c_read_byte_data(i2c_context, MPU9250_ACCEL_ZOUT_H) << 8 |
			mraa_i2c_read_byte_data(i2c_context, MPU9250_ACCEL_ZOUT_L));
	sensor_data.accel.x = +(float)sensor_data.accel.x_raw * MPU9250A_2g;
	sensor_data.accel.y = +(float)sensor_data.accel.z_raw * MPU9250A_2g;
	sensor_data.accel.z = -(float)sensor_data.accel.y_raw * MPU9250A_2g;

	// Temperature
	sensor_data.temp_raw = (int16_t)((uint16_t)
			mraa_i2c_read_byte_data(i2c_context, MPU9250_TEMP_OUT_H) << 8 |
			mraa_i2c_read_byte_data(i2c_context, MPU9250_TEMP_OUT_L));
	sensor_data.temp = (float)sensor_data.temp_raw * MPU9250T_85degC + 21.0f;

	// Gyroscope
	sensor_data.gyro.x_raw = (int16_t)((uint16_t)
			mraa_i2c_read_byte_data(i2c_context, MPU9250_GYRO_XOUT_H) << 8 |
			mraa_i2c_read_byte_data(i2c_context, MPU9250_GYRO_XOUT_L));
	sensor_data.gyro.y_raw = (int16_t)((uint16_t)
			mraa_i2c_read_byte_data(i2c_context, MPU9250_GYRO_YOUT_H) << 8 |
			mraa_i2c_read_byte_data(i2c_context, MPU9250_GYRO_YOUT_L));
	sensor_data.gyro.z_raw = (int16_t)((uint16_t)
			mraa_i2c_read_byte_data(i2c_context, MPU9250_GYRO_ZOUT_H) << 8 |
			mraa_i2c_read_byte_data(i2c_context, MPU9250_GYRO_ZOUT_L));
	sensor_data.gyro.x = +(float)sensor_data.gyro.x_raw * MPU9250G_2000dps * M_PI / 180.f;
	sensor_data.gyro.y = +(float)sensor_data.gyro.z_raw * MPU9250G_2000dps * M_PI / 180.f;
	sensor_data.gyro.z = -(float)sensor_data.gyro.y_raw * MPU9250G_2000dps * M_PI / 180.f;
	sensor_data.gyro.x_integral += sensor_data.gyro.x / 50;
	sensor_data.gyro.y_integral += sensor_data.gyro.y / 50;
	sensor_data.gyro.z_integral += sensor_data.gyro.z / 50;

	// Magnetometer
	uint8_t XL = mraa_i2c_read_byte_data(i2c_context, MPU9250_EXT_SENS_DATA_01);
	uint8_t XH = mraa_i2c_read_byte_data(i2c_context, MPU9250_EXT_SENS_DATA_02);
	uint8_t YL = mraa_i2c_read_byte_data(i2c_context, MPU9250_EXT_SENS_DATA_03);
	uint8_t YH = mraa_i2c_read_byte_data(i2c_context, MPU9250_EXT_SENS_DATA_04);
	uint8_t ZL = mraa_i2c_read_byte_data(i2c_context, MPU9250_EXT_SENS_DATA_05);
	uint8_t ZH = mraa_i2c_read_byte_data(i2c_context, MPU9250_EXT_SENS_DATA_06);
	sensor_data.magnet.x_raw = (int16_t)(
			XL | ((uint16_t)
			XH << 8));
	sensor_data.magnet.y_raw = (int16_t)(
			YL | ((uint16_t)
			YH << 8));
	sensor_data.magnet.z_raw = (int16_t)(
			ZL | ((uint16_t)
			ZH << 8));
#ifdef FIND_MAG_RANGE
	if (x_min == 0x7FF8) x_min = sensor_data.magnet.x_raw;
	if (x_max == 0x7FF8) x_max = sensor_data.magnet.x_raw;
	if (y_min == 0x7FF8) y_min = sensor_data.magnet.y_raw;
	if (y_max == 0x7FF8) y_max = sensor_data.magnet.y_raw;
	if (z_min == 0x7FF8) z_min = sensor_data.magnet.z_raw;
	if (z_max == 0x7FF8) z_max = sensor_data.magnet.z_raw;
	if (x_min > sensor_data.magnet.x_raw) x_min = sensor_data.magnet.x_raw;
	if (x_max < sensor_data.magnet.x_raw) x_max = sensor_data.magnet.x_raw;
	if (y_min > sensor_data.magnet.y_raw) y_min = sensor_data.magnet.y_raw;
	if (y_max < sensor_data.magnet.y_raw) y_max = sensor_data.magnet.y_raw;
	if (z_min > sensor_data.magnet.z_raw) z_min = sensor_data.magnet.z_raw;
	if (z_max < sensor_data.magnet.z_raw) z_max = sensor_data.magnet.z_raw;
	printf("%d %d %d %d %d %d ---- ", x_min, x_max, y_min, y_max, z_min, z_max);
	x_mid = (x_min + x_max) / 2;
	y_mid = (y_min + y_max) / 2;
	z_mid = (z_min + z_max) / 2;
	printf("%d %d %d ---- ", x_mid, y_mid, z_mid);
#endif
	sensor_data.magnet.x = +(float)(sensor_data.magnet.y_raw - sensor_data.magnet.y_offset) * sensor_data.magnet.y_gain * MPU9250M_4800uT;
	sensor_data.magnet.y = -(float)(sensor_data.magnet.z_raw - sensor_data.magnet.z_offset) * sensor_data.magnet.z_gain * MPU9250M_4800uT;
	sensor_data.magnet.z = -(float)(sensor_data.magnet.x_raw - sensor_data.magnet.x_offset) * sensor_data.magnet.x_gain * MPU9250M_4800uT;

//	printf("tempr : %f degC\n",
//			sensor_data.temp);
//	printf("accel : %8.2f %8.2f %8.2f\n",
//			sensor_data.accel.x,
//			sensor_data.accel.y,
//			sensor_data.accel.z);
//	printf("gyros : %8.2f %8.2f %8.2f\n",
//			sensor_data.gyro.x,
//			sensor_data.gyro.y,
//			sensor_data.gyro.z);
//	printf("magen : %8.2f %8.2f %8.2f\n",
//			sensor_data.magnet.x,
//			sensor_data.magnet.y,
//			sensor_data.magnet.z);

//	float accfx, accfy, accfz;
//	accfx = sensor_data.magnet.x;
//	accfy = sensor_data.magnet.y;
//	accfz = sensor_data.magnet.z;

	float anglex, angley, anglez;
	anglex = atan2(sensor_data.accel.z, sensor_data.accel.y) * 180 / M_PI;
	while (anglex < 0) anglex += 360;
	while (anglex > 360) anglex -= 360;
//	angley = atan2(sensor_data.magnet.x, sensor_data.magnet.z) * 180 / M_PI - 180;
	angley = -sensor_data.gyro.y_integral * 180 / M_PI;
	while (angley < 0) angley += 360;
	while (angley > 360) angley -= 360;
	anglez = atan2(sensor_data.accel.y, sensor_data.accel.x) * 180 / M_PI - 90;
	while (anglez < 0) anglez += 360;
	while (anglez > 360) anglez -= 360;

	unsigned char msg[12];
	int c_i = 0;
	unsigned char *pdata;
	int i;

	float angle_x = -anglex;
	pdata = ((unsigned char *)&angle_x);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	float angle_y = -angley;
	pdata = ((unsigned char *)&angle_y);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	float angle_z = -anglez;
	pdata = ((unsigned char *)&angle_z);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	tcpserver_send(msg, 12);
}
