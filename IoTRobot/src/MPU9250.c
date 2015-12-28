/*
 * MPU9250.c
 *
 *  Created on: Dec 12, 2015
 *      Author: terry
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include "mraa.h"

#include "MPU9250.h"
#include "I2CBus.h"
#include "TCPServer.h"

//#define FIND_MAG_RANGE
#ifdef FIND_MAG_RANGE
int16_t x_min = 0x7FF8;
int16_t x_max = 0x7FF8;
int16_t y_min = 0x7FF8;
int16_t y_max = 0x7FF8;
int16_t z_min = 0x7FF8;
int16_t z_max = 0x7FF8;
#endif

SensorData sensor_data;
Quaternion sensor_quaternion = {0, 0, 1, 0};
struct timeval start, stop, diff;
double diff_sec;
int delay_us = 1;

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

int timeval_subtract(struct timeval* result, struct timeval* x, struct timeval* y) {
	if (x->tv_sec > y->tv_sec)
		return -1;
	if ((x->tv_sec == y->tv_sec) && (x->tv_usec >= y->tv_usec))
		return -1;
	result->tv_sec = (y->tv_sec - x->tv_sec);
	result->tv_usec = (y->tv_usec - x->tv_usec);
	if (result->tv_usec < 0) {
		result->tv_sec--;
		result->tv_usec += 1000000;
	}
	return 0;
}

void get_diff_time() {
	gettimeofday(&stop, 0);
	if (timeval_subtract(&diff, &start, &stop) == -1 || diff.tv_sec > 0) {
		diff.tv_sec = 0;
		diff.tv_usec = 1;
	}
	gettimeofday(&start, 0);
//	printf("%.2f\n", diff.tv_usec / 1000.f);
	diff_sec = diff.tv_usec / 1000000.0f;
}

void delay_for_ms(int ms) {
	delay_us += (ms * 1000 - diff.tv_usec) / 2;
//	printf("%d ", delay_us);
	if (delay_us > 0)
		usleep(delay_us);
	else
		printf("%.3f > %d ms!\n", diff.tv_usec / 1000.f, ms);
}

void quaternion_to_euler(Quaternion qua, EulerAngles *euler) {
	euler->x = atan2(2 * qua.w * qua.x + 2 * qua.y * qua.z,
			1 - 2 * qua.x * qua.x - 2 * qua.y * qua.y);
	euler->y = asin(2 * qua.w * qua.y - 2 * qua.z * qua.x);
	euler->z = atan2(2 * qua.w * qua.z + 2 * qua.x * qua.y,
			1 - 2 * qua.y * qua.y - 2 * qua.z * qua.z);
}

void sync_posture() {
	unsigned char msg[16];
	int c_i = 0;
	unsigned char *pdata;
	int i;

	float a = 2 * acos(sensor_quaternion.w);
	a = a * 180 / M_PI;
	while (a < 0) a += 360;
	while (a >= 360) a -= 360;
	pdata = ((unsigned char *)&a);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	float x = 0;
	if (a != 0) x = sensor_quaternion.x / sin(acos(sensor_quaternion.w));
	pdata = ((unsigned char *)&x);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	float y = 0;
	if (a != 0) y = sensor_quaternion.y / sin(acos(sensor_quaternion.w));
	pdata = ((unsigned char *)&y);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	float z = 0;
	if (a != 0) z = sensor_quaternion.z / sin(acos(sensor_quaternion.w));
	pdata = ((unsigned char *)&z);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	tcpserver_send(msg, 16);
}

void ak_init(mraa_i2c_context i2c_context) {
	mpu_i2c_write_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_CNTL1, 0x16);
	uint8_t x_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAX);
	uint8_t y_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAY);
	uint8_t z_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAZ);
	sensor_data.magnet.x_gain = ((x_asa - 128) * 0.5 / 128.f + 1);
	sensor_data.magnet.y_gain = ((y_asa - 128) * 0.5 / 128.f + 1);
	sensor_data.magnet.z_gain = ((z_asa - 128) * 0.5 / 128.f + 1);
	printf("asa xyz : %f %f %f\n", sensor_data.magnet.x_gain, sensor_data.magnet.y_gain, sensor_data.magnet.z_gain);

	sensor_data.magnet.x_offset = 100;
	sensor_data.magnet.y_offset = 320;
	sensor_data.magnet.z_offset = -120;
}

void ak_release(mraa_i2c_context i2c_context) {
	mpu_i2c_write_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_CNTL1, 0x10);
}

void euler_to_quaternion(EulerAngles euler, Quaternion *qua) {
	qua->w = cos(euler.x / 2) * cos(euler.y / 2) * cos(euler.z / 2) +
			sin(euler.x / 2) * sin(euler.y / 2) * sin(euler.z / 2);
	qua->x = sin(euler.x / 2) * cos(euler.y / 2) * cos(euler.z / 2) -
			cos(euler.x / 2) * sin(euler.y / 2) * sin(euler.z / 2);
	qua->y = cos(euler.x / 2) * sin(euler.y / 2) * cos(euler.z / 2) +
			sin(euler.x / 2) * cos(euler.y / 2) * sin(euler.z / 2);
	qua->z = cos(euler.x / 2) * cos(euler.y / 2) * sin(euler.z / 2) -
			sin(euler.x / 2) * sin(euler.y / 2) * cos(euler.z / 2);
}

Quaternion quaternion_multiply(Quaternion a_qua, Quaternion b_qua) {
	Quaternion r_qua;
	r_qua.w = a_qua.w * b_qua.w - a_qua.x * b_qua.x - a_qua.y * b_qua.y - a_qua.z * b_qua.z;
	r_qua.x = a_qua.w * b_qua.x + a_qua.x * b_qua.w + a_qua.y * b_qua.z - a_qua.z * b_qua.y;
	r_qua.y = a_qua.w * b_qua.y - a_qua.x * b_qua.z + a_qua.y * b_qua.w + a_qua.z * b_qua.x;
	r_qua.z = a_qua.w * b_qua.z + a_qua.x * b_qua.y - a_qua.y * b_qua.x + a_qua.z * b_qua.w;
	return r_qua;
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

	mpu_set_bypass(i2c_context, 1);// Into bypass mode
	ak_init(i2c_context);

	mpu_set_bypass(i2c_context, 0);// Into master mode
	// Set Slave to Read AK8963
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_ADDR, 0x80 | AK8963_I2C_ADDR);// AK8963_I2C_ADDR (Read | 0x0C)
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_REG, AK8963_ST1);
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_I2C_SLV0_CTRL, 0x89);// Enable 9 bytes read 10001001

	gettimeofday(&start, 0);
}

void mpu_release(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL) {
		perror("Err : i2c_context = NULL");
		return;
	}
	mpu_set_bypass(i2c_context, 1);// Into bypass mode
	ak_release(i2c_context);
	mpu_set_bypass(i2c_context, 0);// Into master mode
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
	get_diff_time();
	sensor_data.gyro.euler_angles.x = +(float)sensor_data.gyro.x_raw * MPU9250G_2000dps * M_PI / 180.f * diff_sec;
	sensor_data.gyro.euler_angles.y = +(float)sensor_data.gyro.z_raw * MPU9250G_2000dps * M_PI / 180.f * diff_sec;
	sensor_data.gyro.euler_angles.z = -(float)sensor_data.gyro.y_raw * MPU9250G_2000dps * M_PI / 180.f * diff_sec;
	euler_to_quaternion(sensor_data.gyro.euler_angles, &sensor_data.gyro.quaternion);

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
	sensor_data.magnet.x_offset = (x_min + x_max) / 2;
	sensor_data.magnet.y_offset = (y_min + y_max) / 2;
	sensor_data.magnet.z_offset = (z_min + z_max) / 2;
	printf("%d %d %d ---- ", sensor_data.magnet.x_offset, sensor_data.magnet.y_offset, sensor_data.magnet.z_offset);
#endif
	sensor_data.magnet.x = +(float)(sensor_data.magnet.y_raw - sensor_data.magnet.y_offset) * sensor_data.magnet.y_gain * MPU9250M_4800uT;
	sensor_data.magnet.y = -(float)(sensor_data.magnet.z_raw - sensor_data.magnet.z_offset) * sensor_data.magnet.z_gain * MPU9250M_4800uT;
	sensor_data.magnet.z = -(float)(sensor_data.magnet.x_raw - sensor_data.magnet.x_offset) * sensor_data.magnet.x_gain * MPU9250M_4800uT;

//	printf("tempr : %f degC\n",
//			sensor_data.temp);
//	printf("accel : %8.2f %8.2f %8.2f g\n",
//			sensor_data.accel.x,
//			sensor_data.accel.y,
//			sensor_data.accel.z);
//	printf("gyros : %8.2f %8.2f %8.2f rad/s\n",
//			sensor_data.gyro.x,
//			sensor_data.gyro.y,
//			sensor_data.gyro.z);
//	printf("magen : %8.2f %8.2f %8.2f uT\n",
//			sensor_data.magnet.x,
//			sensor_data.magnet.y,
//			sensor_data.magnet.z);

	sensor_quaternion = quaternion_multiply(sensor_quaternion, sensor_data.gyro.quaternion);

	sync_posture();

	delay_for_ms(10);
}
