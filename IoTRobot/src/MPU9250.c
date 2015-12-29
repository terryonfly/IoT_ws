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
#include "Posture.h"

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
SensorDataRaw sensor_data_raw;

struct timeval start, stop, diff;
float delay_us = 1;

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

double get_diff_time() {
	gettimeofday(&stop, 0);
	if (timeval_subtract(&diff, &start, &stop) == -1 || diff.tv_sec > 0) {
		diff.tv_sec = 0;
		diff.tv_usec = 1;
	}
	gettimeofday(&start, 0);
//	printf("%.2f\n", diff.tv_usec / 1000.f);
	return diff.tv_usec / 1000000.0f;
}

void delay_for_ms(int ms) {
	delay_us += (ms * 1000 - diff.tv_usec) / 2.f;
	if (delay_us > 0) {
//		printf("%.3f %.3f ms\n", delay_us / 1000.f, diff.tv_usec / 1000.f);
		usleep((int)delay_us);
	} else {
		printf("%.3f > %d ms!\n", diff.tv_usec / 1000.f, ms);
		delay_us = 0;
	}
}

void ak_init(mraa_i2c_context i2c_context) {
	mpu_i2c_write_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_CNTL1, 0x16);
	uint8_t x_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAX);
	uint8_t y_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAY);
	uint8_t z_asa = mpu_i2c_read_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_ASAZ);
	sensor_data_raw.magnet_gain.x = ((x_asa - 128) * 0.5 / 128.f + 1);
	sensor_data_raw.magnet_gain.y = ((y_asa - 128) * 0.5 / 128.f + 1);
	sensor_data_raw.magnet_gain.z = ((z_asa - 128) * 0.5 / 128.f + 1);
	printf("asa xyz : %f %f %f\n", sensor_data_raw.magnet_gain.x, sensor_data_raw.magnet_gain.y, sensor_data_raw.magnet_gain.z);

//	sensor_data_raw.magnet_offset.x = 100;
//	sensor_data_raw.magnet_offset.y = 320;
//	sensor_data_raw.magnet_offset.z = -70;
}

void ak_release(mraa_i2c_context i2c_context) {
	mpu_i2c_write_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_CNTL1, 0x10);
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

	printf("MPU9250 init finished!\n");
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

	uint8_t bulk_data[23];
	int ret = mraa_i2c_read_bytes_data(i2c_context, MPU9250_ACCEL_XOUT_H, bulk_data, 23);
	if (ret != 23) {
		printf("err read bulk len = %d\n", ret);
		return;
	}

	// Difference Time
	sensor_data.diff_sec = get_diff_time();

	// Accelerometer
	sensor_data_raw.accel_raw.x = (int16_t)((uint16_t)
			bulk_data[0] << 8 |
			bulk_data[1]);
	sensor_data_raw.accel_raw.y = (int16_t)((uint16_t)
			bulk_data[2] << 8 |
			bulk_data[3]);
	sensor_data_raw.accel_raw.z = (int16_t)((uint16_t)
			bulk_data[4] << 8 |
			bulk_data[5]);
	sensor_data.accel.x = +(float)sensor_data_raw.accel_raw.x * MPU9250A_2g;
	sensor_data.accel.y = +(float)sensor_data_raw.accel_raw.z * MPU9250A_2g;
	sensor_data.accel.z = -(float)sensor_data_raw.accel_raw.y * MPU9250A_2g;

	// Temperature
	sensor_data_raw.temp_raw = (int16_t)((uint16_t)
			bulk_data[6] << 8 |
			bulk_data[7]);
//	sensor_data.temp = (float)sensor_data_raw.temp_raw * MPU9250T_85degC + 21.0f;

	// Gyroscope
	sensor_data_raw.gyro_raw.x = (int16_t)((uint16_t)
			bulk_data[8] << 8 |
			bulk_data[9]);
	sensor_data_raw.gyro_raw.y = (int16_t)((uint16_t)
			bulk_data[10] << 8 |
			bulk_data[11]);
	sensor_data_raw.gyro_raw.z = (int16_t)((uint16_t)
			bulk_data[12] << 8 |
			bulk_data[13]);
	sensor_data.gyro.x = +(float)sensor_data_raw.gyro_raw.x * MPU9250G_2000dps * M_PI / 180.f;
	sensor_data.gyro.y = +(float)sensor_data_raw.gyro_raw.z * MPU9250G_2000dps * M_PI / 180.f;
	sensor_data.gyro.z = -(float)sensor_data_raw.gyro_raw.y * MPU9250G_2000dps * M_PI / 180.f;

	// Magnetometer
	uint8_t XL = bulk_data[15];
	uint8_t XH = bulk_data[16];
	uint8_t YL = bulk_data[17];
	uint8_t YH = bulk_data[18];
	uint8_t ZL = bulk_data[19];
	uint8_t ZH = bulk_data[20];
	sensor_data_raw.magnet_raw.x = (int16_t)(
			XL | ((uint16_t)
			XH << 8));
	sensor_data_raw.magnet_raw.y = (int16_t)(
			YL | ((uint16_t)
			YH << 8));
	sensor_data_raw.magnet_raw.z = (int16_t)(
			ZL | ((uint16_t)
			ZH << 8));
#ifdef FIND_MAG_RANGE
	if (x_min == 0x7FF8) x_min = sensor_data_raw.magnet_raw.x;
	if (x_max == 0x7FF8) x_max = sensor_data_raw.magnet_raw.x;
	if (y_min == 0x7FF8) y_min = sensor_data_raw.magnet_raw.y;
	if (y_max == 0x7FF8) y_max = sensor_data_raw.magnet_raw.y;
	if (z_min == 0x7FF8) z_min = sensor_data_raw.magnet_raw.z;
	if (z_max == 0x7FF8) z_max = sensor_data_raw.magnet_raw.z;
	if (x_min > sensor_data_raw.magnet_raw.x) x_min = sensor_data_raw.magnet_raw.x;
	if (x_max < sensor_data_raw.magnet_raw.x) x_max = sensor_data_raw.magnet_raw.x;
	if (y_min > sensor_data_raw.magnet_raw.y) y_min = sensor_data_raw.magnet_raw.y;
	if (y_max < sensor_data_raw.magnet_raw.y) y_max = sensor_data_raw.magnet_raw.y;
	if (z_min > sensor_data_raw.magnet_raw.z) z_min = sensor_data_raw.magnet_raw.z;
	if (z_max < sensor_data_raw.magnet_raw.z) z_max = sensor_data_raw.magnet_raw.z;
	printf("%d %d %d %d %d %d ---- ", x_min, x_max, y_min, y_max, z_min, z_max);
	sensor_data_raw.magnet_offset.x = (x_min + x_max) / 2;
	sensor_data_raw.magnet_offset.y = (y_min + y_max) / 2;
	sensor_data_raw.magnet_offset.z = (z_min + z_max) / 2;
	printf("%d %d %d\n",
			sensor_data_raw.magnet_offset.x,
			sensor_data_raw.magnet_offset.y,
			sensor_data_raw.magnet_offset.z);
#endif
	sensor_data.magnet.x = +(float)(sensor_data_raw.magnet_raw.y - sensor_data_raw.magnet_offset.y) * sensor_data_raw.magnet_gain.y * MPU9250M_4800uT;
	sensor_data.magnet.y = -(float)(sensor_data_raw.magnet_raw.z - sensor_data_raw.magnet_offset.z) * sensor_data_raw.magnet_gain.z * MPU9250M_4800uT;
	sensor_data.magnet.z = -(float)(sensor_data_raw.magnet_raw.x - sensor_data_raw.magnet_offset.x) * sensor_data_raw.magnet_gain.x * MPU9250M_4800uT;

	/*
	printf("tempr : %f degC\n",
			sensor_data.temp);
	printf("accel : %8.2f %8.2f %8.2f g\n",
			sensor_data.accel.x,
			sensor_data.accel.y,
			sensor_data.accel.z);
	printf("gyros : %8.2f %8.2f %8.2f rad/s\n",
			sensor_data.gyro.x,
			sensor_data.gyro.y,
			sensor_data.gyro.z);
	printf("magen : %8.2f %8.2f %8.2f uT\n",
			sensor_data.magnet.x,
			sensor_data.magnet.y,
			sensor_data.magnet.z);
//	*/

	update_sensor_data(sensor_data);

	delay_for_ms(10);
}
