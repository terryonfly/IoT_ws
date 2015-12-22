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
#define BMP180_ADDR 		0x77

float SelfTest[12];
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};
float aRes, gRes, mRes;// scale resolutions per LSB for the sensors
uint8_t status;// MAX21100 data status register
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

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

void max_gyr_init(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);
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
}

void max_acc_init(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);
	// SNS_ACC_FSC = 2g
	// ACC_SELF_TEST = desabled
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b11000111, 0x04);// PWR_ACC_CFG
	// SNS_ACC_HPF_CFG = ODR / 50
	// SNS_ACC_LPF_CFG = ODR / 3
	// SNS_ACC_ODR = 2kHz
	mraa_i2c_write_byte_data(i2c_context, 0b11110000, 0x05);// ACC_CFG1
	// SNS_MAG_ODR = ACC_ODR / 16
	// SNS_ACC_LPF_CFG = disabled
	mraa_i2c_write_byte_data(i2c_context, 0b00001001, 0x06);// ACC_CFG2
}

void max_init_bypass(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);
	max_gyr_init(i2c_context);
	max_acc_init(i2c_context);
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
	max_gyr_init(i2c_context);
	max_acc_init(i2c_context);
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

void bmp_init(mraa_i2c_context i2c_context) {
	if (mraa_i2c_address(i2c_context, BMP180_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", BMP180_ADDR);
}

void max_selftest(mraa_i2c_context i2c_context, float *selfTest) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);

	int16_t gyroSelfTestplus[3] = {0, 0, 0}, gyroSelfTestminus[3] = {0, 0, 0};
	int16_t accelSelfTestplus[3] = {0, 0, 0}, accelSelfTestminus[3] = {0, 0, 0};
	uint8_t rawData[6];
	// accelero normal + gyro normal mode
	// SNS_EN_Z, SNS_EN_Y, SNS_EN_X = enabled
	mraa_i2c_write_byte_data(i2c_context, 0b01111111, 0x00);// POWER_CFG
	// SELF_TEST = positive deflection
	// SNS_LPF_CFG = 14Hz
	// SNS_DOUT_FSC = 250dps
	float gyrosensitivity  = 131.0f;   // = 131 LSB/degrees/sec per data sheet
	mraa_i2c_write_byte_data(i2c_context, 0b01010111, 0x01);
	usleep(100 * 1000);

	mraa_i2c_read_bytes_data(i2c_context, 0x24, (uint8_t *)rawData, 6);
	gyroSelfTestplus[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
	gyroSelfTestplus[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
	gyroSelfTestplus[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

	// SELF_TEST = negative deflection
	// SNS_LPF_CFG = 14Hz
	// SNS_DOUT_FSC = 250dps
	mraa_i2c_write_byte_data(i2c_context, 0b10010111, 0x01);// GYRO_CFG1
	usleep(100 * 1000);

	mraa_i2c_read_bytes_data(i2c_context, 0x24, (uint8_t *)rawData, 6);
	gyroSelfTestminus[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
	gyroSelfTestminus[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
	gyroSelfTestminus[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

	selfTest[0] = (float)gyroSelfTestplus[0] / gyrosensitivity;
	selfTest[1] = (float)gyroSelfTestplus[1] / gyrosensitivity;
	selfTest[2] = (float)gyroSelfTestplus[2] / gyrosensitivity;
	selfTest[3] = (float)gyroSelfTestminus[0] / gyrosensitivity;
	selfTest[4] = (float)gyroSelfTestminus[1] / gyrosensitivity;
	selfTest[5] = (float)gyroSelfTestminus[2] / gyrosensitivity;

	// SELF_TEST = desable selftest
	// SNS_LPF_CFG = 14Hz
	// SNS_DOUT_FSC = 250dps
	mraa_i2c_write_byte_data(i2c_context, 0b00010111, 0x01);// GYRO_CFG1 : disable to default

	// Configure the accelerometer
	// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
	float accelsensitivity = 16384.0f;  // = 16384 LSB/g per data sheet

	//  positive x-axis deflection
	mraa_i2c_write_byte_data(i2c_context, 0x03 << 6 | 0x08 | 0x07, 0x04); // pos x axis
	usleep(100 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, 0x2A, (uint8_t *)rawData, 2);// Read the raw data registers into data array
	accelSelfTestplus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	//  negative x-axis deflection
	mraa_i2c_write_byte_data(i2c_context, 0x03 << 6 | 0x28 | 0x07, 0x04); // neg x axis
	usleep(100 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, 0x2A, (uint8_t *)rawData, 2);// Read the raw data registers into data array
	accelSelfTestminus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	//  positive y-axis deflection
	mraa_i2c_write_byte_data(i2c_context, 0x03 << 6 | 0x10 | 0x07, 0x04); // pos y axis
	usleep(100 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, 0x2C, (uint8_t *)rawData, 2);// Read the raw data registers into data array
	accelSelfTestplus[1] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);   // Turn the MSB and LSB into a signed 16-bit value

	//  negative y-axis deflection
	mraa_i2c_write_byte_data(i2c_context, 0x03 << 6 | 0x30 | 0x07, 0x04); // neg y axis
	usleep(100 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, 0x2C, (uint8_t *)rawData, 2);// Read the raw data registers into data array
	accelSelfTestminus[1] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	//  positive z-axis deflection
	mraa_i2c_write_byte_data(i2c_context, 0x03 << 6 | 0x18 | 0x07, 0x04); // pos z axis
	usleep(100 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, 0x2E, (uint8_t *)rawData, 2);// Read the raw data registers into data array
	accelSelfTestplus[2] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	//  negative z-axis deflection
	mraa_i2c_write_byte_data(i2c_context, 0x03 << 6 | 0x38 | 0x07, 0x04); // neg z axis
	usleep(100 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, 0x2E, (uint8_t *)rawData, 2);// Read the raw data registers into data array
	accelSelfTestminus[2] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	selfTest[6] = (float)accelSelfTestplus[0] / accelsensitivity;
	selfTest[7] = (float)accelSelfTestplus[1] / accelsensitivity;
	selfTest[8] = (float)accelSelfTestplus[2] / accelsensitivity;
	selfTest[9] = (float)accelSelfTestminus[0] / accelsensitivity;
	selfTest[10] = (float)accelSelfTestminus[1] / accelsensitivity;
	selfTest[11] = (float)accelSelfTestminus[2] / accelsensitivity;

	// disable accel self test
	mraa_i2c_write_byte_data(i2c_context, 0x03 << 6 | 0x00 | 0x07, 0x04);
}

void max_accel_gyro_calibrate(mraa_i2c_context i2c_context, float *dest1, float *dest2) {
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);
	max_bank_select(i2c_context, 0);

	uint8_t data[12];// data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii = 0, fifo_count = 0, sample_count = 0;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// Choose normal power mode (accelnormalgyronormalmode = 0x0F in bits 6:3
	// Enable all axes (z = bit 2, y = bit 1, x = bit 0)
	mraa_i2c_write_byte_data(i2c_context, 0x0F << 3 | 0x07, 0x00);// POWER_CFG
	usleep(100 * 1000);

	// Configure gyro
	// Select 14 Hz gyro bandwidth (bits 5:2) and 250 dps gyro full scale (bits 1:0)
	mraa_i2c_write_byte_data(i2c_context, 0x05 << 2 | 0x03, 0x01);// GYRO_CFG1
	// Select 125 Hz gyro ODR (bits 1:0)
	mraa_i2c_write_byte_data(i2c_context, 0x06, 0x02);// GYRO_CFG2
	usleep(100 * 1000);

	// Configure the accelerometer
	// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
	mraa_i2c_write_byte_data(i2c_context, 0x03 << 6 | 0x07, 0x04);// PWR_ACC_CFG
	// Select 14 Hz accel band width (bits 5:4) and 125 Hz accel ODR (bits 3:0)
	mraa_i2c_write_byte_data(i2c_context, 0x02 << 4 | 0x04, 0x05);// ACC_CFG1
	usleep(100 * 1000);

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec per data sheet
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g per data sheet

	// 128 bytes in the FIFO = 64 words; use 63 words to collect 63/3 axes = 21 3-axis gyro data samples
	// Use FIFO to collect and average 21 gyro data samples
	mraa_i2c_write_byte_data(i2c_context,  0x3C, 0x17);// FIFO_TH // Set word threshold to 63 = 21/125 Hz = 168 ms of data
	mraa_i2c_write_byte_data(i2c_context, 0x41, 0x18);// FIFO_CFG // Set FIFO normal mode and accumulate gyro data
	usleep(200 * 1000);  // Wait to collect gyro data in the FIFO

	if(mraa_i2c_read_byte_data(i2c_context, MAX21100_FIFO_STATUS) & 0x04) {  // Verify FIFO threshold reached
		fifo_count = mraa_i2c_read_byte_data(i2c_context, MAX21100_FIFO_COUNT);  // get number of samples in the FIFO
		sample_count = fifo_count/3; // should be 21 for 63 word fifo count
		for(ii = 0; ii < sample_count; ii++) {
			int16_t gyro_temp[3] = {0, 0, 0};
			mraa_i2c_read_bytes_data(i2c_context, MAX21100_FIFO_DATA, (uint8_t *)data, 6);// Read the six FIFO registers per sample
			//    readBytes(MAX21100_ADDRESS, MAX21100_GYRO_X_H, 6, &data[0]);  // Read the six FIFO registers per sample
			gyro_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]) ; // Form signed 16-bit integer for each sample in FIFO
			gyro_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]) ;
			gyro_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]) ;

			gyro_bias[0]  += (int32_t) gyro_temp[0];
			gyro_bias[1]  += (int32_t) gyro_temp[1];
			gyro_bias[2]  += (int32_t) gyro_temp[2];
		}
	}
	gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias
	gyro_bias[1]  /= (int32_t) sample_count;
	gyro_bias[2]  /= (int32_t) sample_count;

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Now do the same for the accelerometer
	// 128 bytes in the FIFO = 64 words; use 63 words to collect 63/3 axes = 21 3-axis accel data samples
	// Use FIFO to collect and average 21 accel data samples
	mraa_i2c_write_byte_data(i2c_context, MAX21100_FIFO_TH,  0x3C); // Set word threshold to 63 = 21/125 Hz = 168 ms of data
	mraa_i2c_write_byte_data(i2c_context, MAX21100_FIFO_CFG, 0x42); // Set FIFO normal mode and accumulate accel data
	usleep(200 * 1000);  // Wait to collect accel data in the FIFO

	if(mraa_i2c_read_byte_data(i2c_context, MAX21100_FIFO_STATUS) & 0x04) {  // Verify FIFO threshold reached
		fifo_count = mraa_i2c_read_byte_data(i2c_context, MAX21100_FIFO_COUNT);  // get number of samples in the FIFO
		sample_count = fifo_count/3;
		for(ii = 0; ii < sample_count; ii++) {
			int16_t accel_temp[3] = {0, 0, 0};
			mraa_i2c_read_bytes_data(i2c_context, MAX21100_FIFO_DATA, (uint8_t *)data, 6);// Read the six FIFO registers per sample
			//    readBytes(MAX21100_ADDRESS, MAX21100_ACC_X_H, 6, &data[0]);  // Read the six FIFO registers per sample
			accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]) ; // Form signed 16-bit integer for each sample in FIFO
			accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]) ;
			accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]) ;

			accel_bias[0]  += (int32_t) accel_temp[0];
			accel_bias[1]  += (int32_t) accel_temp[1];
			accel_bias[2]  += (int32_t) accel_temp[2];
		}
	}
	accel_bias[0] /= (int32_t) sample_count;  // get average gyro bias
	accel_bias[1] /= (int32_t) sample_count;
	accel_bias[2] /= (int32_t) sample_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Output scaled gyro biases for display in the main program
	dest2[0] = (float) accel_bias[0]/(float) accelsensitivity;
	dest2[1] = (float) accel_bias[1]/(float) accelsensitivity;
	dest2[2] = (float) accel_bias[2]/(float) accelsensitivity;

	mraa_i2c_write_byte_data(i2c_context, MAX21100_FIFO_CFG, 0x00); // Turn off FIFO

	// Switch to bank 2 to write gyro bias into gyro bias registers
	// Per MAXIM Integrated, these registers hold gyro bias data at a resolution
	// of 1 LSB/8.33 mdps, or more properly
	// 131 LSB/dps as expected from the 16-bit data and the 250 dps full range.
	// Gyro bias is 13-bits, 2-s complement and issub tracted from data result
	mraa_i2c_write_byte_data(i2c_context, MAX21100_BANK_SELECT, 0x02);  // select bank 2

	gyro_bias[0] *= -1;
	gyro_bias[1] *= -1;
	gyro_bias[2] *= -1;

	int16_t xGyroBias = (int16_t) gyro_bias[0];
	int16_t yGyroBias = (int16_t) gyro_bias[1];
	int16_t zGyroBias = (int16_t) gyro_bias[2];

	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_GYRO_X_H, ((xGyroBias >> 8) & 0x1F));  // load bias into gyro bias registers
	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_GYRO_X_L, (xGyroBias  & 0xFF));
	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_GYRO_Y_H, ((yGyroBias >> 8) & 0x1F));  // load bias into gyro bias registers
	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_GYRO_Y_L, (yGyroBias  & 0xFF));
	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_GYRO_Z_H, ((zGyroBias >> 8) & 0x1F));  // load bias into gyro bias registers
	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_GYRO_Z_L, (zGyroBias  & 0xFF));

	// Write accelerometer bias to accel bias registers also. In this case, there are 7-bits
	// 2-s complement to hold the accel bias and Maxim claims the sensitivity of the accel bias
	// is 1 LSB/ 6.4 mg or 32768/(2 g x 100) by my reckoning or 1/100th of what one would expect
	// therefore the accel bias must be divided first by 100 and then cast into the correct register
	// positions
	//
	accel_bias[0] *= -1/100;
	accel_bias[1] *= -1/100;
	accel_bias[2] *= -1/100;

	int16_t xAccelBias = (int16_t) accel_bias[0];
	int16_t yAccelBias = (int16_t) accel_bias[1];
	int16_t zAccelBias = (int16_t) accel_bias[2];


	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_COMP_ACC_X, (xAccelBias & 0x7F));  // load x bias into accel bias registers
	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_COMP_ACC_Y, (yAccelBias & 0x7F));  // load y bias into accel bias registers
	mraa_i2c_write_byte_data(i2c_context, MAX21100_BIAS_COMP_ACC_Z, (zAccelBias & 0x7F));  // load z bias into accel bias registers

	mraa_i2c_write_byte_data(i2c_context, MAX21100_BANK_SELECT, 0x00);  // select bank 0
}

void max_get_mres() {
//	switch (Mscale) {
//	// Possible magnetometer scales (and their register bit settings)are:
//		case MFS_4Gauss:
			mRes = 4.0f/32.768f; // Proper scale to return milliGauss
//			break;
//		case MFS_8Gauss:
//			mRes = 8.0f/32.768f; // Proper scale to return milliGauss
//			break;
//		case MFS_12Gauss:
//			mRes = 12.0f/32.768f; // Proper scale to return milliGauss
//			break;
//		case MFS_16Gauss:
//			mRes = 16.0f/32.768f; // Proper scale to return milliGauss
//			break;
//	}
}

void max_get_gres() {
//	switch (Gscale) {
//	// Possible gyro scales (and their register bit settings) are:
//	// 250 DPS (11), 500 DPS (10), 1000 DPS (01), and 2000 DPS  (00).
//		case GFS_250DPS:
//			gRes = 250.0f/32768.0f;
//			break;
//		case GFS_500DPS:
//			gRes = 500.0f/32768.0f;
//			break;
//		case GFS_1000DPS:
//			gRes = 1000.0f/32768.0f;
//			break;
//		case GFS_2000DPS:
			gRes = 2000.0f/32768.0f;
//			break;
//	}
}

void max_get_ares() {
//	switch (Ascale) {
//	// Possible accelerometer scales (and their register bit settings) are:
//	// 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
//		case AFS_2G:
			aRes = 2.0f/32768.0f;
//			break;
//		case AFS_4G:
//			aRes = 4.0f/32768.0f;
//			break;
//		case AFS_8G:
//			aRes = 8.0f/32768.0f;
//			break;
//		case AFS_16G:
//			aRes = 16.0f/32768.0f;
//			break;
//	}
}

void max_init(void) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);

	uint8_t who_am_i_val = mraa_i2c_read_byte_data(i2c_context, 0x20);
	printf("MAX who_am_i_val = 0x%02x\n", who_am_i_val);
	uint8_t revision_id = mraa_i2c_read_byte_data(i2c_context, 0x21);
	printf("MAX revision_id = 0x%02x\n", revision_id);

	max_selftest(i2c_context, SelfTest);// Start by performing self test and reporting values
	usleep(1000 * 1000);
	max_accel_gyro_calibrate(i2c_context, gyroBias, accelBias);// Calibrate gyro and accelerometers, load biases in bias registers
	usleep(1000 * 1000);

	max_init_bypass(i2c_context);
	usleep(1000 * 1000);
	hmc_init(i2c_context);
	bmp_init(i2c_context);
	usleep(100 * 1000);
	max_init_master(i2c_context);
	max_get_ares();
	max_get_gres();
	max_get_mres();
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

void max_read_accel_data(mraa_i2c_context i2c_context, int16_t *destination) {
	uint8_t rawData[6];  // x/y/z accel register data stored here
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_ACC_X_H, (uint8_t *)rawData, 6); // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;      // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
	printf("acc : %5d, %5d, %5d\n", destination[0], destination[1], destination[2]);
}

void max_read_gyro_data(mraa_i2c_context i2c_context, int16_t *destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_GYRO_X_H, (uint8_t *)rawData, 6); // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
	printf("gyr : %5d, %5d, %5d\n", destination[0], destination[1], destination[2]);
}

void max_read_mag_data(mraa_i2c_context i2c_context, int16_t * destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_MAG_X_H, (uint8_t *)rawData, 6); // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
	printf("mag : %5d, %5d, %5d\n", destination[0], destination[1], destination[2]);
}

void max_run() {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);

	max_bank_select(i2c_context, 0);

	status = mraa_i2c_read_byte_data(i2c_context, MAX21100_SYSTEM_STATUS);
	printf("status = 0x%02x\n", status);
	if( (status & 0x04) && !(status & 0x08) ) {  // check if accel data ready and no accel data error
		max_read_accel_data(i2c_context, accelCount);  // Read the x/y/z adc values
		// Now we'll calculate the accleration value into actual g's
		ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
		ay = (float)accelCount[1]*aRes - accelBias[1];
		az = (float)accelCount[2]*aRes - accelBias[2];
	}

	if( (status & 0x01) && !(status & 0x02) ) {  // check if gyro data ready  and no gyro data error
		max_read_gyro_data(i2c_context, gyroCount);  // Read the x/y/z adc values
		// Calculate the gyro value into actual degrees per second
		gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
		gy = (float)gyroCount[1]*gRes; // - gyroBias[1];
		gz = (float)gyroCount[2]*gRes ; //- gyroBias[2];
	}

	if( (status & 0x10) && !(status & 0x20) ) {  // if all three axes have new magnetometer data
		max_read_mag_data(i2c_context, magCount);  // Read the x/y/z adc values
		// Calculate the magnetometer values in milliGauss
		mx = (float)magCount[0]*mRes;//  - magBias[0];  // get actual magnetometer value, this depends on scale being set
		my = (float)magCount[1]*mRes;//  - magBias[1];
		mz = (float)magCount[2]*mRes;//  - magBias[2];
	}
	printf("acc : %5d, %5d, %5d\n", accelCount[0], accelCount[1], accelCount[2]);
	printf("gyr : %5d, %5d, %5d\n", gyroCount[0], gyroCount[1], gyroCount[2]);
	printf("mag : %5d, %5d, %5d\n", magCount[0], magCount[1], magCount[2]);
//	printf("acc : %5d, %5d, %5d\n", ax, ay, az);
//	printf("gyr : %5d, %5d, %5d\n", gx, gy, gz);
//	printf("mag : %5d, %5d, %5d\n", mx, my, mz);


















//	int8_t bluk_data[20];
//	int actual = mraa_i2c_read_bytes_data(i2c_context, 0x24, (uint8_t *)bluk_data, 20);
////	printf("=== [%d] ===\n", actual);
//
//	int16_t gyr_x =
//			bluk_data[0] << 8 |
//			bluk_data[1];
//	int16_t gyr_y =
//			bluk_data[2] << 8 |
//			bluk_data[3];
//	int16_t gyr_z =
//			bluk_data[4] << 8 |
//			bluk_data[5];
//	int16_t acc_x =
//			bluk_data[6] << 8 |
//			bluk_data[7];
//	int16_t acc_y =
//			bluk_data[8] << 8 |
//			bluk_data[9];
//	int16_t acc_z =
//			bluk_data[10] << 8 |
//			bluk_data[11];
//	int16_t mag_x =
//			bluk_data[12] << 8 |
//			bluk_data[13];
//	int16_t mag_y =
//			bluk_data[14] << 8 |
//			bluk_data[15];
//	int16_t mag_z =
//			bluk_data[16] << 8 |
//			bluk_data[17];
//	int16_t temp =
//			bluk_data[18] << 8 |
//			bluk_data[19];
//	float temperature = temp / 256.0f;
//	printf("gyr : %5d, %5d, %5d\n", gyr_x, gyr_y, gyr_z);
//	printf("acc : %5d, %5d, %5d\n", acc_x, acc_y, acc_z);
//	printf("mag : %5d, %5d, %5d\n", mag_x, mag_y, mag_z);
//	printf("temp : %.2f degC\n", temperature);
//
//	max_bank_select(i2c_context, 2);
//	int8_t quaternion_bluk_data[28];
//	actual = mraa_i2c_read_bytes_data(i2c_context, 0x00, (uint8_t *)quaternion_bluk_data, 28);
////	printf("==%d==\n", actual);
//	int16_t quat[4];
//	quat[0] =
//			quaternion_bluk_data[0] << 8 |
//			quaternion_bluk_data[1];
//	quat[1] =
//			quaternion_bluk_data[2] << 8 |
//			quaternion_bluk_data[3];
//	quat[2] =
//			quaternion_bluk_data[4] << 8 |
//			quaternion_bluk_data[5];
//	quat[3] =
//			quaternion_bluk_data[6] << 8 |
//			quaternion_bluk_data[7];
//	float norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
//	norm = 1.0f/norm;
//
//	float qx, qy, qz, qw;
//	qx = (float) quat[0] * norm;
//	qy = (float) quat[1] * norm;
//	qz = (float) quat[2] * norm;
//	qw = (float) quat[3] * norm;
////	printf("qua : %.2f, %.2f, %.2f, %.2f\n", qx, qy, qz, qw);
//
//	float angle_x = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
//	float angle_y = asin(2 * (qw * qy - qz * qx));
//	float angle_z = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
////	float angle_x = atan2(mag_z, mag_y);
////	float angle_y = atan2(mag_x, mag_z);
////	float angle_z = atan2(mag_y, mag_x);
//
//	angle_x = angle_x * 180 / M_PI;
//	while (angle_x < 0) angle_x += 360.0f;
//	angle_y = angle_y * 180 / M_PI;
//	while (angle_y < 0) angle_y += 360.0f;
//	angle_z = angle_z * 180 / M_PI;
//	while (angle_z < 0) angle_z += 360.0f;
//
////	angle_x = 0.0;
////	angle_y = 0.0;
////	angle_z = 0.0;
//
////	printf("%6.2f %6.2f %6.2f\n", angle_x, angle_y, angle_z);
//
//	unsigned char msg[12];
//	int c_i = 0;
//	unsigned char *pdata;
//	int i;
//
//	pdata = ((unsigned char *)&angle_x);
//	for (i = 0; i < 4; i ++) {
//		msg[c_i ++] = *pdata ++;
//	}
//
//	pdata = ((unsigned char *)&angle_y);
//	for (i = 0; i < 4; i ++) {
//		msg[c_i ++] = *pdata ++;
//	}
//
//	pdata = ((unsigned char *)&angle_z);
//	for (i = 0; i < 4; i ++) {
//		msg[c_i ++] = *pdata ++;
//	}
//	tcpserver_send(msg, 12);
}

