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

// Specify sensor full scale
uint8_t OSR = ADC_8192;       // set pressure amd temperature oversample rate
uint8_t Gscale = GFS_250DPS;  // Gyro full scale
uint8_t Godr = GODR_250Hz;    // Gyro sample rate
uint8_t Gbw = GBW_22Hz;       // Gyro bandwidth
uint8_t Ascale = AFS_2G;      // Accel full scale
uint8_t Aodr = AODR_250Hz;    // Accel sample rate
uint8_t Abw = ABW_div9;       // Accel bandwidth, accel sample rate divided by ABW_divx
uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
uint8_t Mopmode = MOM_hiperf; // Select magnetometer perfomance mode
uint8_t Modr = MODR_10Hz;     // Select magnetometer ODR when in MAX21100 bypass mode
uint8_t MSodr = MODR_div16;   // Select magnetometer ODR as Aodr/MODR_divx
uint8_t powerSelect = 0x00;   // no DSYNC enable or usage
uint8_t powerMode = accelnormalgyronormalmode;  // specify power mode for accel + gyro
uint8_t status;               // MAX21100 data status register
float aRes, gRes, mRes;       // scale resolutions per LSB for the sensors

uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempGCount, tempMCount;      // temperature raw count output of mag and gyro
float   Gtemperature, Mtemperature; // Stores the MAX21100 gyro and LIS3MDL mag internal chip temperatures in degrees Celsius
double Temperature, Pressure;       // stores MS5637 pressures sensor pressure and temperature
float SelfTest[12];                  // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = M_PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta;// = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta;// = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

void selftestMAX21100(float * selfTest) {
	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);

	int16_t gyroSelfTestplus[3] = {0, 0, 0}, gyroSelfTestminus[3] = {0, 0, 0};
	int16_t accelSelfTestplus[3] = {0, 0, 0}, accelSelfTestminus[3] = {0, 0, 0};
	uint8_t rawData[6];

	// Enable power select to control power mode from DSYNC (enable = 0x80, disable = 0x00)
	// choose power mode (accelnormal_gyronormal = 0x0F in bits 6:3
	// Enable all axes (z = bit 2, y = bit 1, x = bit 0)
	mraa_i2c_write_byte_data(i2c_context, 0x0F << 3 | 0x07, MAX21100_POWER_CFG);

	// Configure gyro
	// Select gyro bandwidth (bits 5:2) and gyro full scale (bits 1:0)
	float gyrosensitivity  = 131.0f;   // = 131 LSB/degrees/sec per data sheet
	mraa_i2c_write_byte_data(i2c_context, 0x40 | GBW_14Hz << 2 | GFS_250DPS, MAX21100_GYRO_CFG1); // positive deflection
	usleep(100 * 1000);

	mraa_i2c_read_bytes_data(i2c_context, MAX21100_GYRO_X_H, rawData, 6); // Read the six raw data registers sequentially into data array
	gyroSelfTestplus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);   // Turn the MSB and LSB into a signed 16-bit value
	gyroSelfTestplus[1] = (int16_t) (((int16_t)rawData[2] << 8) | rawData[3]);
	gyroSelfTestplus[2] = (int16_t) (((int16_t)rawData[4] << 8) | rawData[5]);

	mraa_i2c_write_byte_data(i2c_context, 0x80 | GBW_14Hz << 2 | GFS_250DPS, MAX21100_GYRO_CFG1); // negative deflection
	usleep(100 * 1000);

	mraa_i2c_read_bytes_data(i2c_context, MAX21100_GYRO_X_H, rawData, 6); // Read the six raw data registers sequentially into data array
	gyroSelfTestminus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
	gyroSelfTestminus[1] = (int16_t) (((int16_t)rawData[2] << 8) | rawData[3]);
	gyroSelfTestminus[2] = (int16_t) (((int16_t)rawData[4] << 8) | rawData[5]);

	selfTest[0] = (float) gyroSelfTestplus[0] / gyrosensitivity;
	selfTest[1] = (float) gyroSelfTestplus[1] / gyrosensitivity;
	selfTest[2] = (float) gyroSelfTestplus[2] / gyrosensitivity;
	selfTest[3] = (float) gyroSelfTestminus[0] / gyrosensitivity;
	selfTest[4] = (float) gyroSelfTestminus[1] / gyrosensitivity;
	selfTest[5] = (float) gyroSelfTestminus[2] / gyrosensitivity;

	// disable gyro self test mode
	mraa_i2c_write_byte_data(i2c_context, 0x00 | GBW_14Hz << 2 | GFS_250DPS, MAX21100_GYRO_CFG1);

	// Configure the accelerometer
	// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
	float accelsensitivity = 16384.0f;  // = 16384 LSB/g per data sheet

	//  positive x-axis deflection
	mraa_i2c_write_byte_data(i2c_context, AFS_2G << 6 | 0x08 | 0x07, MAX21100_PWR_ACC_CFG); // pos x axis
	usleep(1000 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_ACC_X_H, rawData, 2); // Read the raw data registers into data array
	accelSelfTestplus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	//  positive y-axis deflection
	mraa_i2c_write_byte_data(i2c_context, AFS_2G << 6 | 0x10 | 0x07, MAX21100_PWR_ACC_CFG); // pos y axis
	usleep(1000 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_ACC_Y_H, rawData, 2); // Read the raw data registers into data array
	accelSelfTestplus[1] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);   // Turn the MSB and LSB into a signed 16-bit value

	//  positive z-axis deflection
	mraa_i2c_write_byte_data(i2c_context, AFS_2G << 6 | 0x18 | 0x07, MAX21100_PWR_ACC_CFG); // pos z axis
	usleep(1000 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_ACC_Z_H, rawData, 2); // Read the raw data registers into data array
	accelSelfTestplus[2] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	//  negative x-axis deflection
	mraa_i2c_write_byte_data(i2c_context, AFS_2G << 6 | 0x28 | 0x07, MAX21100_PWR_ACC_CFG); // neg x axis
	usleep(1000 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_ACC_X_H, rawData, 2); // Read the raw data registers into data array
	accelSelfTestminus[0] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	//  negative y-axis deflection
	mraa_i2c_write_byte_data(i2c_context, AFS_2G << 6 | 0x30 | 0x07, MAX21100_PWR_ACC_CFG); // neg y axis
	usleep(1000 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_ACC_Y_H, rawData, 2); // Read the raw data registers into data array
	accelSelfTestminus[1] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	//  negative z-axis deflection
	mraa_i2c_write_byte_data(i2c_context, AFS_2G << 6 | 0x38 | 0x07, MAX21100_PWR_ACC_CFG); // neg z axis
	usleep(1000 * 1000);
	mraa_i2c_read_bytes_data(i2c_context, MAX21100_ACC_Z_H, rawData, 2); // Read the raw data registers into data array
	accelSelfTestminus[2] = (int16_t) (((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value

	selfTest[6] = (float) accelSelfTestplus[0] / accelsensitivity;
	selfTest[7] = (float) accelSelfTestplus[1] / accelsensitivity;
	selfTest[8] = (float) accelSelfTestplus[2] / accelsensitivity;
	selfTest[9] = (float) accelSelfTestminus[0] / accelsensitivity;
	selfTest[10] = (float) accelSelfTestminus[1] / accelsensitivity;
	selfTest[11] = (float) accelSelfTestminus[2] / accelsensitivity;

	// disable accel self test
	mraa_i2c_write_byte_data(i2c_context, AFS_2G << 6 | 0x00 | 0x07, MAX21100_PWR_ACC_CFG); // end accel self test
}

void max_init(void) {
	beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
	zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	mraa_i2c_context i2c_context = i2cbus_get_instance();
	if (i2c_context == NULL)
		perror("Err : i2c_context = NULL");
	if (mraa_i2c_address(i2c_context, MAX21100_ADDR) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", MAX21100_ADDR);

	uint8_t who_am_i_val = mraa_i2c_read_byte_data(i2c_context, MAX21100_WHO_AM_I);
	printf("MAX who_am_i_val = 0x%02x\n", who_am_i_val);
	uint8_t revision_id = mraa_i2c_read_byte_data(i2c_context, MAX21100_REVISION_ID);
	printf("MAX revision_id = 0x%02x\n", revision_id);

	selftestMAX21100(SelfTest); // Start by performing self test and reporting values
	printf("x-axis self test +: gyration : %.2f should be +55\n", SelfTest[0]);
	printf("y-axis self test -: gyration : %.2f should be -55\n", SelfTest[1]);
	printf("z-axis self test +: gyration : %.2f should be +55\n", SelfTest[2]);
	printf("x-axis self test -: gyration : %.2f should be -55\n", SelfTest[3]);
	printf("y-axis self test +: gyration : %.2f should be +55\n", SelfTest[4]);
	printf("z-axis self test -: gyration : %.2f should be -55\n", SelfTest[5]);
	printf("x-axis self test +: acceleration : %.2f should be +300 mg\n", 1000*SelfTest[6]);
	printf("y-axis self test -: acceleration : %.2f should be -300 mg\n", 1000*SelfTest[7]);
	printf("z-axis self test +: acceleration : %.2f should be +300 mg\n", 1000*SelfTest[8]);
	printf("x-axis self test -: acceleration : %.2f should be -300 mg\n", 1000*SelfTest[9]);
	printf("y-axis self test +: acceleration : %.2f should be +300 mg\n", 1000*SelfTest[10]);
	printf("z-axis self test -: acceleration : %.2f should be -300 mg\n", 1000*SelfTest[11]);

//	accelgyrocalMAX21100(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
//	println(1000*accelBias[0]);  println(1000*accelBias[1]);  print(1000*accelBias[2]); println(" mg");
//	println(gyroBias[0]);  println(gyroBias[1]);  print(gyroBias[2]); println(" dps");
//
//	initbypassMAX21100(); // Treat MAX21100 and LIS3MDL and MS5637 as slaves to the Teensy 3.1 master
//	println("MAX21100 initialized for bypass mode...."); // Initialize MAX21100 for direct read of magnetometer by microcontroller
//
//	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
//	byte d = readByte(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I);  // Read WHO_AM_I register for LIS3MDL
//	print("LIS3MDL "); print("I AM "); print(d, HEX); print(" I should be "); println(0x3D, HEX);
//
//	// Initialize and set up LIS3MDL magnetometer
//	initLIS3MDL(); println("LIS3MDL initialized for active data mode...."); // Initialize device for active mode read of magnetometer
//
//	// Reset the MS5637 pressure sensor
//	MS5637Reset();
//	usleep(100 * 1000);
//	println("MS5637 pressure sensor reset...");
//	// Read PROM data from MS5637 pressure sensor
//	MS5637PromRead(Pcal);
//	println("PROM data read:");
//	print("C0 = "); println(Pcal[0]);
//	unsigned char refCRC = Pcal[0] >> 12;
//	print("C1 = "); println(Pcal[1]);
//	print("C2 = "); println(Pcal[2]);
//	print("C3 = "); println(Pcal[3]);
//	print("C4 = "); println(Pcal[4]);
//	print("C5 = "); println(Pcal[5]);
//	print("C6 = "); println(Pcal[6]);
//
//	nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
//	print("Checksum = "); print(nCRC); print(" , should be "); println(refCRC);
//
//	initmasterMAX21100(); // Let the MAX21100 be master to the LIS3MDL slave
//	println("MAX21100 initialized for master mode...."); // Initialize MAX21100 for read of magnetometer as master
//
//	// get sensor resolutions, only need to do this once
//	getAres();
//	getGres();
//	getMres();
//
//	magcalMAX21100(magBias);  // calibrate the LIS3MDL magnetometer
//	println(magBias[0]);  println(magBias[1]);  print(magBias[2]); println(" mG");
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
}

