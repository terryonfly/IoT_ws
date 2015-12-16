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
#include "TCPServer.h"


/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g       0.000061035156f  // 0.000061035156 g/LSB
#define MPU9250A_4g       0.000122070312f  // 0.000122070312 g/LSB
#define MPU9250A_8g       0.000244140625f  // 0.000244140625 g/LSB
#define MPU9250A_16g      0.000488281250f  // 0.000488281250 g/LSB

#define MPU9250G_250dps   0.007633587786f  // 0.007633587786 dps/LSB
#define MPU9250G_500dps   0.015267175572f  // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  0.030487804878f  // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  0.060975609756f  // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   0.6f             // 0.6 uT/LSB

#define MPU9250T_85degC   0.002995177763f  // 0.002995177763 degC/LSB (1/333.87)

/* ---- MPU9250 Reg In MPU9250 ---------------------------------------------- */

#define MPU9250_I2C_ADDR            0x68
#define MPU9250_Device_ID           0x71  // In MPU9250

#define MPU9250_SELF_TEST_XG        0x00
#define MPU9250_SELF_TEST_YG        0x01
#define MPU9250_SELF_TEST_ZG        0x02
#define MPU9250_SELF_TEST_XA        0x0D
#define MPU9250_SELF_TEST_YA        0x0E
#define MPU9250_SELF_TEST_ZA        0x0F
#define MPU9250_XG_OFFSET_H         0x13
#define MPU9250_XG_OFFSET_L         0x14
#define MPU9250_YG_OFFSET_H         0x15
#define MPU9250_YG_OFFSET_L         0x16
#define MPU9250_ZG_OFFSET_H         0x17
#define MPU9250_ZG_OFFSET_L         0x18
#define MPU9250_SMPLRT_DIV          0x19
#define MPU9250_CONFIG              0x1A
#define MPU9250_GYRO_CONFIG         0x1B
#define MPU9250_ACCEL_CONFIG        0x1C
#define MPU9250_ACCEL_CONFIG_2      0x1D
#define MPU9250_LP_ACCEL_ODR        0x1E
#define MPU9250_MOT_THR             0x1F
#define MPU9250_FIFO_EN             0x23
#define MPU9250_I2C_MST_CTRL        0x24
#define MPU9250_I2C_SLV0_ADDR       0x25
#define MPU9250_I2C_SLV0_REG        0x26
#define MPU9250_I2C_SLV0_CTRL       0x27
#define MPU9250_I2C_SLV1_ADDR       0x28
#define MPU9250_I2C_SLV1_REG        0x29
#define MPU9250_I2C_SLV1_CTRL       0x2A
#define MPU9250_I2C_SLV2_ADDR       0x2B
#define MPU9250_I2C_SLV2_REG        0x2C
#define MPU9250_I2C_SLV2_CTRL       0x2D
#define MPU9250_I2C_SLV3_ADDR       0x2E
#define MPU9250_I2C_SLV3_REG        0x2F
#define MPU9250_I2C_SLV3_CTRL       0x30
#define MPU9250_I2C_SLV4_ADDR       0x31
#define MPU9250_I2C_SLV4_REG        0x32
#define MPU9250_I2C_SLV4_DO         0x33
#define MPU9250_I2C_SLV4_CTRL       0x34
#define MPU9250_I2C_SLV4_DI         0x35
#define MPU9250_I2C_MST_STATUS      0x36
#define MPU9250_INT_PIN_CFG         0x37
#define MPU9250_INT_ENABLE          0x38
#define MPU9250_INT_STATUS          0x3A
#define MPU9250_ACCEL_XOUT_H        0x3B
#define MPU9250_ACCEL_XOUT_L        0x3C
#define MPU9250_ACCEL_YOUT_H        0x3D
#define MPU9250_ACCEL_YOUT_L        0x3E
#define MPU9250_ACCEL_ZOUT_H        0x3F
#define MPU9250_ACCEL_ZOUT_L        0x40
#define MPU9250_TEMP_OUT_H          0x41
#define MPU9250_TEMP_OUT_L          0x42
#define MPU9250_GYRO_XOUT_H         0x43
#define MPU9250_GYRO_XOUT_L         0x44
#define MPU9250_GYRO_YOUT_H         0x45
#define MPU9250_GYRO_YOUT_L         0x46
#define MPU9250_GYRO_ZOUT_H         0x47
#define MPU9250_GYRO_ZOUT_L         0x48
#define MPU9250_EXT_SENS_DATA_00    0x49
#define MPU9250_EXT_SENS_DATA_01    0x4A
#define MPU9250_EXT_SENS_DATA_02    0x4B
#define MPU9250_EXT_SENS_DATA_03    0x4C
#define MPU9250_EXT_SENS_DATA_04    0x4D
#define MPU9250_EXT_SENS_DATA_05    0x4E
#define MPU9250_EXT_SENS_DATA_06    0x4F
#define MPU9250_EXT_SENS_DATA_07    0x50
#define MPU9250_EXT_SENS_DATA_08    0x51
#define MPU9250_EXT_SENS_DATA_09    0x52
#define MPU9250_EXT_SENS_DATA_10    0x53
#define MPU9250_EXT_SENS_DATA_11    0x54
#define MPU9250_EXT_SENS_DATA_12    0x55
#define MPU9250_EXT_SENS_DATA_13    0x56
#define MPU9250_EXT_SENS_DATA_14    0x57
#define MPU9250_EXT_SENS_DATA_15    0x58
#define MPU9250_EXT_SENS_DATA_16    0x59
#define MPU9250_EXT_SENS_DATA_17    0x5A
#define MPU9250_EXT_SENS_DATA_18    0x5B
#define MPU9250_EXT_SENS_DATA_19    0x5C
#define MPU9250_EXT_SENS_DATA_20    0x5D
#define MPU9250_EXT_SENS_DATA_21    0x5E
#define MPU9250_EXT_SENS_DATA_22    0x5F
#define MPU9250_EXT_SENS_DATA_23    0x60
#define MPU9250_I2C_SLV0_DO         0x63
#define MPU9250_I2C_SLV1_DO         0x64
#define MPU9250_I2C_SLV2_DO         0x65
#define MPU9250_I2C_SLV3_DO         0x66
#define MPU9250_I2C_MST_DELAY_CTRL  0x67
#define MPU9250_SIGNAL_PATH_RESET   0x68
#define MPU9250_MOT_DETECT_CTRL     0x69
#define MPU9250_USER_CTRL           0x6A
#define MPU9250_PWR_MGMT_1          0x6B
#define MPU9250_PWR_MGMT_2          0x6C
#define MPU9250_FIFO_COUNTH         0x72
#define MPU9250_FIFO_COUNTL         0x73
#define MPU9250_FIFO_R_W            0x74
#define MPU9250_WHO_AM_I            0x75    // ID = 0x71 In MPU9250
#define MPU9250_XA_OFFSET_H         0x77
#define MPU9250_XA_OFFSET_L         0x78
#define MPU9250_YA_OFFSET_H         0x7A
#define MPU9250_YA_OFFSET_L         0x7B
#define MPU9250_ZA_OFFSET_H         0x7D
#define MPU9250_ZA_OFFSET_L         0x7E

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             0x0C
#define AK8963_Device_ID            0x48

// Read-only Reg
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

#define I2C_BUS 		0x06

mraa_i2c_context i2c_context;

mraa_result_t mpu_i2c_write_byte_data(mraa_i2c_context dev, const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t data) {
	if (mraa_i2c_address(i2c_context, dev_addr) != MRAA_SUCCESS)
		printf("can not found 0x%02x sensor\n", dev_addr);
	mraa_result_t ret = mraa_i2c_write_byte_data(i2c_context, data, reg_addr);
//	printf("write to 0x%02x 0x%02x -> 0x%02x %s\n", dev_addr, reg_addr, data, (ret == MRAA_SUCCESS) ? "SUCCESS" : "FAIL");
	return ret;
}

uint8_t mpu_i2c_read_byte_data(mraa_i2c_context dev, const uint8_t dev_addr, const uint8_t reg_addr) {
	if (mraa_i2c_address(i2c_context, dev_addr) != MRAA_SUCCESS)
			printf("can not found 0x%02x sensor\n", dev_addr);
	uint8_t data = mraa_i2c_read_byte_data(i2c_context, reg_addr);
//	printf("read from 0x%02x 0x%02x <- 0x%02x\n", dev_addr, reg_addr, data);
	return data;
}

void mpu_set_bypass(uint8_t status) {
	printf("mpu_set_bypass : %d\n", status);
    uint8_t pincfg;
    uint8_t usrctl;
    pincfg = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG);
//    usrctl = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_USER_CTRL);
    if (status) {
//        usrctl &= 0xDF;
//        pincfg |= 0x02;
    }
    else {
//        usrctl |= 0x20;
        pincfg &= 0xFD;
    }
//    mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_USER_CTRL, usrctl);
    mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, pincfg);
}

void mpu_init(void) {
	i2c_context = mraa_i2c_init(I2C_BUS);
	mraa_i2c_frequency(i2c_context, MRAA_I2C_FAST);

	// Init
	uint8_t i = 0;
	typedef struct
	{
		const uint8_t res_addr;
		const uint8_t value;
	}Smpu9250_InitData;
	Smpu9250_InitData Mpu9250_InitData[] = {
		{MPU9250_PWR_MGMT_2,          0x00},  // Enable Acc & Gyro
		{MPU9250_CONFIG,              0x07},
		{MPU9250_GYRO_CONFIG,         0x18},  // +-2000dps
		{MPU9250_ACCEL_CONFIG,        0x08},  // +-4G
		{MPU9250_ACCEL_CONFIG_2,      0x00},  // Set Acc Data Rates
		{MPU9250_I2C_MST_CTRL,        0x0D},  // I2C Speed 400 kHz

		// Set Slave to Read AK8963
		{MPU9250_I2C_SLV0_ADDR,       0x8C},  // AK8963_I2C_ADDR (0x0C| Read)
		{MPU9250_I2C_SLV0_REG,        AK8963_ST1},
		{MPU9250_I2C_SLV0_CTRL,       0x89},  // Enable 8 bytes read 10001001
	};
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_1, 0x80);
	usleep(1 * 1000);
	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, MPU9250_USER_CTRL, 0x00);
	mpu_set_bypass(1);
	mpu_i2c_write_byte_data(i2c_context, AK8963_I2C_ADDR, AK8963_CNTL1, 0x16);
	mpu_set_bypass(0);
    for (i = 0; i < sizeof(Mpu9250_InitData) / sizeof(Smpu9250_InitData); i++) {
    	mpu_i2c_write_byte_data(i2c_context, MPU9250_I2C_ADDR, Mpu9250_InitData[i].res_addr, Mpu9250_InitData[i].value);
    }
}

void mpu_release(void) {
	mraa_i2c_stop(i2c_context);
}

void mpu_run(void) {
	// Accelerometer
	int16_t acc_x, acc_y, acc_z;
	acc_x = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x3B) << 8 |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x3C);
	acc_y = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x3D) << 8 |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x3E);
	acc_z = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x3F) << 8 |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x40);
	float accelerometer_x = (float)acc_x * MPU9250A_4g;
	float accelerometer_y = (float)acc_y * MPU9250A_4g;
	float accelerometer_z = (float)acc_z * MPU9250A_4g;

	// Temperature
	int16_t temp;
	temp = 	mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x41) << 8 |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x42);
	float temperature = (float)temp * MPU9250T_85degC + 21.0f;

	// Gyroscope
	int16_t gyr_x, gyr_y, gyr_z;
	gyr_x = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x43) << 8 |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x44);
	gyr_y = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x45) << 8 |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x46);
	gyr_z = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x47) << 8 |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x48);
	float gyroscope_x = (float)gyr_x * MPU9250G_2000dps * M_PI / 180.0;
	float gyroscope_y = (float)gyr_y * MPU9250G_2000dps * M_PI / 180.0;
	float gyroscope_z = (float)gyr_z * MPU9250G_2000dps * M_PI / 180.0;

	// Magnetometer
	int16_t mag_x, mag_y, mag_z;
	mag_x = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x49) |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x4A) << 8;
	mag_y = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x4B) |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x4C) << 8;
	mag_z = mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x4D) |
			mpu_i2c_read_byte_data(i2c_context, MPU9250_I2C_ADDR, 0x4E) << 8;
	float magnetometer_x = (float)mag_x * MPU9250M_4800uT;
	float magnetometer_y = (float)mag_y * MPU9250M_4800uT;
	float magnetometer_z = (float)mag_z * MPU9250M_4800uT;

	printf("tempr : %f degC\n",
			temperature);
	printf("accel : %6.2f %6.2f %6.2f\n",
			accelerometer_x,
			accelerometer_y,
			accelerometer_z);
	printf("gyros : %6.2f %6.2f %6.2f\n",
			gyroscope_x,
			gyroscope_y,
			gyroscope_z);
	printf("magen : %6.2f %6.2f %6.2f\n",
			magnetometer_x,
			magnetometer_y,
			magnetometer_z);

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
