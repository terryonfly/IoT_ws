/*
 * MPU9250.h
 *
 *  Created on: Dec 12, 2015
 *      Author: terry
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "Math.h"

/*
|     |      ACCELEROMETER      |        GYROSCOPE        |
| LPF | BandW | Delay  | Sample | BandW | Delay  | Sample |
+-----+-------+--------+--------+-------+--------+--------+
|  0  | 260Hz |    0ms |  1kHz  | 256Hz | 0.98ms |  8kHz  |
|  1  | 184Hz |  2.0ms |  1kHz  | 188Hz |  1.9ms |  1kHz  |
|  2  |  94Hz |  3.0ms |  1kHz  |  98Hz |  2.8ms |  1kHz  |
|  3  |  44Hz |  4.9ms |  1kHz  |  42Hz |  4.8ms |  1kHz  |
|  4  |  21Hz |  8.5ms |  1kHz  |  20Hz |  8.3ms |  1kHz  |
|  5  |  10Hz | 13.8ms |  1kHz  |  10Hz | 13.4ms |  1kHz  |
|  6  |   5Hz | 19.0ms |  1kHz  |   5Hz | 18.6ms |  1kHz  |
|  7  | -- Reserved -- |  1kHz  | -- Reserved -- |  8kHz  |
*/
typedef enum {
	MPU_LPS_256Hz   = 0x00,
	MPU_LPS_188Hz   = 0x01,
	MPU_LPS_98Hz    = 0x02,
	MPU_LPS_42Hz    = 0x03,
	MPU_LPS_20Hz    = 0x04,
	MPU_LPS_10Hz    = 0x05,
	MPU_LPS_5Hz     = 0x06,
	MPU_LPS_Disable = 0x07,
} MPU_LPF_TypeDef;

typedef enum {
	MPU_GyrFS_250dps  = 0x00,
	MPU_GyrFS_500dps  = 0x08,
	MPU_GyrFS_1000dps = 0x10,
	MPU_GyrFS_2000dps = 0x18
} MPU_GyrFS_TypeDef;

typedef enum {
	MPU_AccFS_2g  = 0x00,
	MPU_AccFS_4g  = 0x08,
	MPU_AccFS_8g  = 0x10,
	MPU_AccFS_16g = 0x18
} MPU_AccFS_TypeDef;

typedef struct {
	MPU_LPF_TypeDef MPU_LowPassFilter;
	MPU_GyrFS_TypeDef MPU_Gyr_FullScale;
	MPU_AccFS_TypeDef MPU_Acc_FullScale;
} MPU_InitTypeDef;

/* ---- Sensitivity --------------------------------------------------------- */
#define MPU9250A_2g       0.000061035156f  // 0.000061035156 g/LSB
#define MPU9250A_4g       0.000122070312f  // 0.000122070312 g/LSB
#define MPU9250A_8g       0.000244140625f  // 0.000244140625 g/LSB
#define MPU9250A_16g      0.000488281250f  // 0.000488281250 g/LSB

#define MPU9250G_250dps   0.007633587786f  // 0.007633587786 dps/LSB
#define MPU9250G_500dps   0.015267175572f  // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  0.030487804878f  // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  0.060975609756f  // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   0.15f             // 0.15 uT/LSB

#define MPU9250T_85degC   0.002995177763f  // 0.002995177763 degC/LSB (1/333.87)

/* ---- MPU9250 Reg In MPU9250 ---------------------------------------------- */
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

#define MPU9250_BANK_SEL			0x6D
#define MPU9250_MEM_START_ADDR		0x6E
#define MPU9250_MEM_R_W				0x6F

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

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

#define MPU9250_I2C_ADDR            0x68
#define AK8963_I2C_ADDR             0x0C

typedef struct {
	int			magnet_enable;
	Vector3f 	accel;
	Vector3f 	gyro;
	Vector3f 	magnet;
	float 		temp;
	double		diff_sec;
} SensorData;

typedef struct {
	Vector3s 	accel_raw;
	Vector3s 	gyro_raw;
	Vector3s 	gyro_offset;
	Vector3s 	magnet_raw;
	Vector3s 	magnet_offset;
	Vector3f 	magnet_gain;
	int16_t 	temp_raw;
} SensorDataRaw;

double get_diff_time();

void delay_for_ms(int ms);

void mpu_init(void);

void mpu_release(void);

void mpu_run(void);

#endif /* MPU9250_H_ */
