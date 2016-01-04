/*
 * I2CBus.h
 *
 *  Created on: 2015年12月16日
 *      Author: terry
 */

#ifndef I2CBUS_H_
#define I2CBUS_H_

void i2cbus_init(void);

void i2cbus_release(void);

mraa_i2c_context i2cbus_get_instance_mpu(void);

mraa_i2c_context i2cbus_get_instance_pca(void);

#endif /* I2CBUS_H_ */
