/*
 * Servo.h
 *
 *  Created on: Dec 15, 2015
 *      Author: terry
 */

#ifndef PCA9685_H_
#define PCA9685_H_

void pca_init(void);

void pca_release(void);

void pca_run(float left_angle, float right_angle, float left_power, float right_power);

#endif /* PCA9685_H_ */
