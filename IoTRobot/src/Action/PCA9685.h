/*
 * Servo.h
 *
 *  Created on: Dec 15, 2015
 *      Author: terry
 */

#ifndef PCA9685_H_
#define PCA9685_H_

extern float left_angle;
extern float right_angle;
extern float left_power;
extern float right_power;

void pca_init(void);

void pca_release(void);

void pca_run();

#endif /* PCA9685_H_ */
