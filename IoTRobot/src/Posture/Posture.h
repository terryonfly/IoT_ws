///*
// * Posture.h
// *
// *  Created on: 2015年12月28日
// *      Author: terry
// */

#ifndef POSTURE_H_
#define POSTURE_H_

#include "MPU9250.h"

extern SensorData sensor_data;
extern Quaternion posture_quaternion;
extern Vector3f posture_euler;

void pos_init(void);

void pos_release(void);

void pos_run(void);

#endif /* POSTURE_H_ */
