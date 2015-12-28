///*
// * Poseture.h
// *
// *  Created on: 2015年12月28日
// *      Author: terry
// */

#ifndef POSETURE_H_
#define POSETURE_H_

#include "MPU9250.h"

int poseture_init(void);

void poseture_release(void);

void poseture_run(void);

void update_sensor_data(SensorData sd);

#endif /* POSETURE_H_ */
