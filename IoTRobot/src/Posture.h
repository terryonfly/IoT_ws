///*
// * Posture.h
// *
// *  Created on: 2015年12月28日
// *      Author: terry
// */

#ifndef POSTURE_H_
#define POSTURE_H_

#include "MPU9250.h"

int posture_init(void);

void posture_release(void);

void posture_run(void);

void update_sensor_data(SensorData sd);

#endif /* POSTURE_H_ */
