/*
 * ControlPID.h
 *
 *  Created on: 2016年1月6日
 *      Author: terry
 */

#ifndef CONTROLLER_CONTROLLER_H_
#define CONTROLLER_CONTROLLER_H_

#include "../Common/Math.h"
#include "../Posture/MPU9250.h"

extern SensorData sensor_data;
extern Quaternion posture_quaternion;

void ctl_init(void);

void ctl_release(void);

void ctl_run(void);

#endif /* CONTROLLER_CONTROLLER_H_ */
