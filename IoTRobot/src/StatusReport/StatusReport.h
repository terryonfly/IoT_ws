/*
 * StatusReport.h
 *
 *  Created on: 2015年12月28日
 *      Author: terry
 */

#ifndef STATUSREPORT_H_
#define STATUSREPORT_H_

#include "../Common/Math.h"
#include "../Posture/MPU9250.h"

int statusreport_init(void);

void statusreport_release(void);

void statusreport_run(void);

void sync_posture(SensorData sd, Quaternion sensor_quaternion);

void sync_action(float la, float ra, float lp, float rp);

void sync_pid(float cw, float cr);

void sync_data_ready();

#endif /* STATUSREPORT_H_ */
