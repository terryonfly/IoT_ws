/*
 * StatusReport.h
 *
 *  Created on: 2015年12月28日
 *      Author: terry
 */

#ifndef STATUSREPORT_H_
#define STATUSREPORT_H_

#include "Math.h"

int statusreport_init(void);

void statusreport_release(void);

void statusreport_run(void);

void sync_posture(Quaternion sensor_quaternion);

#endif /* STATUSREPORT_H_ */