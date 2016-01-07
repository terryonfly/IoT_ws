///*
// * Posture.c
// *
// *  Created on: 2015年12月28日
// *      Author: terry
// */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <pthread.h>
#include <math.h>

#include "../StatusReport/StatusReport.h"

#include "Posture.h"
#include "MahonyAHRS.h"

Quaternion posture_quaternion = {1.f, 0.f, 0.f, 0.f};
Vector3f posture_euler = {0.f, 0.f, 0.f};

void pos_init(void) {

}

void pos_release(void) {

}

void pos_run(void) {
	if (0) {// Magnet offset is ready
		MahonyAHRSupdate(
				sensor_data.gyro,
				sensor_data.accel,
				sensor_data.magnet,
				sensor_data.diff_sec,
				&posture_quaternion);
	} else {// Magnet offset is not ready
		MahonyAHRSupdateIMU(
				sensor_data.gyro,
				sensor_data.accel,
				sensor_data.diff_sec,
				&posture_quaternion);
	}
	quaternion_to_euler(posture_quaternion, &posture_euler);
	sync_posture(sensor_data, posture_quaternion);
}

