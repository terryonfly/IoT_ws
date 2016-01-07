/*
 * ControlPID.c
 *
 *  Created on: 2016年1月6日
 *      Author: terry
 */
#include "Controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <pthread.h>
#include <math.h>

#include "PID.h"

float left_angle = 0.0;
float right_angle = 0.0;
float left_power = 0.0;
float right_power = 0.0;

PID sPID_gyro_z;
PID sPID_angle_z;

void ctl_init(void) {
	pid_init(&sPID_gyro_z, 0.5, 0.0, 0.0);
	pid_init(&sPID_angle_z, 0.5, 0.0, 0.0);
}

void ctl_release(void) {
	//
}

#define MAX_POWER 0.5

void ctl_run(void) {
	double ctrl_angle_z = pid_run(&sPID_angle_z, posture_euler.z, 0.0);
//	printf("%.2f -> %.2f ", posture_euler.z, ctrl_angle_z);
	ctrl_angle_z *= 8.f;
	if (ctrl_angle_z > 2.0) ctrl_angle_z = 2.0;
	if (ctrl_angle_z < -2.0) ctrl_angle_z = -2.0;

	double ctrl_power_z = pid_run(&sPID_gyro_z, sensor_data.gyro.z, ctrl_angle_z);
//	printf(" %.2f -> %.2f\n", sensor_data.gyro.z, ctrl_power_z);
	float left_power_tmp = ctrl_power_z / 5.0;
	float right_power_tmp = -ctrl_power_z / 5.0;
	if (left_power_tmp < 0) left_power_tmp = 0.0;
	if (left_power_tmp > MAX_POWER) left_power_tmp = MAX_POWER;
	if (right_power_tmp < 0) right_power_tmp = 0.0;
	if (right_power_tmp > MAX_POWER) right_power_tmp = MAX_POWER;
	left_power = left_power_tmp + 0.15;
	right_power = right_power_tmp + 0.15;
}
