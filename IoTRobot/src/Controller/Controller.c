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
#include "../StatusReport/StatusReport.h"

#define MOCK_WANT_MAX (float)1.0
#define MOCK_WANT_MIN (float)-1.0
#define MOCK_MAX_CHANGE_TIME 5 * 100// s
float mock_want = 0.0;
int mock_change_time = 0;

// -45.0 ~ 45.0
float left_angle = 0.0;
float right_angle = 0.0;
// -100.0 ~ 100.0
float left_power = 0.0;
float right_power = 0.0;

PID sPID_gyro_z;
PID sPID_angle_z;

#define ENGINE_MAX_POWER 100.0 // %

#define PID_POWER_COEFFICIENT (float)0.5
float pid_power_coefficient = 0.0;

float base_power = 5.0;

void ctl_init(void) {
	pid_power_coefficient = PID_POWER_COEFFICIENT * 1000 * 2.0 / (left_power_plus + right_power_plus);
	// Angle
	pid_init(&sPID_angle_z, 0.09, 0.0001, 0.05);
	// Gyro
	pid_init(&sPID_gyro_z, 1.5, 0.005, 10.0);
}

void ctl_release(void) {
	//
}

void ctl_run(void) {
//	mock_want = ctrl_x / 1000.0;
	sPID_angle_z.Proportion = ctrl_x / 1000.0;
	sPID_angle_z.Integral = ctrl_y / 1000.0;
	sPID_angle_z.Derivative = ctrl_z / 1000.0;
	base_power = ctrl_w / 1000.0;

	mock_change_time ++;
	if (mock_change_time > MOCK_MAX_CHANGE_TIME) {
		mock_change_time = 0;
		printf("%5.2f     ", mock_want);
		if (mock_want < 0)
			mock_want = MOCK_WANT_MAX;
		else
			mock_want = MOCK_WANT_MIN;
		printf("->     %5.2f\n", mock_want);
	}

	// outside PID of Roll : angle
	float posture_euler_z = posture_euler.z;
	while (posture_euler_z > M_PI) posture_euler_z -= 2 * M_PI;
	while (posture_euler_z < -M_PI) posture_euler_z += 2 * M_PI;
	float err_euler = posture_euler_z - mock_want;
	if (err_euler > M_PI) posture_euler_z -= 2 * M_PI;
	if (err_euler < -M_PI) posture_euler_z += 2 * M_PI;
	double ctrl_angle_z = pid_run(&sPID_angle_z, posture_euler_z, mock_want);
	// inside PID of Roll : gyro
	double ctrl_power_z = pid_run(&sPID_gyro_z, sensor_data.gyro.z, ctrl_angle_z);
	float left_power_axis_yaw = base_power + ctrl_power_z * pid_power_coefficient;
	float right_power_axis_yaw = base_power - ctrl_power_z * pid_power_coefficient;
	if (left_power_axis_yaw < 0) left_power_axis_yaw = 0.0;
	if (left_power_axis_yaw > ENGINE_MAX_POWER) left_power_axis_yaw = ENGINE_MAX_POWER;
	if (right_power_axis_yaw < 0) right_power_axis_yaw = 0.0;
	if (right_power_axis_yaw > ENGINE_MAX_POWER) right_power_axis_yaw = ENGINE_MAX_POWER;
	left_power = left_power_axis_yaw;
	right_power = right_power_axis_yaw;

	sync_pid(mock_want, posture_euler_z);
}
