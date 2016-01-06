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


float left_angle = 0.5;
float right_angle = 0.5;
float left_power = 0.0;
float right_power = 0.0;

void ctl_init(void) {

}

void ctl_release(void) {

}

void ctl_run(void) {
//	sensor_data.gyro.x
//	sensor_data.gyro.y
//	sensor_data.gyro.z
}
