///*
// * Poseture.c
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

#include "Poseture.h"
#include "StatusReport.h"

#include "MahonyAHRS/MahonyAHRS.h"

Quaternion accel_magnet_quaternion;

Quaternion gyro_quaternion;
Quaternion gyro_quaternion_integral = {1, 0, 0, 0};

pthread_t thread_id;
int thread_running;

SensorData sensor_data;
int sensor_data_updated = 0;
pthread_mutex_t update_mutex = PTHREAD_MUTEX_INITIALIZER;

int poseture_init(void) {
	int ret;
	thread_running = 1;
	ret = pthread_create(&thread_id, NULL, (void *)poseture_run, NULL);
	if (ret != 0) {
		perror("Create pthread error!\n");
		return -1;
	}
	return 0;
}

void poseture_release(void) {
	printf("Release pthread\n");
	thread_running = 0;
	pthread_join(thread_id, NULL);
}

void poseture_run(void) {
    while(thread_running){
    	while (!sensor_data_updated);
    	pthread_mutex_lock(&update_mutex);
    	if (sensor_data_updated) {
//    		euler_to_quaternion(sensor_data.gyro, &gyro_quaternion);
//    		gyro_quaternion_integral = quaternion_multiply(gyro_quaternion_integral, gyro_quaternion);
//    		sync_posture(gyro_quaternion_integral);

//    		Vector3f magnet_angles;
//    		magnet_angles.x = atan2(sensor_data.accel.z, sensor_data.accel.y);
//    		magnet_angles.y = atan2(sensor_data.magnet.x, sensor_data.magnet.z);
//    		magnet_angles.z = atan2(sensor_data.accel.y, sensor_data.accel.x);
//    		euler_to_quaternion(magnet_angles, &accel_magnet_quaternion);
//    		sync_posture(accel_magnet_quaternion);

    		MahonyAHRSupdateIMU(
    				sensor_data.gyro.x, sensor_data.gyro.y, sensor_data.gyro.z,
    				sensor_data.accel.x, sensor_data.accel.y, sensor_data.accel.z,
					sensor_data.diff_sec, &accel_magnet_quaternion);
    		sync_posture(accel_magnet_quaternion);

    		sensor_data_updated = 0;
    	}
    	pthread_mutex_unlock(&update_mutex);
    }
}

void update_sensor_data(SensorData sd) {
	pthread_mutex_lock(&update_mutex);
	sensor_data = sd;
	sensor_data_updated = 1;
	pthread_mutex_unlock(&update_mutex);
}

