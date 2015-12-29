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

#include "StatusReport.h"

#include "MahonyAHRS/MahonyAHRS.h"
#include "Posture.h"

Quaternion posture_quaternion = {1.f, 0.f, 0.f, 0.f};

pthread_t thread_id;
int thread_running;

SensorData sensor_data;
int sensor_data_updated = 0;
pthread_mutex_t update_mutex = PTHREAD_MUTEX_INITIALIZER;

int posture_init(void) {
	int ret;
	thread_running = 1;
	ret = pthread_create(&thread_id, NULL, (void *)posture_run, NULL);
	if (ret != 0) {
		perror("Create pthread error!\n");
		return -1;
	}
	return 0;
}

void posture_release(void) {
	printf("Release pthread\n");
	thread_running = 0;
	pthread_join(thread_id, NULL);
}

void posture_run(void) {
    while(thread_running){
    	while (!sensor_data_updated);
    	pthread_mutex_lock(&update_mutex);
    	if (sensor_data_updated) {
    		if (1) {// Magnet offset is ready
				MahonyAHRSupdateIMU(
						sensor_data.gyro,
						sensor_data.accel,
						sensor_data.diff_sec,
						&posture_quaternion);
    		} else {// Magnet offset is not ready
				MahonyAHRSupdate(
						sensor_data.gyro,
						sensor_data.accel,
						sensor_data.magnet,
						sensor_data.diff_sec,
						&posture_quaternion);
    		}
    		sync_posture(sensor_data, posture_quaternion);

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

