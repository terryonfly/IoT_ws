/*
 * PostureReport.c
 *
 *  Created on: 2015年12月28日
 *      Author: terry
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <pthread.h>
#include <math.h>

#include "StatusReport.h"
#include "TCPServer.h"

pthread_t thread_id;
int thread_running;

float left_angle, right_angle, left_power, right_power;
SensorData sensor_data;
Quaternion sensor_quaternion;
int need_send = 0;
pthread_mutex_t tcpsend_mutex = PTHREAD_MUTEX_INITIALIZER;

int statusreport_init(void) {
	int ret;
	thread_running = 1;
	ret = pthread_create(&thread_id, NULL, (void *)statusreport_run, NULL);
	if (ret != 0) {
		perror("Create pthread error!\n");
		return -1;
	}
	return 0;
}

void statusreport_release(void) {
	printf("Release pthread\n");
	thread_running = 0;
	pthread_join(thread_id, NULL);
}

void statusreport_run(void) {
    while(thread_running){
    	while (!need_send);
    	if (need_send) {
    		Quaternion sync_quaternion;
        	pthread_mutex_lock(&tcpsend_mutex);
    		sync_quaternion.w = sensor_quaternion.w;
    		sync_quaternion.x = sensor_quaternion.x;
    		sync_quaternion.y = sensor_quaternion.y;
    		sync_quaternion.z = sensor_quaternion.z;
    		need_send = 0;
        	pthread_mutex_unlock(&tcpsend_mutex);

        	unsigned char msg[56];
        	int c_i = 0;
        	unsigned char *pdata;
        	int i;

        	{/* Posture */
            	float a = 2 * acos(sync_quaternion.w);
            	a = a * 180 / M_PI;
            	while (a < 0) a += 360;
            	while (a >= 360) a -= 360;
            	pdata = ((unsigned char *)&a);
            	for (i = 0; i < 4; i ++) {
            		msg[c_i ++] = *pdata ++;
            	}
            	float x = 0;
            	if (a != 0) x = sync_quaternion.x / sin(acos(sync_quaternion.w));
            	pdata = ((unsigned char *)&x);
            	for (i = 0; i < 4; i ++) {
            		msg[c_i ++] = *pdata ++;
            	}
            	float y = 0;
            	if (a != 0) y = sync_quaternion.y / sin(acos(sync_quaternion.w));
            	pdata = ((unsigned char *)&y);
            	for (i = 0; i < 4; i ++) {
            		msg[c_i ++] = *pdata ++;
            	}
            	float z = 0;
            	if (a != 0) z = sync_quaternion.z / sin(acos(sync_quaternion.w));
            	pdata = ((unsigned char *)&z);
            	for (i = 0; i < 4; i ++) {
            		msg[c_i ++] = *pdata ++;
            	}
        	}

        	{/* Accel */
				float x = sensor_data.accel.x;
				pdata = ((unsigned char *)&x);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
				float y = sensor_data.accel.y;
				pdata = ((unsigned char *)&y);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
				float z = sensor_data.accel.z;
				pdata = ((unsigned char *)&z);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
        	}

        	{/* Magnet */
				float x = sensor_data.magnet.x;
				pdata = ((unsigned char *)&x);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
				float y = sensor_data.magnet.y;
				pdata = ((unsigned char *)&y);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
				float z = sensor_data.magnet.z;
				pdata = ((unsigned char *)&z);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
        	}

        	{/* Action */
				float la = left_angle;
				pdata = ((unsigned char *)&la);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
				float ra = right_angle;
				pdata = ((unsigned char *)&ra);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
				float lp = left_power;
				pdata = ((unsigned char *)&lp);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
				float rp = right_power;
				pdata = ((unsigned char *)&rp);
				for (i = 0; i < 4; i ++) {
					msg[c_i ++] = *pdata ++;
				}
        	}

        	tcpserver_send(msg, 56);
    	}
    }
}

void sync_posture(SensorData sd, Quaternion qua) {
	pthread_mutex_lock(&tcpsend_mutex);
	sensor_data = sd;
	sensor_quaternion = qua;
	pthread_mutex_unlock(&tcpsend_mutex);
}

void sync_action(float la, float ra, float lp, float rp) {
	pthread_mutex_lock(&tcpsend_mutex);
	left_angle = la;
	right_angle = ra;
	left_power = lp;
	right_power = rp;
	pthread_mutex_unlock(&tcpsend_mutex);
}

void sync_data_ready() {
	pthread_mutex_lock(&tcpsend_mutex);
	need_send = 1;
	pthread_mutex_unlock(&tcpsend_mutex);
}
