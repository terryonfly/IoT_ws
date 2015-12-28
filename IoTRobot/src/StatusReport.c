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

        	unsigned char msg[16];
        	int c_i = 0;
        	unsigned char *pdata;
        	int i;

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
        	tcpserver_send(msg, 16);
    	}
    }
}

void sync_posture(Quaternion qua) {
	pthread_mutex_lock(&tcpsend_mutex);
	sensor_quaternion = qua;
	need_send = 1;
	pthread_mutex_unlock(&tcpsend_mutex);
}
