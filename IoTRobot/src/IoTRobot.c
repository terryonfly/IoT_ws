#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include "mraa.h"

#include "TCPServer.h"
#include "StatusReport.h"
#include "I2CBus.h"
#include "MPU9250.h"
#include "PCA9685.h"
#include "Posture.h"

int running = 1;

float a_x = 0.0;
float a_y = 0.0;
float a_z = 0.0;

float y_a = 0.0;
float y_l = 10.0;

void sync_status() {
	unsigned char msg[24];
	int c_i = 0;
	unsigned char *pdata;
	int i;

	y_a += 3;
	if (y_a > 360.0) y_a = 0.0;
	a_x = y_l * cos(y_a * M_PI / 180.0);
	a_z = y_l * sin(y_a * M_PI / 180.0);

	float angle_x = a_x;
	pdata = ((unsigned char *)&angle_x);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	a_y += 0.1;
	if (a_y > 360.0) a_y = 0.0;

	float angle_y = a_y;
	pdata = ((unsigned char *)&angle_y);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	float angle_z = a_z;
	pdata = ((unsigned char *)&angle_z);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	float xangle_z = a_z;
	pdata = ((unsigned char *)&xangle_z);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	float yangle_z = a_z;
	pdata = ((unsigned char *)&yangle_z);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}

	float zangle_z = a_z;
	pdata = ((unsigned char *)&zangle_z);
	for (i = 0; i < 4; i ++) {
		msg[c_i ++] = *pdata ++;
	}
	get_diff_time();
	tcpserver_send(msg, 24);

	delay_for_ms(10);
}

void cs(int n) {
    printf("now dowm!\n");
    running = 0;
}

int main() {
	mraa_platform_t platform = mraa_get_platform_type();
	if (platform != MRAA_INTEL_EDISON_FAB_C) {
		perror("Platform err\n");
		return -1;
	}
	printf("=== robot start ===\n");
    signal(SIGINT, cs);  //ctrl+c
    signal(SIGTERM, cs);  //kill
	mraa_init();
	tcpserver_init();
	statusreport_init();
	i2cbus_init();
	mpu_init();
	pca_init();
	posture_init();
	pca_run(0.5, 0.5, 0.0, 0.0);
	sleep(1);
	float x = 0.0;
	while (running) {
		x += 0.0005;
		if (x > 0.20) x = 0.0;
		mpu_run();
		pca_run(0.5, 0.5, 0.0, x);
		sync_data_ready();
	}
	posture_release();
	pca_release();
	mpu_release();
	i2cbus_release();
	statusreport_release();
	tcpserver_release();
	printf("==== robot end ====\n");
	return 0;
}
