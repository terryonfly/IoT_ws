#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include "mraa.h"

#include "TCPServer.h"
#include "I2CBus.h"
#include "MAX21100.h"
#include "PCA9685.h"

int running = 1;

float a_x = 0.0;
float a_y = 0.0;
float a_z = 0.0;

float y_a = 0.0;
float y_l = 10.0;

void sync_status() {
	unsigned char msg[12];
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
	tcpserver_send(msg, 12);
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
	i2cbus_init();
	max_init();
	pca_init();
	float pwm_persent = 0.0f;
	int bak = 0;
	while (running) {
		max_run();
		if (bak) {
			pwm_persent += 0.001f;
			if (pwm_persent >= 1.0f) bak = !bak;
		} else {
			pwm_persent -= 0.001f;
			if (pwm_persent <= 0.0f) bak = !bak;
		}
		pca_run(pwm_persent);
		usleep(10 * 1000);
	}
	pca_release();
	max_release();
	i2cbus_release();
	tcpserver_release();
	printf("==== robot end ====\n");
	return 0;
}
