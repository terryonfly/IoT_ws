#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include "mraa.h"

#include "Common/I2CBus.h"
#include "Posture/MPU9250.h"
#include "Posture/Posture.h"
#include "Action/PCA9685.h"
#include "Controller/Controller.h"
#include "StatusReport/TCPServer.h"
#include "StatusReport/StatusReport.h"

int running = 1;

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
	pos_init();
	pca_init();
	ctl_init();
	while (running) {
		mpu_run();
		pos_run();
		ctl_run();
		pca_run();
		sync_data_ready();
		delay_for_ms(10);
	}
	ctl_release();
	pca_release();
	pos_release();
	mpu_release();
	i2cbus_release();
	statusreport_release();
	tcpserver_release();
	printf("==== robot end ====\n");
	return 0;
}
