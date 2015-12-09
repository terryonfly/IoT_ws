#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

#include "mraa.h"

#include "TCPServer.h"

int running;

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
	tcpserver_init();
	running = 1;
	int i;
	while (running) {
		unsigned char msg[12];
		for (i = 0; i < 12; i ++) {
			msg[i] = i;
		}
		tcpserver_send(msg, 12);
		usleep(10 * 1000);
	}
	tcpserver_release();
	printf("==== robot end ====\n");
	return 0;
}
