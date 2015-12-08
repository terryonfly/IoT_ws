#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> /* netdb is necessary for struct hostent */

#include "mraa.h"

#define PORT 4321 /* server port */
#define MAXDATASIZE 100

int main() {
	mraa_platform_t platform = mraa_get_platform_type();
	if (platform != MRAA_INTEL_EDISON_FAB_C) {
		fprintf(stderr, "Platform err\n");
		return -1;
	}
	printf("=== robot start ===\n");

	int sockfd, num; /* files descriptors */
	char buf[MAXDATASIZE]; /* buf will store received text */
	struct hostent *he; /* structure that will get information about remote host */
	struct sockaddr_in server;

	if ((he = gethostbyname("192.168.0.210")) == NULL) {
		printf("gethostbyname() error\n");
		return 1;
	}

	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		printf("socket() error\n");
		return 1;
	}
	bzero(&server, sizeof(server));
	server.sin_family = AF_INET;
	server.sin_port = htons(PORT);
	server.sin_addr = *((struct in_addr *)he->h_addr);
	while (connect(sockfd, (struct sockaddr *)&server, sizeof(server)) == -1) {
		printf("try connect() error\n");
		sleep(1);
	}
	printf("connect() successes\n");

	while (1) {
		char str[] = "horst\n";
		if ((num = send(sockfd, str, sizeof(str), 0)) == -1) {
			printf("send() error\n");
		}
		if ((num = recv(sockfd, buf, MAXDATASIZE, 0)) == -1) {
			printf("recv() error\n");
		}
		buf[num-1]='\0';
		printf("server message: %s\n",buf);
	}
	close(sockfd);

	printf("==== robot end ====\n");
	return 0;
}
