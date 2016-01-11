/*
 * TCPServer.c
 *
 *  Created on: 2015年12月10日
 *      Author: terry
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "TCPServer.h"

#define PORT 7777

#define BACKLOG 1
#define MAXRECVLEN 1024

int connectfd = -1;

pthread_t thread_id;
int thread_running;

#define CMD_DATA_CONTENT 0x80
#define CMD_DATA_HEADER 0x81
#define CMD_DATA_FOOTER 0x82
#define MAX_REV_CONTENT_LEN 1024
unsigned char rev_content[MAX_REV_CONTENT_LEN];
int rev_content_index;
int rev_is_cmd = 1;
unsigned char rev_current_cmd;

#define MAX_SEND_DATA_LEN 1024
unsigned char send_data[MAX_SEND_DATA_LEN];

float ctrl_x = 0.0;
float ctrl_y = 0.0;
float ctrl_z = 0.0;
float ctrl_w = 0.0;

int tcpserver_init(void) {
	int ret;
	thread_running = 1;
	ret = pthread_create(&thread_id, NULL, (void *)tcpserver_run, NULL);
	if (ret != 0) {
		perror("Create pthread error!\n");
		return -1;
	}
	return 0;
}

void tcpserver_release(void) {
	printf("Release pthread\n");
	thread_running = 0;
	pthread_join(thread_id, NULL);
}

void tcpserver_run(void) {
    unsigned char buf[MAXRECVLEN];
    int listenfd;  /* socket descriptors */
    struct sockaddr_in server; /* server's address information */
    struct sockaddr_in client; /* client's address information */
    socklen_t addrlen;
    /* Create TCP socket */
    if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        /* handle exception */
        perror("socket() error. Failed to initiate a socket");
        return;
    }

    /* set socket option */
    int opt = SO_REUSEADDR;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    bzero(&server, sizeof(server));

    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listenfd, (struct sockaddr *)&server, sizeof(server)) == -1) {
        /* handle exception */
        perror("Bind() error.");
        return;
    }

    if (listen(listenfd, BACKLOG) == -1) {
        perror("listen() error.\n");
        return;
    }

    addrlen = sizeof(client);
    while(thread_running){
        if ((connectfd = accept(listenfd, (struct sockaddr *)&client, &addrlen)) == -1) {
            perror("accept() error. \n");
            return;
        }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        printf("You got a connection from client's ip %s, port %d at time %ld.%ld\n", inet_ntoa(client.sin_addr), htons(client.sin_port), tv.tv_sec, tv.tv_usec);

        int read_len = -1;
        while(thread_running)
        {
        	read_len = recv(connectfd, buf, MAXRECVLEN, 0);
            if (read_len > 0) {
            	tcpserver_data_decode(buf, read_len);
            } else {
                close(connectfd);
                connectfd = -1;
                break;
            }
        }
    }
    close(listenfd); /* close listenfd */
}

void tcpserver_data_decode(unsigned char *buf, size_t len) {
    int i;
    for (i = 0; i < len; i ++) {
    	if (rev_is_cmd) {
    		if (buf[i] == CMD_DATA_CONTENT || buf[i] == CMD_DATA_HEADER || buf[i] == CMD_DATA_FOOTER) {
    			rev_current_cmd = buf[i];
    			rev_is_cmd = 0;
    		}
    	} else {
    		switch (rev_current_cmd) {
    		case CMD_DATA_CONTENT:
    			rev_content[rev_content_index] = buf[i];
    			rev_content_index ++;
    			if (rev_content_index > MAX_REV_CONTENT_LEN) rev_content_index = 0;
    			break;
    		case CMD_DATA_HEADER:
    			// buf[i] is header data
    			rev_content_index = 0;
    			break;
    		case CMD_DATA_FOOTER:
    			// buf[i] is footer data
//    			for (k = 0; k < rev_content_index; k ++)
//    				printf("%02x ", rev_content[k]);
//    			printf("\n");
                tcpserver_content_decode(rev_content, rev_content_index);

    			rev_content_index = 0;
    			break;

    		default:
    			break;
    		}
			rev_is_cmd = 1;
    	}
    }
}

void tcpserver_content_decode(unsigned char *buf, size_t len)
{
    unsigned char i;
    unsigned char* px = buf;
    void *pf;

    pf = &ctrl_x;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &ctrl_y;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &ctrl_z;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    pf = &ctrl_w;
    for(i = 0; i < 4; i ++) {
        *((unsigned char*)pf+i) = *(px++);
    }

    printf("PID : %10.6f %10.6f %10.6f \n", ctrl_x / 1000.0, ctrl_y / 1000.0, ctrl_z / 1000.0);
}

int tcpserver_send(unsigned char *buf, size_t len)
{
	if (connectfd == -1) return -1;
	if (len > MAX_SEND_DATA_LEN / 2) return -1;
	int set_i = 0;
	int i, send_len;
	// Header
	send_data[set_i] = CMD_DATA_HEADER;
	set_i ++;
	send_data[set_i] = 0x01;
	set_i ++;
	// Content
	for (i = 0; i < len; i ++) {
		send_data[set_i] = CMD_DATA_CONTENT;
		set_i ++;
		send_data[set_i] = buf[i];
		set_i ++;
	}
	// Footer
	send_data[set_i] = CMD_DATA_FOOTER;
	set_i ++;
	send_data[set_i] = 0x82;
	set_i ++;
	if ((send_len = send(connectfd, send_data, set_i, 0)) == -1) {
		printf("send_len = %d\n", send_len);
		perror("send failure\n");
		return -1;
	}
	return 0;
}

