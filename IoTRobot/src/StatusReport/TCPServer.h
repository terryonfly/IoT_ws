/*
 * TCPServer.h
 *
 *  Created on: 2015年12月10日
 *      Author: terry
 */

#ifndef TCPSERVER_H_
#define TCPSERVER_H_

int tcpserver_init(void);

void tcpserver_release(void);

void tcpserver_run(void);

void tcpserver_data_decode(unsigned char *buf, size_t len);

void tcpserver_content_decode(unsigned char *buf, size_t len);

int tcpserver_send(unsigned char *buf, size_t len);

#endif /* TCPSERVER_H_ */
