/*
 * PID.h
 *
 *  Created on: Jan 6, 2016
 *      Author: terry
 */

#ifndef CONTROLLER_PID_H_
#define CONTROLLER_PID_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <pthread.h>
#include <math.h>

typedef struct {
	double SetPoint;		// 设定目标Desired value
	double Proportion;		// 比例常数Proportional Const
	double Integral;		// 积分常数Integral Const
	double Derivative; 		// 微分常数Derivative Const

	double LastError; 		// Error[-1]
	double PrevError; 		// Error[-2]
	double SumError; 		// Sums of Errors
} PID;

void pid_init(PID *sPID, double p, double i, double d);

double pid_run(PID *sPID, double nextPoint, double setPoint);

#endif /* CONTROLLER_PID_H_ */
