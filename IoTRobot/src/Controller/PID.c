/*
 * PID.c
 *
 * Created on: Jan 6, 2016
 * Author: terry
 */

#include "PID.h"

void pid_init(PID *sPID, double p, double i, double d) {
	memset (sPID, 0, sizeof(PID));
	sPID->Proportion = p; // Set PID Coefficients
	sPID->Integral = i;
	sPID->Derivative = d;
}

double pid_run(PID *sPID, double nextPoint, double setPoint) {
	sPID->SetPoint = setPoint;
	double dError, Error;
	Error = sPID->SetPoint - nextPoint; // 偏差
	sPID->SumError += Error; // 积分
	dError = sPID->LastError - sPID->PrevError; // 当前微分
	sPID->PrevError = sPID->LastError;
	sPID->LastError = Error;
	return (sPID->Proportion * Error // 比例项
			+ sPID->Integral * sPID->SumError // 积分项
			+ sPID->Derivative * dError); // 微分项
}

//// DEMO
//void pid_demo(void)
//{
//	PID sPID; // PID Control Structure
//	PIDInit(&sPID, 0.5, 0.5, 0.0); // Initialize Structure
//	double rOut; // PID Response (Output)
//	double rIn; // PID Feedback (Input)
//	while { // Mock Up of PID Processing
//		rIn = sensor(); // Read Input
//		rOut = PIDCalc(&sPID, rIn, 100.0); // Perform PID Interation
//		actuator(rOut); // Effect Needed Changes
//	}
//}
