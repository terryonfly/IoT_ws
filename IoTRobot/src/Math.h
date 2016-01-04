/*
 * Math.h
 *
 *  Created on: 2015年12月28日
 *      Author: terry
 */

#ifndef MATH_H_
#define MATH_H_

typedef struct {
	float 		w;
	float 		x;
	float 		y;
	float 		z;
} Quaternion;

typedef struct {
	float 		x;
	float 		y;
	float 		z;
} Vector3f;

typedef struct {
	int16_t 	x;
	int16_t 	y;
	int16_t 	z;
} Vector3s;

void quaternion_to_euler(Quaternion qua, Vector3f *euler);

void euler_to_quaternion(Vector3f euler, Quaternion *qua);

Quaternion quaternion_multiply(Quaternion a_qua, Quaternion b_qua);

#endif /* MATH_H_ */
