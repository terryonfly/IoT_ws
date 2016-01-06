/*
 * Math.c
 *
 *  Created on: 2015年12月28日
 *      Author: terry
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Math.h"

void quaternion_to_euler(Quaternion qua, Vector3f *euler) {
	euler->x = atan2(2 * qua.w * qua.x + 2 * qua.y * qua.z,
			1 - 2 * qua.x * qua.x - 2 * qua.y * qua.y);
	euler->y = asin(2 * qua.w * qua.y - 2 * qua.z * qua.x);
	euler->z = atan2(2 * qua.w * qua.z + 2 * qua.x * qua.y,
			1 - 2 * qua.y * qua.y - 2 * qua.z * qua.z);
}

void euler_to_quaternion(Vector3f euler, Quaternion *qua) {
	qua->w = cos(euler.x / 2) * cos(euler.y / 2) * cos(euler.z / 2) +
			sin(euler.x / 2) * sin(euler.y / 2) * sin(euler.z / 2);
	qua->x = sin(euler.x / 2) * cos(euler.y / 2) * cos(euler.z / 2) -
			cos(euler.x / 2) * sin(euler.y / 2) * sin(euler.z / 2);
	qua->y = cos(euler.x / 2) * sin(euler.y / 2) * cos(euler.z / 2) +
			sin(euler.x / 2) * cos(euler.y / 2) * sin(euler.z / 2);
	qua->z = cos(euler.x / 2) * cos(euler.y / 2) * sin(euler.z / 2) -
			sin(euler.x / 2) * sin(euler.y / 2) * cos(euler.z / 2);
}

Quaternion quaternion_multiply(Quaternion a_qua, Quaternion b_qua) {
	Quaternion r_qua;
	r_qua.w = a_qua.w * b_qua.w - a_qua.x * b_qua.x - a_qua.y * b_qua.y - a_qua.z * b_qua.z;
	r_qua.x = a_qua.w * b_qua.x + a_qua.x * b_qua.w + a_qua.y * b_qua.z - a_qua.z * b_qua.y;
	r_qua.y = a_qua.w * b_qua.y - a_qua.x * b_qua.z + a_qua.y * b_qua.w + a_qua.z * b_qua.x;
	r_qua.z = a_qua.w * b_qua.z + a_qua.x * b_qua.y - a_qua.y * b_qua.x + a_qua.z * b_qua.w;
	return r_qua;
}
