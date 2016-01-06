//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Maccel.yhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <math.h>

#include "MahonyAHRS.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float integralFBz = 0.0f,  integralFBx = 0.0f, integralFBy = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(
		Vector3f gyro,
		Vector3f accel,
		Vector3f magnet,
		double diff_time,
		Quaternion *qua) {
	float recipNorm;
    float qwqw, qwqz, qwqx, qwqy, qzqz, qzqx, qzqy, qxqx, qxqy, qyqy;
	float hx, hz, by, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((magnet.x == 0.0f) && (magnet.y == 0.0f) && (magnet.z == 0.0f)) {
		MahonyAHRSupdateIMU(gyro, accel, diff_time, qua);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((accel.x == 0.0f) && (accel.y == 0.0f) && (accel.z == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
		accel.x *= recipNorm;
		accel.y *= recipNorm;
		accel.z *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(magnet.x * magnet.x + magnet.y * magnet.y + magnet.z * magnet.z);
		magnet.x *= recipNorm;
		magnet.y *= recipNorm;
		magnet.z *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        qwqw = qua->w * qua->w;
        qwqz = qua->w * qua->z;
        qwqx = qua->w * qua->x;
        qwqy = qua->w * qua->y;
        qzqz = qua->z * qua->z;
        qzqx = qua->z * qua->x;
        qzqy = qua->z * qua->y;
        qxqx = qua->x * qua->x;
        qxqy = qua->x * qua->y;
        qyqy = qua->y * qua->y;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (magnet.z * (qzqx + qwqy) + magnet.x * (0.5f - qzqz - qyqy) + magnet.y * (qxqy - qwqz));
        hz = 2.0f * (magnet.z * (0.5f - qxqx - qyqy) + magnet.x * (qzqx - qwqy) + magnet.y * (qzqy + qwqx));
        by = 2.0f * (magnet.z * (qzqy - qwqx) + magnet.x * (qxqy + qwqz) + magnet.y * (0.5f - qzqz - qxqx));
        bz = sqrt(hz * hz + hx * hx);

		// Estimated direction of gravity and magnetic field
		halfvx = qwqz + qxqy;
		halfvy = qwqw - 0.5f + qyqy;
		halfvz = qzqy - qwqx;
        halfwx = bz * (qzqx - qwqy) + by * (qwqz + qxqy);
        halfwy = bz * (qwqx + qzqy) + by * (0.5f - qzqz - qxqx);
        halfwz = bz * (0.5f - qxqx - qyqy) + by * (qzqy - qwqx);
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (accel.y * halfvz - accel.z * halfvy) + (magnet.y * halfwz - magnet.z * halfwy);
		halfey = (accel.z * halfvx - accel.x * halfvz) + (magnet.z * halfwx - magnet.x * halfwz);
		halfez = (accel.x * halfvy - accel.y * halfvx) + (magnet.x * halfwy - magnet.y * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * diff_time;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * diff_time;
			integralFBz += twoKi * halfez * diff_time;
			gyro.x += integralFBx;	// apply integral feedback
			gyro.y += integralFBy;
			gyro.z += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gyro.x += twoKp * halfex;
		gyro.y += twoKp * halfey;
		gyro.z += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gyro.x *= (0.5f * diff_time);		// pre-multiply common factors
	gyro.y *= (0.5f * diff_time);
	gyro.z *= (0.5f * diff_time);
	qa = qua->w;
	qc = qua->x;
	qb = qua->z;
	qua->w += (-qc * gyro.x - qua->y * gyro.y - qb * gyro.z);
	qua->x += (qa * gyro.x + qua->y * gyro.z - qb * gyro.y);
	qua->y += (qa * gyro.y - qc * gyro.z + qb * gyro.x);
	qua->z += (qa * gyro.z + qc * gyro.y - qua->y * gyro.x);
	
	// Normalise quaternion
	recipNorm = invSqrt(qua->w * qua->w + qua->x * qua->x + qua->y * qua->y + qua->z * qua->z);
	qua->w *= recipNorm;
	qua->x *= recipNorm;
	qua->y *= recipNorm;
	qua->z *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(
		Vector3f gyro,
		Vector3f accel,
		double diff_time,
		Quaternion *qua) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((accel.x == 0.0f) && (accel.y == 0.0f) && (accel.z == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
		accel.x *= recipNorm;
		accel.y *= recipNorm;
		accel.z *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = qua->w * qua->z + qua->x * qua->y;
		halfvy = qua->w * qua->w - 0.5f + qua->y * qua->y;
		halfvz = qua->z * qua->y - qua->w * qua->x;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (accel.y * halfvz - accel.z * halfvy);
		halfey = (accel.z * halfvx - accel.x * halfvz);
		halfez = (accel.x * halfvy - accel.y * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * diff_time;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * diff_time;
			integralFBz += twoKi * halfez * diff_time;
			gyro.x += integralFBx;	// apply integral feedback
			gyro.y += integralFBy;
			gyro.z += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gyro.x += twoKp * halfex;
		gyro.y += twoKp * halfey;
		gyro.z += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gyro.x *= (0.5f * diff_time);		// pre-multiply common factors
	gyro.y *= (0.5f * diff_time);
	gyro.z *= (0.5f * diff_time);
	qa = qua->w;
	qc = qua->x;
	qb = qua->z;
	qua->w += (-qb * gyro.z - qc * gyro.x - qua->y * gyro.y);
	qua->x += (qa * gyro.x + qua->y * gyro.z - qb * gyro.y);
	qua->y += (qa * gyro.y - qc * gyro.z + qb * gyro.x);
	qua->z += (qa * gyro.z + qc * gyro.y - qua->y * gyro.x);
	
	// Normalise quaternion
	recipNorm = invSqrt(qua->w * qua->w + qua->x * qua->x + qua->y * qua->y + qua->z * qua->z);
	qua->w *= recipNorm;
	qua->x *= recipNorm;
	qua->y *= recipNorm;
	qua->z *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
