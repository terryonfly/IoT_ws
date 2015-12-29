//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include "../Math.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(
		Vector3f gyro,
		Vector3f accel,
		Vector3f magnet,
		double diff_time,
		Quaternion *qua);

void MahonyAHRSupdateIMU(
		Vector3f gyro,
		Vector3f accel,
		double diff_time,
		Quaternion *qua);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
