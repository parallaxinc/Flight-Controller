#ifndef _AHRS_H__
#define _AHRS_H__

#include <QQuaternion>
#include <QMatrix3x3>

#include "elev8data.h"


// AHRS - Attitude and Heading Reference System
// IMU - Inertial Measurement Unit

// Both of these terms are used to describe code that maintains an orientation estimate.
// The IMU is simpler, only using gyro and accelerometer readings, while the AHRS also
// uses the magnetometer readings, giving a magnetic north relative heading


class AHRS
{
public:
	QQuaternion quat;	// current orientation estimate
	float err[3];		// error corrections for the 3 axis
	bool foundZero;
	float zx, zy, zz;
	int zeroSamples;

	AHRS();

	void Update( SensorData & sens , float TimeDelta , bool IgnoreMag );
};

#endif
