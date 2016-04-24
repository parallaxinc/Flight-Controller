
#include <math.h>
#include "ahrs.h"

const float RadToDeg = (180.0f / 3.141592654f);                         //Degrees per Radian
const float GyroToDeg =  (1000.0f / 70.0f);                             //Gyro units per degree @ 2000 deg/sec sens = 70 mdps/bit
const float GyroScale = 1.0f / (GyroToDeg * RadToDeg);


AHRS::AHRS()
{
	foundZero = false;
	zeroSamples = 0;
	zx = zy = zz = 0.0f;

	quat = QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
	err[0] = err[1] = err[2] = 0.0f;
}

void AHRS::Update( SensorData & sens , float TimeDelta , bool IgnoreMag )
{
	if( foundZero == false )
	{
		zx += sens.GyroX;
		zy += sens.GyroY;
		zz += sens.GyroZ;
		zeroSamples++;

		if( zeroSamples == 16 )
		{
			zx *= 1.0f/16.0f;
			zy *= 1.0f/16.0f;
			zz *= 1.0f/16.0f;
			foundZero = true;
		}
		return;
	}

	float gx =  (sens.GyroX - zx) * GyroScale;
	float gz = -(sens.GyroY - zy) * GyroScale;
	float gy = -(sens.GyroZ - zz) * GyroScale;

	float ax = -sens.AccelX;
	float az = sens.AccelY;
	float ay = sens.AccelZ;

	float mx = sens.MagX;
	float mz = sens.MagY;
	float my = sens.MagZ;

	float qw = quat.scalar(), qx = quat.x(), qy = quat.y(), qz = quat.z();   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	static float Kp = 1.0f;
	static float Ki = 0.0f;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return;	// handle NaN
	norm = 1.0f / norm;			// use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Auxiliary variables to avoid repeated arithmetic
	float qw2 = qw * qw;
	float qx2 = qx * qx;
	float qy2 = qy * qy;
	float qz2 = qz * qz;
	float qwx = qw * qx;
	float qwy = qw * qy;
	float qxz = qx * qz;
	float qyz = qy * qz;
	float qwz = qw * qz;
	float qxy = qx * qy;

	// Estimated direction of gravity
	vx =        2.0f * (qxy + qwz);
	vy = 1.0f - 2.0f * (qx2 + qz2);
	vz =        2.0f * (qyz - qwx);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);

	if( IgnoreMag == false )
	{
		// Normalise magnetometer measurement
		norm = sqrtf(mx * mx + my * my + mz * mz);
		if (norm == 0.0f) return;	// handle NaN
		norm = 1.0f / norm;			// use reciprocal for division

		mx *= norm;
		my *= norm;
		mz *= norm;

		/*
		// Estimated direction of magnetic field

		// This code computes vMag rotated into the earth frame:  hMag = Q * vMag * Q(conjugate)
		// The result then has the Y component removed so the vector lies in the flat XZ plane


		// Reference direction of Earth's magnetic field
		hx = mx * 2.0f * (0.5f - qy2 - qz2) + my * 2.0f *        (qxy - qwz) + mz * 2.0f * (qxz + qwy);
		hy = mx * 2.0f *        (qxy + qwz) + my * 2.0f * (0.5f - qx2 - qz2) + mz * 2.0f * (qyz - qwx);
		bz = mx * 2.0f *        (qxz - qwy) + my * 2.0f *        (qyz + qwx) + mz * 2.0f * (0.5f - qx2 - qy2);

		bx = sqrtf((hx * hx) + (hy * hy));

		// mag field - this needs to be corrected for my reference frame (I think)
		wy = 2.0f * bx * (0.5f - qy2 - qz2) + 2.0f * bz * (qxz - qwy);
		wx = 2.0f * bx * (qxy - qwz) + 2.0f * bz * (qwx + qyz);
		wz = 2.0f * bx * (qwy + qxz) + 2.0f * bz * (0.5f - qx2 - qy2);

		// Error is the sum of:
		// - cross product between estimated gravity and measured gravity
		// - cross product between estimated mag and measured mag
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
		*/


		/*
		// I don't understand why this doesn't work, but it doesn't
		// It seems to align the heading of the quaternion such that it's
		// always zero, which is the opposite of what I expect - I want it
		// to align the Z axis of the matrix with the measured magnetic field

		// Test unit is NOT CALIBRATED - might be causing issues just because of that
		*/

		// Compute "left" vector from orientation estimate
		wx = 1.0f - 2.0f * (qy2 + qz2);
		wy =        2.0f * (qxy - qwz);
		wz =        2.0f * (qxz + qwy);

		// compute accel cross mag to get measured "left" vector, then normalize it
		float mlx = ay * mz - az * my;
		float mly = az * mx - ax * mz;
		float mlz = ax * my - ay * mx;

		norm = sqrtf(mlx * mlx + mly * mly + mlz * mlz);
		if( norm > 0.001f )
		{
			norm = 1.0f / norm;
			mlx *= norm;
			mly *= norm;
			mlz *= norm;

			ex += (mly * wz - mlz * wy);	// mag correction
			ey += (mlz * wx - mlx * wz);
			ez += (mlx * wy - mly * wx);
		}
	}


	// Apply feedback terms to gyro rates
	gx += Kp * ex;
	gy += Kp * ey;
	gz += Kp * ez;

	// Integrate rate of change of quaternion
	pa = qx;
	pb = qy;
	pc = qz;
	qw = qw + (-qx * gx - qy * gy - qz * gz) * (0.5f * TimeDelta);
	qx = pa + ( qw * gx + pb * gz - pc * gy) * (0.5f * TimeDelta);
	qy = pb + ( qw * gy - pa * gz + pc * gx) * (0.5f * TimeDelta);
	qz = pc + ( qw * gz + pa * gy - pb * gx) * (0.5f * TimeDelta);

	// Normalise quaternion
	norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
	norm = 1.0f / norm;

	quat.setScalar( qw * norm );
	quat.setX( qx * norm );
	quat.setY( qy * norm );
	quat.setZ( qz * norm );
}
