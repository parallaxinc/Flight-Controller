
#include "quatutil.h"

// This exists because apparently linux users may not have Q 5.5,
// as it may not be in the latest package.  The Q.toRotationMatrix()
// function only exists as of Qt 5.5, so I've written my own.

QMatrix3x3 QuatToMatrix( QQuaternion & Q )
{
QMatrix3x3 m;

	float qx2 = Q.x() * Q.x() * 2.0f;
	float qy2 = Q.y() * Q.y() * 2.0f;
	float qz2 = Q.z() * Q.z() * 2.0f;
	float qxqy2 = Q.x() * Q.y() * 2.0f;
	float qxqz2 = Q.x() * Q.z() * 2.0f;
	float qxqw2 = Q.x() * Q.scalar() * 2.0f;
	float qyqz2 = Q.y() * Q.z() * 2.0f;
	float qyqw2 = Q.y() * Q.scalar() * 2.0f;
	float qzqw2 = Q.z() * Q.scalar() * 2.0f;
	m(0, 0) = 1.0f - qy2 - qz2;
	m(0, 1) = qxqy2 - qzqw2;
	m(0, 2) = qxqz2 + qyqw2;
	m(1, 0) = qxqy2 + qzqw2;
	m(1, 1) = 1.0f - qx2 - qz2;
	m(1, 2) = qyqz2 - qxqw2;
	m(2, 0) = qxqz2 - qyqw2;
	m(2, 1) = qyqz2 + qxqw2;
	m(2, 2) = 1.0f - qx2 - qy2;
	return m;
}
