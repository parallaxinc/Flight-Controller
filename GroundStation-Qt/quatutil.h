
#ifndef QUATUTIL_H_
#define QUATUTIL_H_

#include <QQuaternion>
#include <QMatrix3x3>

// This exists because apparently linux users may not have Q 5.5,
// as it may not be in the latest package.  The Q.toRotationMatrix()
// function only exists as of Qt 5.5, so I've written my own.

QMatrix3x3 QuatToMatrix( QQuaternion & Q );


#endif
