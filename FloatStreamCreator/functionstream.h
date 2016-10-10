
#ifndef FUNCTIONSTREAM_H_
#define FUNCTIONSTREAM_H_

#include "elev8data.h"
#include <QQuaternion>

void QuatUpdate( SensorData & sens );
void UpdateControls_Manual(void);
void UpdateControls_AutoLevel(void);
void UpdateControls_ComputeOrientationChange(void);

void Quat_GetQuaternion( QQuaternion & cq );
void Quat_GetHeadingVect( QVector3D & hv );

void QuatIMU_InitVars(void);

#endif
