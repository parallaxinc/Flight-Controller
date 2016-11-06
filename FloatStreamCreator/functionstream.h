
#ifndef FUNCTIONSTREAM_H_
#define FUNCTIONSTREAM_H_

#include "elev8data.h"
#include <QQuaternion>

void QuatUpdate( SensorData & sens );
//void UpdateControls_Manual(void);
//void UpdateControls_AutoLevel(void);
//void UpdateControls_ComputeOrientationChange(void);

void Quat_GetQuaternion( QQuaternion & cq );
void Quat_GetHeadingVect( QVector3D & hv );

int  Quat_GetAltitudeEstimate(void);

void QuatIMU_InitVars(void);

void QuatIMU_Mag_InitCalibrate(void);
void QuatIMU_Mag_AddCalibratePoint(void);
void QuatIMU_Mag_ComputeCalibrate_SetupIteration(void);
void QuatIMU_Mag_ComputeCalibrate_IterationStep(void);

#endif
