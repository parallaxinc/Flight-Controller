
#ifndef __QUATIMU_H__
#define __QUATIMU_H__

// These are created this way because I need to be able to convert the code to addresses easily at runtime

#include "elev8-main.h"   // for RADIO struct

void QuatIMU_Start(void);


int QuatIMU_GetYaw(void);

float QuatIMU_GetFloatYaw(void);
float QuatIMU_GetFloatHeading(void);

int QuatIMU_GetThrustFactor(void);

int * QuatIMU_GetSensors(void);
float * QuatIMU_GetMatrix(void);

float * QuatIMU_GetQuaternion(void);

int QuatIMU_GetVerticalVelocityEstimate(void);
int QuatIMU_GetAltitudeEstimate(void);

int QuatIMU_GetPitchDifference(void);
int QuatIMU_GetRollDifference(void);
int QuatIMU_GetYawDifference(void);

void QuatIMU_ResetDesiredYaw(void);
void QuatIMU_ResetDesiredOrientation(void);

void QuatIMU_GetDesiredQ( float * dest );
void QuatIMU_GetDebugFloat( float * dest );

void QuatIMU_SetInitialAltitudeGuess( int altiMM );

void QuatIMU_SetRollCorrection( float * addr );
void QuatIMU_SetPitchCorrection( float * addr );


void QuatIMU_InitFunctions(void);
void QuatIMU_SetGyroZero( int x, int y, int z );
 

void QuatIMU_Update( int * packetAddr );

void QuatIMU_UpdateControls( RADIO * Radio , int ManualMode );


int QuatIMU_WaitForCompletion(void);


void QuatIMU_AdjustStreamPointers( int * p );

#endif
