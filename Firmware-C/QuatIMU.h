
#ifndef __QUATIMU_H__
#define __QUATIMU_H__

// These are created this way because I need to be able to convert the code to addresses easily at runtime

void QuatIMU_Start(void);


int QuatIMU_GetPitch(void);
int QuatIMU_GetRoll(void);
int QuatIMU_GetYaw(void);
int QuatIMU_GetThrustFactor(void);

int * QuatIMU_GetSensors(void);
float * QuatIMU_GetMatrix(void);
int * QuatIMU_GetFixedMatrix(void);
float * QuatIMU_GetQuaternion(void);

int QuatIMU_GetVerticalVelocityEstimate(void);
int QuatIMU_GetAltitudeEstimate(void);

void QuatIMU_SetInitialAltitudeGuess( int altiMM );

void QuatIMU_SetRollCorrection( float * addr );
void QuatIMU_SetPitchCorrection( float * addr );


void QuatIMU_InitFunctions(void);
void QuatIMU_SetGyroZero( int x, int y, int z );
 

void QuatIMU_Update( int * packetAddr );
int QuatIMU_WaitForCompletion(void);


void QuatIMU_AdjustStreamPointers( int * p );

#endif
