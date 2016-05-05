
#ifndef __QUATIMU_H__
#define __QUATIMU_H__

/*
  This file is part of the ELEV-8 Flight Controller Firmware
  for Parallax part #80204, Revision A
  
  Copyright 2015 Parallax Incorporated

  ELEV-8 Flight Controller Firmware is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  ELEV-8 Flight Controller Firmware is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the ELEV-8 Flight Controller Firmware.  If not, see <http://www.gnu.org/licenses/>.
  
  Written by Jason Dorie
*/

#include "elev8-main.h"   // for RADIO struct


void QuatIMU_Start(void);
void QuatIMU_SetErrScaleMode( int IsStartup );

//int QuatIMU_GetYaw(void);
int QuatIMU_GetRoll(void);
int QuatIMU_GetPitch(void);

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

void QuatIMU_SetAutoLevelRates( float MaxRollPitch , float YawRate );
void QuatIMU_SetManualRates( float RollPitchRate, float YawRate );

void QuatIMU_InitFunctions(void);
void QuatIMU_SetGyroZero( int x, int y, int z );
 

void QuatIMU_Update( int * packetAddr );
void QuatIMU_UpdateControls( RADIO * Radio , bool ManualMode , bool AutoManual );

void QuatIMU_WaitForCompletion(void);

void QuatIMU_AdjustStreamPointers( unsigned char * p );

#endif
