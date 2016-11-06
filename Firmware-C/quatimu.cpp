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

#include <propeller.h>

#include "constants.h"
#include "f32.h"
#include "quatimu.h"


#define RadToDeg (180.0 / 3.141592654)                         //Degrees per Radian
#define GyroToDeg  (1000.0 / 70.0)                             //Gyro units per degree @ 2000 deg/sec sens = 70 mdps/bit
#define AccToG  (float)(Const_OneG)                            //Accelerometer per G @ 8g sensitivity = ~0.24414 mg/bit 
#define GyroScale  (GyroToDeg * RadToDeg * (float)Const_UpdateRate)


const float Startup_ErrScale = 1.0f/32.0f;      // Converge quickly on startup
const float Running_ErrScale = 1.0f/512.0f;     // Converge more slowly once up & running


static int  zx, zy, zz;                          // Gyro zero readings


// Working variables for the IMU code, in a struct so the compiler doesn't screw up the order,
// or remove some of them due to overly aggressive (IE wrong) optimization.

enum IMU_VarLabels {
    #include "QuatIMU_Vars.inc"

    IMU_VARS_SIZE                    // This entry MUST be last so we can compute the array size required
};


// The type doesn't matter much here - everything is a 32 bit value.  This is mostly an array of floats, but some are integer.
// Using a union of both allows me to freely use any slot in the array as either float or integer.
// The whole reason this struct exists is that the GCC linker can't down-cast a pointer to a 16-bit value, and I don't want to
// waste an extra 16 bits per entry in the command arrays.  I store an index into the IMU_VARS array at compile time,
// and then the UpdateStreamPointers function converts it to a real 16-bit memory address at runtime.

static union {
  float IMU_VARS[ IMU_VARS_SIZE ];
  int   INT_VARS[ IMU_VARS_SIZE ];
};


#define PI  3.141592654


void QuatIMU_Start(void)
{
  memset( &IMU_VARS[0], 0, sizeof(IMU_VARS) );

  IMU_VARS[qx] = 0.0f;
  IMU_VARS[qy] = 0.0f;
  IMU_VARS[qz] = 0.0f;
  IMU_VARS[qw] = 1.0f;

  IMU_VARS[accRollCorrSin] = 0.0f;                       // used to correct the accelerometer vector angle offset
  IMU_VARS[accRollCorrCos] = 1.0f;
  IMU_VARS[accPitchCorrSin] = 0.0f;
  IMU_VARS[accPitchCorrCos] = 1.0f;

  //Various constants used by the float math engine - Every command in the instruction stream reads two
  //arguments from memory using memory addresses, so the values actually need to exist somewhere

  IMU_VARS[const_GyroScale]         =    1.0f / (float)GyroScale;    
  IMU_VARS[const_NegGyroScale]      =   -1.0f / (float)GyroScale;

  #include "QuatIMU_Var_init.inc"

  IMU_VARS[const_epsilon]           =    0.00000001f;     //Added to vector length value before inverting (1/X) to insure no divide-by-zero problems

  IMU_VARS[const_AccErrScale]       =    Startup_ErrScale;  //How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
//IMU_VARS[const_MagErrScale]       =    Startup_ErrScale;  //How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
  IMU_VARS[const_AccScale]          =    1.0f/(float)AccToG;//Conversion factor from accel units to G's
  IMU_VARS[const_G_mm_PerSec]       =    9.80665f * 1000.0f;  // gravity in mm/sec^2
  IMU_VARS[const_UpdateScale]       =    1.0f / (float)Const_UpdateRate;    //Convert units/sec to units/update

  IMU_VARS[const_velAccScale]       =    0.9995f;     // was 0.9995     - Used to generate the vertical velocity estimate
  IMU_VARS[const_velAltiScale]      =    0.0005f;     // was 0.0005

  IMU_VARS[const_velAccTrust]       =    0.9993f;      // was 0.9990    - used to generate the absolute altitude estimate
  IMU_VARS[const_velAltiTrust]      =    0.0007f;      // was 0.0010

  IMU_VARS[const_YawRateScale]      =    ((120.0f / 200.0f) / 1024.0f) * (PI/180.f) * 0.5f; // 120 deg/sec / UpdateRate * Deg2Rad * HalfAngle
  IMU_VARS[const_AutoBankScale]     =    (45.0f / 1024.0f) * (PI/180.0f) * 0.5f;

  IMU_VARS[const_ManualYawScale]    =   ((180.0f / 200.0f) / 1024.0f) * (PI/180.f) * 0.5f; // 120 deg/sec / UpdateRate * Deg2Rad * HalfAngle
  IMU_VARS[const_ManualBankScale]   =   ((120.0f / 200.0f) / 1024.0f) * (PI/180.f) * 0.5f; // 120 deg/sec / UpdateRate * Deg2Rad * HalfAngle
  
  IMU_VARS[const_TwoPI]             =    2.0f * PI;

  IMU_VARS[const_outAngleScale]     =    65536.0f / PI;
  IMU_VARS[const_outNegAngleScale]  =   -65536.0f / PI;
}


void QuatIMU_SetErrScaleMode( int IsStartup )
{
  if( IsStartup ) {
    IMU_VARS[const_AccErrScale] = Startup_ErrScale;
    //IMU_VARS[const_MagErrScale] = Startup_ErrScale;
  }
  else {
    IMU_VARS[const_AccErrScale] = Running_ErrScale;
    //IMU_VARS[const_MagErrScale] = Running_ErrScale;
  }
}



//int QuatIMU_GetYaw(void) {
//  return INT_VARS[ Yaw ];
//}

int QuatIMU_GetRoll(void) {
  return INT_VARS[ Roll ];
}

int QuatIMU_GetPitch(void) {
  return INT_VARS[ Pitch ];
}

int QuatIMU_GetThrustFactor(void) {
  return INT_VARS[ ThrustFactor ];
}

int * QuatIMU_GetMag(void) {
  return &INT_VARS[mx];
}

float * QuatIMU_GetMatrix(void) {
  return &IMU_VARS[m00];
}  

float * QuatIMU_GetQuaternion(void) {
  return &IMU_VARS[qx];
}

int QuatIMU_GetVerticalVelocityEstimate(void) {
  return INT_VARS[ VelocityEstMM ];
}

int QuatIMU_GetAltitudeEstimate(void) {
  return INT_VARS[ AltitudeEstMM ];
}

void QuatIMU_SetInitialAltitudeGuess( int altiMM )
{
  IMU_VARS[altitudeEstimate] = F32::FFloat(altiMM);
}

int QuatIMU_GetPitchDifference(void) {
  return INT_VARS[PitchDiff];
}

int QuatIMU_GetRollDifference(void) {
  return INT_VARS[RollDiff];
}

int QuatIMU_GetYawDifference(void) {
  return INT_VARS[YawDiff];
}


void QuatIMU_SetAutoLevelRates( float MaxRollPitch , float YawRate )
{
  IMU_VARS[const_AutoBankScale] = MaxRollPitch; // (45.0f / 1024.0f) * (PI/180.0f) * 0.5f;
  IMU_VARS[const_YawRateScale]  = YawRate;      // ((120.0f / 200.0f) / 1024.0f) * (PI/180.f) * 0.5f;
}

void QuatIMU_SetManualRates( float RollPitchRate, float YawRate )
{
  IMU_VARS[const_ManualBankScale] = RollPitchRate;
  IMU_VARS[const_ManualYawScale] = YawRate;
}


void QuatIMU_ResetDesiredYaw(void)
{
  IMU_VARS[Heading] = IMU_VARS[HalfYaw];   // Desired value = current computed value half-angle
}


void QuatIMU_ResetDesiredOrientation(void)
{
  IMU_VARS[cqw] = IMU_VARS[qw];
  IMU_VARS[cqx] = IMU_VARS[qx];
  IMU_VARS[cqy] = IMU_VARS[qy];
  IMU_VARS[cqz] = IMU_VARS[qz];
}


float QuatIMU_GetFloatYaw(void)
{
  return IMU_VARS[FloatYaw];
}

float QuatIMU_GetFloatHeading(void)
{
  return IMU_VARS[Heading];
}


void QuatIMU_GetDesiredQ( float * dest )
{
  dest[0] = IMU_VARS[cqx];
  dest[1] = IMU_VARS[cqy];
  dest[2] = IMU_VARS[cqz];
  dest[3] = IMU_VARS[cqw];
}

void QuatIMU_GetDebugFloat( float * dest )
{
  dest[0] = IMU_VARS[DebugFloat];
}

void QuatIMU_SetRollCorrection( float * addr )
{
  IMU_VARS[accRollCorrSin] = addr[0];
  IMU_VARS[accRollCorrCos] = addr[1];
}

void QuatIMU_SetPitchCorrection( float * addr )
{
  IMU_VARS[accPitchCorrSin] = addr[0];
  IMU_VARS[accPitchCorrCos] = addr[1];
}


void QuatIMU_SetGyroZero( int x, int y, int z )
{
  zx = x;
  zy = y;
  zz = z;
}


/*
  'Quaternion update as C code

  'This code computes the current incremental rotation from the gyro in radians
  'The rotation is converted into quaternion form, multiplied by the inverse of
  'the current orientation.  The rotations are added, and the result is normalized.

  'The math is functionally idential to what happens in UpdateControls_Manual.

  'For a good primer on quaternion math, see here:
  '  http://mathinfo.univ-reims.fr/IMG/pdf/Rotating_Objects_Using_Quaternions.pdf

  {
  rx = gx / GyroScale + errCorrX
  ry = gy / GyroScale + errCorrY
  rz = gz / GyroScale + errCorrZ

  rmag = sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) / 2.0 

  cosr = Cos(rMag)
  sinr = Sin(rMag) / rMag

  qdot.w = -(rx * qx + ry * qy + rz * qz) / 2.0
  qdot.x =  (rx * qw + rz * qy - ry * qz) / 2.0
  qdot.y =  (ry * qw - rz * qx + rx * qz) / 2.0
  qdot.z =  (rz * qw + ry * qx - rx * qy) / 2.0

  qw = cosr * qw + sinr * qdot.w
  qx = cosr * qx + sinr * qdot.x
  qy = cosr * qy + sinr * qdot.y
  qz = cosr * qz + sinr * qdot.z
   
  q = q.Normalize()
  }
*/


unsigned char QuatUpdateCommands[] = {
  #include "QuatIMU_QuatUpdate.inc"
};

unsigned char UpdateControls_Manual[] = {
  #include "QuatIMU_UpdateControls_Manual.inc"
};

unsigned char UpdateControlQuaternion_AutoLevel[] = {
  #include "QuatIMU_UpdateControls_AutoLevel.inc"
};

unsigned char UpdateControls_ComputeOrientationChange[] = {
  #include "QuatIMU_UpdateControls_ComputeOrientationChange.inc"
};


unsigned char QuatIMU_Mag_InitCalibrate[] = {
  #include "QuatIMU_Mag_InitCalibrate.inc"
};
  
unsigned char QuatIMU_Mag_AddCalibratePoint[] = {
  #include "QuatIMU_Mag_AddCalibratePoint.inc"
};
  
unsigned char QuatIMU_Mag_ComputeCalibrate_SetupIteration[] = {
  #include "QuatIMU_Mag_ComputeCalibrate_SetupIteration.inc"
};
  
unsigned char QuatIMU_Mag_ComputeCalibrate_IterationStep[] = {
  #include "QuatIMU_Mag_ComputeCalibrate_IterationStep.inc"
};
  

// was 24124, now 24908 with Compass calibrate functions
// Total runtime = 27696 (32768=32kb, so approx 5Kb remain)

void QuatIMU_Update( int * packetAddr )
{
  memcpy( &IMU_VARS[gx], packetAddr, 11 * sizeof(int) );
  INT_VARS[gx] -= zx;
  INT_VARS[gy] -= zy;
  INT_VARS[gz] -= zz;

  F32::RunStream( QuatUpdateCommands , IMU_VARS );
}

inline static int abs( int v )
{
  return (v < 0) ? -v : v;
}

// Maps an input from (-N .. 0 .. +N) to output zero when the absolute input value is < db, removes the range from the output so it doesn't pop
static int Deadband( int v , int db )
{
  if( v > db ) return v - db;
  if( v < -db ) return v + db;
  return 0;
}

void QuatIMU_UpdateControls( RADIO * Radio , bool ManualMode , bool AutoManual )
{
  if( ManualMode & AutoManual ) {
    // Auto-manual mode behaves differently - manual control takes over at half throw, so compress
    // the range of manual into the other half of the range so the manual part feels less twitchy
    
    ((int*)IMU_VARS)[In_Elev] = Deadband( Radio->Elev, 485 ) << 1;
    ((int*)IMU_VARS)[In_Aile] = Deadband( Radio->Aile, 485 ) << 1;
  }
  else {
    ((int*)IMU_VARS)[In_Elev] = Deadband( Radio->Elev, 24 );
    ((int*)IMU_VARS)[In_Aile] = Deadband( Radio->Aile, 24 );
  }
  ((int*)IMU_VARS)[In_Rudd] = Deadband( Radio->Rudd, 24 );

  if( ManualMode ) {
    F32::RunStream( UpdateControls_Manual , IMU_VARS );
  }
  else {
    F32::RunStream( UpdateControlQuaternion_AutoLevel , IMU_VARS );
  }

  F32::WaitStream();
  F32::RunStream( UpdateControls_ComputeOrientationChange , IMU_VARS );
}



void QuatIMU_CompassInitCalibrate(void)
{
  F32::RunStream( QuatIMU_Mag_InitCalibrate , IMU_VARS );
}

void QuatIMU_CompassCalibrateAddSample(void)
{
  F32::RunStream( QuatIMU_Mag_AddCalibratePoint , IMU_VARS );
}

void QuatIMU_CompassCalibrateComputeOffsets(void)
{
  F32::RunStream( QuatIMU_Mag_ComputeCalibrate_SetupIteration , IMU_VARS );
  F32::WaitStream();    // Wait for the stream to complete

  for( int i=0; i<50; i++) {
    F32::RunStream( QuatIMU_Mag_ComputeCalibrate_IterationStep , IMU_VARS );
    F32::WaitStream();    // Wait for the stream to complete
  }
}

void QuatIMU_WaitForCompletion(void)
{
  F32::WaitStream();    // Wait for the stream to complete
}
