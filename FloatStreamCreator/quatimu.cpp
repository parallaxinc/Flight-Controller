
#include "functioninterpreter.h"
#include "quatimu.h"
#include <stdlib.h>
#include <memory.h>

#include "functionstream.h"

namespace FC {
enum IMU_VarLabels {
	#include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_vars.inc"

	IMU_VARS_SIZE                    // This entry MUST be last so we can compute the array size required
};

static union {
  float IMU_VARS[ IMU_VARS_SIZE ];
  int   INT_VARS[ IMU_VARS_SIZE ];
};

const int Const_UpdateRate = 200 / 8;
const int Const_OneG = 4096;

#define RadToDeg (180.0 / 3.141592654)                         //Degrees per Radian
#define GyroToDeg  (1000.0 / 70.0)                             //Gyro units per degree @ 2000 deg/sec sens = 70 mdps/bit
#define AccToG  (float)(Const_OneG)                            //Accelerometer per G @ 8g sensitivity = ~0.24414 mg/bit
#define GyroScale  (GyroToDeg * RadToDeg * (float)Const_UpdateRate)

const float Startup_ErrScale = 1.0f/32.0f;      // Converge quickly on startup
const float Running_ErrScale = 1.0f/512.0f;     // Converge more slowly once up & running

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

  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_var_init.inc"

  IMU_VARS[const_epsilon]           =    0.00000001f;     //Added to vector length value before inverting (1/X) to insure no divide-by-zero problems

  IMU_VARS[const_AccErrScale]       =    Startup_ErrScale;  //How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
//IMU_VARS[const_MagErrScale]       =    Startup_ErrScale;  //How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
  IMU_VARS[const_AccScale]          =    1.0f/(float)AccToG;//Conversion factor from accel units to G's
  IMU_VARS[const_G_mm_PerSecPerUpdate] = 9.80665f * 1000.0f / (float)Const_UpdateRate;  // gravity in mm/sec^2 per update
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

quint8 funcQuatUpdateCommands[] = {
  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_QuatUpdate.inc"
};

quint8 funcUpdateControls_Manual[] = {
  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_UpdateControls_Manual.inc"
};

quint8 funcUpdateControlQuaternion_AutoLevel[] = {
  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_UpdateControls_AutoLevel.inc"
};

quint8 funcUpdateControls_ComputeOrientationChange[] = {
  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_UpdateControls_ComputeOrientationChange.inc"
};


quint8 funcQuatIMU_Mag_InitCalibrate[] = {
  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_Mag_InitCalibrate.inc"
};

quint8 funcQuatIMU_Mag_AddCalibratePoint[] = {
  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_Mag_AddCalibratePoint.inc"
};

quint8 funcQuatIMU_Mag_ComputeCalibrate_SetupIteration[] = {
  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_Mag_ComputeCalibrate_SetupIteration.inc"
};

quint8 funcQuatIMU_Mag_ComputeCalibrate_IterationStep[] = {
  #include "F:\GitHub\Flight-Controller\Firmware-C\QuatIMU_Mag_ComputeCalibrate_IterationStep.inc"
};


void QuatIMU_Update( int * packetAddr )
{
  memcpy( &IMU_VARS[gx], packetAddr, 11 * sizeof(int) );
  //INT_VARS[gx] -= zx;
  //INT_VARS[gy] -= zy;
  //INT_VARS[gz] -= zz;

  FunctionInterpreter::Run( funcQuatUpdateCommands , IMU_VARS );
}


void QuatIMU_CompassInitCalibrate(void)
{
	QuatIMU_Mag_InitCalibrate();
	FunctionInterpreter::Run( funcQuatIMU_Mag_InitCalibrate , IMU_VARS );
}

void QuatIMU_CompassCalibrateAddSample(void)
{
	QuatIMU_Mag_AddCalibratePoint();
	FunctionInterpreter::Run( funcQuatIMU_Mag_AddCalibratePoint , IMU_VARS );
}

void QuatIMU_CompassCalibrateComputeOffsets(void)
{
	QuatIMU_Mag_ComputeCalibrate_SetupIteration();
	FunctionInterpreter::Run( funcQuatIMU_Mag_ComputeCalibrate_SetupIteration , IMU_VARS );

	for( int i=0; i<50; i++) {
		QuatIMU_Mag_ComputeCalibrate_IterationStep();
		FunctionInterpreter::Run( funcQuatIMU_Mag_ComputeCalibrate_IterationStep , IMU_VARS );
	}
}
}
