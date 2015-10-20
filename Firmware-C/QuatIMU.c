#include <propeller.h>
#include <stdio.h>
#include "F32.h"
#include "QuatIMU.h"
#include "Constants.h"

#define RadToDeg (180.0 / 3.141592654)                         //Degrees per Radian
#define GyroToDeg  (1000.0 / 70.0)                             //Gyro units per degree @ 2000 deg/sec sens = 70 mdps/bit
#define AccToG  (float)(Const_OneG)                            //Accelerometer per G @ 8g sensitivity = ~0.24414 mg/bit 
#define GyroScale  (GyroToDeg * RadToDeg * (float)Const_UpdateRate)

static int QuatUpdateCommands[];
static int TestUpdate[];


// Working variables for the IMU code
static int Roll, Pitch, Yaw;				// Outputs, scaled units
static int ThrustFactor;

// Inputs
static int zx, zy, zz;						// Gyro zero readings

static int gx, gy, gz;
static int ax, ay, az;						// Sensor inputs
static int mx, my, mz;
static int alt, altRate;


// Internal orientation storage
static float qx, qy, qz, qw;				// Body orientation quaternion

  
static float m00, m01, m02;
static float m10, m11, m12;					// Body orientation as a 3x3 matrix
static float m20, m21, m22;


static int fm00, fm01, fm02;
static int fm10, fm11, fm12;				// Body orientation as a 3x3 matrix in fixed integer form (+/- 65536 == +/- 1.0)
static int fm20, fm21, fm22;
  
  //Internal working variables - It isn't strictly necessary to break all of these out like this,
  //but it makes the code much more readable than having a bunch of temp variables
  
static float qdx, qdy, qdz, qdw;			// Incremental rotation quaternion

static float fx2, fy2, fz2;
static float fwx, fwy, fwz;					// Quaternion to matrix temp coefficients
static float fxy, fxz, fyz;

static float rx, ry, rz;					// Float versions of rotation components
static float fax, fay, faz;					// Float version of accelerometer vector

static float faxn, fayn, fazn;				// Float version of accelerometer vector (normalized)
static float rmag, cosr, sinr;				// magnitude, cos, sin values
  
static float errDiffX, errDiffY, errDiffZ;	// holds difference vector between target and measured orientation
static float errCorrX, errCorrY, errCorrZ;	// computed rotation correction factor
  
static float temp;							// temp value for use in equations

static float axRot, ayRot, azRot;
static float accWeight;

static float accRollCorrSin = 0.0;			// used to correct the accelerometer vector angle offset
static float accRollCorrCos = 1.0;
static float accPitchCorrSin = 0.0;
static float accPitchCorrCos = 1.0 ;

  //Terms used in complementary filter to compute altitude from accelerometer and pressure sensor altitude
static float velocityEstimate;
static float altitudeVelocity;
static float altitudeEstimate;
static int AltitudeEstMM;
static int VelocityEstMM;

static float forceX, forceY, forceZ;		// 'Current forces acting on craft, excluding gravity
static float forceWX, forceWY, forceWZ;		// Current forces acting on craft, excluding gravity, in world frame

  

void QuatIMU_Start(void)
{
  qx = 0.0;
  qy = 0.0;
  qz = 0.0;
  qw = 1.0;

  QuatIMU_InitFunctions();
}

int QuatIMU_GetPitch(void)
{
  return Pitch;
}  

int QuatIMU_GetRoll(void)
{
  return Roll;
}  

int QuatIMU_GetYaw(void)
{
  return Yaw;
}  

int QuatIMU_GetThrustFactor(void)
{
  return ThrustFactor;
}  

int *QuatIMU_GetSensors(void)
{
  return &gx;
}  

float * QuatIMU_GetMatrix(void)
{
  return &m00;
}  

int * QuatIMU_GetFixedMatrix(void)
{
  return &fm00;
}  

float * QuatIMU_GetQuaternion(void)
{
  return &qx;
}  

int QuatIMU_GetVerticalVelocityEstimate(void)
{
  return VelocityEstMM;
}  

int QuatIMU_GetAltitudeEstimate(void)
{
  return AltitudeEstMM;
}  

void QuatIMU_SetInitialAltitudeGuess( int altiMM )
{
  //altitudeEstimate = F32_FDiv( F32_FFloat(altiMM) , const_m_to_mm );
    altitudeEstimate = (float)altiMM / 1000.0;
}  


void QuatIMU_SetRollCorrection( float * addr )
{
  accRollCorrSin = addr[0];                   
  accRollCorrCos = addr[1];                   
}

void QuatIMU_SetPitchCorrection( float * addr )
{
  accPitchCorrSin = addr[0];
  accPitchCorrCos = addr[1];
}

void QuatIMU_InitFunctions(void)
{
  QuatIMU_AdjustStreamPointers( TestUpdate );
  QuatIMU_AdjustStreamPointers( QuatUpdateCommands );

  F32_StartStream( 0, TestUpdate );
}


void QuatIMU_SetGyroZero( int x, int y, int z )
{
  zx = x;
  zy = y;
  zz = z;
}


void QuatIMU_Update( int * packetAddr )
{
  memcpy( &gx, packetAddr, 11 * sizeof(int) );

  //Subtract gyro bias.  Probably better to do this in the sensor code, and ditto for accelerometer offset

  gx -= zx;
  gy -= zy;              
  gz -= zz;

  // F32_RunStream( QuatUpdateCommands );
  F32_RunStream( TestUpdate );
}


void QuatIMU_WaitForCompletion(void)
{
  F32_WaitStream();	// Wait for the stream to complete
}

    
/*
  'Quaternion update as C code  
  {
  fgx = gx / GyroScale + errCorrX
  fgy = gy / GyroScale + errCorrY
  fgz = gz / GyroScale + errCorrZ

  rmag = sqrt(fgx * fgx + fgy * fgy + fgz * fgz + 0.0000000001) / 2.0 

  
  cosr = Cos(rMag)
  sinr = Sin(rMag) / rMag
   
  qdot.w = -(r.x * x + r.y * y + r.z * z) / 2.0
  qdot.x =  (r.x * w + r.z * y - r.y * z) / 2.0
  qdot.y =  (r.y * w - r.z * x + r.x * z) / 2.0
  qdot.z =  (r.z * w + r.y * x - r.x * y) / 2.0
   
  q.w = cosr * q.w + sinr * qdot.w
  q.x = cosr * q.x + sinr * qdot.x
  q.y = cosr * q.y + sinr * qdot.y
  q.z = cosr * q.z + sinr * qdot.z
   
  q = q.Normalize()
  }
*/


void QuatIMU_AdjustStreamPointers( int * p )
{
	while( p[0] != 0 )
	{
		p[0] = (int)F32_GetCommandPtr( p[0] );				// Convert the instruction index into the address of a jump table instruction
		p += 4;
	}
}   



//Various constants used by the float math engine - Every command in the instruction stream reads two
//arguments from memory using memory addresses, so the values actually need to exist somewhere

#define PI  3.151592654

static float const_GyroScale         =    1.0 / GyroScale;    
static float const_NegGyroScale      =   -1.0 / GyroScale;

static long const_0                  =    0;
static long const_1                  =    1;
static long const_neg1               =   -1;
static long const_neg12              =   -12;              //Used to subtract from acc exponent, equivalent to /= 4096.0
static long const_16                 =    16;              //Used to add to exponents, equivalent to *= 65536.0
static float const_F1                =    1.0;
static float const_F2                =    2.0;

static float const_epsilon           =    0.0000000001;    //Added to vector length value before inverting (1/X) to insure no divide-by-zero problems
static float const_half              =    0.5;
static float const_neghalf           =   -0.5;


static float const_ErrScale          =    1.0/512.0;       //How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
static float const_AccScale          =    1.0/AccToG;      //Conversion factor from accel units to G's
static float const_outAngleScale     =    65536.0 / PI;               
static float const_outNegAngleScale  =    -65536.0 / PI;               
static float const_ThrustScale       =    256.0;
static float const_GMetersPerSec     =    9.80665;
static float const_AltiVelScale      =    1.0/1000.0;      //Convert mm to m
static float const_UpdateScale       =    1.0 / (float)Const_UpdateRate;	//Convert units/sec to units/update
static float const_m_to_mm           =    1000.0;

static float const_velAccScale       =    0.9995;
static float const_velAltiScale      =    0.0005;

static float const_velAccTrust       =    0.999;
static float const_velAltiTrust      =    0.001;



static int TestUpdate[] = {
        F32_opFloat, (int)&gx, 0, (int)&rx,                           //rx = float(gx)
        F32_opMul, (int)&rx, (int)&const_GyroScale, (int)&ry,         //rx *= GyroScale
        F32_opDiv, (int)&ry, (int)&const_GyroScale, (int)&rz,         //rx /= GyroScale
        F32_opTruncRound, (int)&rx,  (int)&const_0, (int)&Pitch,  
        0, 0, 0, 0
        };
        
  



  //fgx = gx / GyroScale + errCorrX
              
static int QuatUpdateCommands[] = {
        F32_opFloat, (int)&gx, 0, (int)&rx,                         //rx = float(gx)
        F32_opMul, (int)&rx, (int)&const_GyroScale, (int)&rx,            //rx /= GyroScale
        F32_opAdd, (int)&rx, (int)&errCorrX, (int)&rx,                   //rx += errCorrX

  //fgy = gy / GyroScale + errCorrY
        F32_opFloat, (int)&gz,  0, (int)&ry,                         //ry = float(gz)
        F32_opMul, (int)&ry, (int)&const_NegGyroScale, (int)&ry,         //ry /= GyroScale
        F32_opAdd, (int)&ry, (int)&errCorrY, (int)&ry,                   //ry += errCorrY

  //fgz = gz / GyroScale + errCorrZ
        F32_opFloat, (int)&gy, 0, (int)&rz,                         //rz = float(gy)
        F32_opMul, (int)&rz, (int)&const_NegGyroScale, (int)&rz,         //rz /= GyroScale
        F32_opAdd, (int)&rz, (int)&errCorrZ, (int)&rz,                   //rz += errCorrZ

  //rmag = sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) * 0.5
        F32_opSqr, (int)&rx, 0, (int)&rmag,                                  //rmag = fgx*fgx
        F32_opSqr, (int)&ry, 0, (int)&temp,                                  //temp = fgy*fgy
        F32_opAdd, (int)&rmag, (int)&temp, (int)&rmag,                            //rmag += temp
        F32_opSqr, (int)&rz, 0, (int)&temp,                                  //temp = fgz*fgz
        F32_opAdd, (int)&rmag, (int)&temp, (int)&rmag,                            //rmag += temp
        F32_opAdd, (int)&rmag, (int)&const_epsilon, (int)&rmag,                   //rmag += 0.00000001
        F32_opSqrt, (int)&rmag, 0, (int)&rmag,                               //rmag = Sqrt(rmag)                                                  
        F32_opShift, (int)&rmag, (int)&const_neg1, (int)&rmag,                    //rmag *= 0.5                                                  
  //8 instructions  (17)

  //cosr = Cos(rMag)
  //sinr = Sin(rMag) / rMag
        F32_opSinCos, (int)&rmag,  (int)&sinr, (int)&cosr,                         //sinr = Sin(rmag), cosr = Cos(rmag)  
        F32_opDiv, (int)&sinr,  (int)&rmag, (int)&sinr,                            //sinr /= rmag                                                  
  //3 instructions  (20)

  //qdot.w =  (r.x*x + r.y*y + r.z*z) * -0.5
        F32_opMul, (int)&rx,  (int)&qx, (int)&qdw,                                 //qdw = rx*qx 
        F32_opMul, (int)&ry,  (int)&qy, (int)&temp,                                //temp = ry*qy
        F32_opAdd, (int)&qdw,  (int)&temp, (int)&qdw,                              //qdw += temp
        F32_opMul, (int)&rz,  (int)&qz, (int)&temp,                                //temp = rz*qz
        F32_opAdd, (int)&qdw,  (int)&temp, (int)&qdw,                              //qdw += temp
        F32_opMul, (int)&qdw,  (int)&const_neghalf, (int)&qdw,                     //qdw *= -0.5
  //8 instructions  (28)

  //qdot.x =  (r.x*w + r.z*y - r.y*z) * 0.5
        F32_opMul, (int)&rx,  (int)&qw, (int)&qdx,                                 //qdx = rx*qw 
        F32_opMul, (int)&rz,  (int)&qy, (int)&temp,                                //temp = rz*qy
        F32_opAdd, (int)&qdx,  (int)&temp, (int)&qdx,                              //qdx += temp
        F32_opMul, (int)&ry,  (int)&qz, (int)&temp,                                //temp = ry*qz
        F32_opSub, (int)&qdx,  (int)&temp, (int)&qdx,                              //qdx -= temp
        F32_opShift, (int)&qdx,  (int)&const_neg1, (int)&qdx,                      //qdx *= 0.5
  //8 instructions  (36)

  //qdot.y =  (r.y*w - r.z*x + r.x*z) * 0.5
        F32_opMul, (int)&ry,  (int)&qw, (int)&qdy,                                 //qdy = ry*qw 
        F32_opMul, (int)&rz,  (int)&qx, (int)&temp,                                //temp = rz*qx
        F32_opSub, (int)&qdy,  (int)&temp, (int)&qdy,                              //qdy -= temp
        F32_opMul, (int)&rx,  (int)&qz, (int)&temp,                                //temp = rx*qz
        F32_opAdd, (int)&qdy,  (int)&temp, (int)&qdy,                              //qdy += temp
        F32_opShift, (int)&qdy,  (int)&const_neg1, (int)&qdy,                      //qdy *= 0.5
  //8 instructions  (44)

  //qdot.z =  (r.z*w + r.y*x - r.x*y) * 0.5
        F32_opMul, (int)&rz,  (int)&qw, (int)&qdz,                                 //qdz = rz*qw 
        F32_opMul, (int)&ry,  (int)&qx, (int)&temp,                                //temp = ry*qx
        F32_opAdd, (int)&qdz,  (int)&temp, (int)&qdz,                              //qdz += temp
        F32_opMul, (int)&rx,  (int)&qy, (int)&temp,                                //temp = rx*qy
        F32_opSub, (int)&qdz,  (int)&temp, (int)&qdz,                              //qdz -= temp
        F32_opShift, (int)&qdz,  (int)&const_neg1, (int)&qdz,                      //qdz *= 0.5
  //8 instructions  (52)
   
  //q.w = cosr * q.w + sinr * qdot.w
        F32_opMul, (int)&cosr,  (int)&qw, (int)&qw,                                //qw = cosr*qw 
        F32_opMul, (int)&sinr,  (int)&qdw, (int)&temp,                             //temp = sinr*qdw
        F32_opAdd, (int)&qw,  (int)&temp, (int)&qw,                                //qw += temp

  //q.x = cosr * q.x + sinr * qdot.x
        F32_opMul, (int)&cosr,  (int)&qx, (int)&qx,                                //qx = cosr*qx 
        F32_opMul, (int)&sinr,  (int)&qdx, (int)&temp,                             //temp = sinr*qdx
        F32_opAdd, (int)&qx,  (int)&temp, (int)&qx,                                //qx += temp

  //q.y = cosr * q.y + sinr * qdot.y
        F32_opMul, (int)&cosr,  (int)&qy, (int)&qy,                                //qy = cosr*qy 
        F32_opMul, (int)&sinr,  (int)&qdy, (int)&temp,                             //temp = sinr*qdy
        F32_opAdd, (int)&qy,  (int)&temp, (int)&qy,                                //qy += temp

  //q.z = cosr * q.z + sinr * qdot.z
        F32_opMul, (int)&cosr,  (int)&qz, (int)&qz,                                //qz = cosr*qz 
        F32_opMul, (int)&sinr,  (int)&qdz, (int)&temp,                             //temp = sinr*qdz
        F32_opAdd, (int)&qz,  (int)&temp, (int)&qz,                                //qz += temp
  //12 instructions  (64)

  //q = q.Normalize()
  //rmag = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w + 0.0000001)
        F32_opSqr, (int)&qx,  0, (int)&rmag,                                  //rmag = qx*qx 
        F32_opSqr, (int)&qy,  0, (int)&temp,                                  //temp = qy*qy 
        F32_opAdd, (int)&rmag,  (int)&temp, (int)&rmag,                            //rmag += temp 
        F32_opSqr, (int)&qz,  0, (int)&temp,                                  //temp = qz*qz 
        F32_opAdd, (int)&rmag,  (int)&temp, (int)&rmag,                            //rmag += temp 
        F32_opSqr, (int)&qw,  0, (int)&temp,                                  //temp = qw*qw 
        F32_opAdd, (int)&rmag,  (int)&temp, (int)&rmag,                            //rmag += temp 
        F32_opAdd, (int)&rmag,  (int)&const_epsilon, (int)&rmag,                   //rmag += 0.0000001 
        F32_opSqrt, (int)&rmag,  0, (int)&rmag,                               //sqrt(rmag) 
  //9 instructions (73)

  //q /= rmag   
        F32_opDiv, (int)&qw,  (int)&rmag, (int)&qw,                                //qw /= rmag 
        F32_opDiv, (int)&qx,  (int)&rmag, (int)&qx,                                //qx /= rmag 
        F32_opDiv, (int)&qy,  (int)&rmag, (int)&qy,                                //qy /= rmag 
        F32_opDiv, (int)&qz,  (int)&rmag, (int)&qz,                                //qz /= rmag 
  //4 instructions (77)


  //Now convert the updated quaternion to a rotation matrix 

  //fx2 = qx * qx;
  //fy2 = qy * qy;
  //fz2 = qz * qz;
        F32_opSqr, (int)&qx,  0, (int)&fx2,                                   //fx2 = qx *qx
        F32_opSqr, (int)&qy,  0, (int)&fy2,                                   //fy2 = qy *qy
        F32_opSqr, (int)&qz,  0, (int)&fz2,                                   //fz2 = qz *qz
  //3 instructions (80)

  //fwx = qw * qx;
  //fwy = qw * qy;
  //fwz = qw * qz;
        F32_opMul, (int)&qw,  (int)&qx, (int)&fwx,                                 //fwx = qw *qx
        F32_opMul, (int)&qw,  (int)&qy, (int)&fwy,                                 //fwy = qw *qy
        F32_opMul, (int)&qw,  (int)&qz, (int)&fwz,                                 //fwz = qw *qz
  //3 instructions (83)

  //fxy = qx * qy;
  //fxz = qx * qz;
  //fyz = qy * qz;
        F32_opMul, (int)&qx,  (int)&qy, (int)&fxy,                                 //fxy = qx *qy
        F32_opMul, (int)&qx,  (int)&qz, (int)&fxz,                                 //fxz = qx *qz
        F32_opMul, (int)&qy,  (int)&qz, (int)&fyz,                                 //fyz = qy *qz
  //3 instructions (86)

   
  //m00 = 1.0f - 2.0f * (y2 + z2)
        F32_opAdd, (int)&fy2,  (int)&fz2, (int)&temp,                              //temp = fy2+fz2
        F32_opShift, (int)&temp,  (int)&const_1, (int)&temp,                       //temp *= 2.0
        F32_opSub, (int)&const_F1,  (int)&temp, (int)&m00,                         //m00 = 1.0 - temp
     
  //m01 =        2.0f * (fxy - fwz)
        F32_opSub, (int)&fxy,  (int)&fwz, (int)&temp,                              //temp = fxy-fwz
        F32_opShift, (int)&temp,  (int)&const_1, (int)&m01,                        //m01 = 2.0 * temp

  //m02 =        2.0f * (fxz + fwy)
        F32_opAdd, (int)&fxz,  (int)&fwy, (int)&temp,                              //temp = fxz+fwy
        F32_opShift, (int)&temp,  (int)&const_1, (int)&m02,                        //m02 = 2.0 * temp
  //7 instructions (93)

   
  //m10 =        2.0f * (fxy + fwz)
        F32_opAdd, (int)&fxy,  (int)&fwz, (int)&temp,                              //temp = fxy-fwz
        F32_opShift, (int)&temp,  (int)&const_1, (int)&m10,                        //m10 = 2.0 * temp

  //m11 = 1.0f - 2.0f * (x2 + z2)
        F32_opAdd, (int)&fx2,  (int)&fz2, (int)&temp,                              //temp = fx2+fz2
        F32_opShift, (int)&temp,  (int)&const_1, (int)&temp,                       //temp *= 2.0
        F32_opSub, (int)&const_F1,  (int)&temp, (int)&m11,                         //m11 = 1.0 - temp

  //m12 =        2.0f * (fyz - fwx)
        F32_opSub, (int)&fyz,  (int)&fwx, (int)&temp,                              //temp = fyz-fwx
        F32_opShift, (int)&temp,  (int)&const_1, (int)&m12,                        //m12 = 2.0 * temp
  //7 instructions (100)

   
  //m20 =        2.0f * (fxz - fwy)
        F32_opSub, (int)&fxz,  (int)&fwy, (int)&temp,                              //temp = fxz-fwz
        F32_opShift, (int)&temp,  (int)&const_1, (int)&m20,                        //m20 = 2.0 * temp

  //m21 =        2.0f * (fyz + fwx)
        F32_opAdd, (int)&fyz,  (int)&fwx, (int)&temp,                              //temp = fyz+fwx
        F32_opShift, (int)&temp,  (int)&const_1, (int)&m21,                        //m21 = 2.0 * temp

  //m22 = 1.0f - 2.0f * (x2 + y2)
        F32_opAdd, (int)&fx2,  (int)&fy2, (int)&temp,                              //temp = fx2+fy2
        F32_opShift, (int)&temp,  (int)&const_1, (int)&temp,                       //temp *= 2.0
        F32_opSub, (int)&const_F1,  (int)&temp, (int)&m22,                         //m22 = 1.0 - temp
  //7 instructions (107)



  //fax =  packet.ax;           // Acceleration in X (left/right)
  //fay =  packet.az;           // Acceleration in Y (up/down)
  //faz =  packet.ay;           // Acceleration in Z (toward/away)
        F32_opFloat, (int)&ax,  0, (int)&fax,
        F32_opFloat, (int)&az,  0, (int)&fay,
        F32_opFloat, (int)&ay,  0, (int)&faz,
        F32_opNeg, (int)&fax,  0, (int)&fax,


//Rotation correction of the accelerometer vector - rotate around the pitch and roll axes by the specified amounts

  //axRot = (fax * accRollCorrCos) - (fay * accRollCorrSin)
        F32_opMul, (int)&fax,  (int)&accRollCorrCos, (int)&axRot,
        F32_opMul, (int)&fay,  (int)&accRollCorrSin, (int)&temp,
        F32_opSub, (int)&axRot,  (int)&temp, (int)&axRot,

  //ayRot = (fax * accRollCorrSin) + (fay * accRollCorrCos)
        F32_opMul, (int)&fax,  (int)&accRollCorrSin, (int)&ayRot,
        F32_opMul, (int)&fay,  (int)&accRollCorrCos, (int)&temp,
        F32_opAdd, (int)&ayRot,  (int)&temp, (int)&ayRot,

  //fax = axRot         
  //fay = ayRot
        F32_opMov, (int)&axRot,  0, (int)&fax,
        F32_opMov, (int)&ayRot,  0, (int)&fay,



  //axRot = (faz * accPitchCorrCos) - (fay * accPitchCorrSin)
        F32_opMul, (int)&faz,  (int)&accPitchCorrCos, (int)&axRot,
        F32_opMul, (int)&fay,  (int)&accPitchCorrSin, (int)&temp,
        F32_opSub, (int)&axRot,  (int)&temp, (int)&axRot, 

  //ayRot = (fax * accPitchCorrSin) + (fay * accPitchCorrCos)
        F32_opMul, (int)&faz,  (int)&accPitchCorrSin, (int)&ayRot,                           
        F32_opMul, (int)&fay,  (int)&accPitchCorrCos, (int)&temp,
        F32_opAdd, (int)&ayRot,  (int)&temp, (int)&ayRot,

  //faz = axRot         
  //fay = ayRot
        F32_opMov, (int)&axRot,  0, (int)&faz,          
        F32_opMov, (int)&ayRot,  0, (int)&fay,          



//Compute length of the accelerometer vector to decide weighting                                   

  //rmag = facc.length
        F32_opSqr, (int)&fax,  0, (int)&rmag,                                  //rmag = fax*fax
        F32_opSqr, (int)&fay,  0, (int)&temp,                                  //temp = fay*fay
        F32_opAdd, (int)&rmag,  (int)&temp, (int)&rmag,                             //rmag += temp
        F32_opSqr, (int)&faz,  0, (int)&temp,                                  //temp = faz*faz
        F32_opAdd, (int)&rmag,  (int)&temp, (int)&rmag,                             //rmag += temp
        F32_opAdd, (int)&rmag,  (int)&const_epsilon, (int)&rmag,                    //rmag += 0.00000001
        F32_opSqrt, (int)&rmag,  0, (int)&rmag,                                //rmag = Sqrt(rmag)                                                  

  //facc /= rmag
        F32_opDiv, (int)&fax,  (int)&rmag, (int)&faxn,                              //faxn = fax / rmag 
        F32_opDiv, (int)&fay,  (int)&rmag, (int)&fayn,                              //fayn = fay / rmag 
        F32_opDiv, (int)&faz,  (int)&rmag, (int)&fazn,                              //fazn = faz / rmag 



  //accWeight = 1.0 - FMin( FAbs( 2.0 - accLen * 2.0 ), 1.0 )
        F32_opMul, (int)&rmag,  (int)&const_AccScale, (int)&rmag,                   //rmag /= accScale (accelerometer to 1G units)
        F32_opShift, (int)&rmag,  (int)&const_1, (int)&accWeight,                   //accWeight = rmag * 2.0
        F32_opSub, (int)&const_F2,  (int)&accWeight, (int)&accWeight,               //accWeight = 2.0 - accWeight
        F32_opFAbs, (int)&accWeight,  0, (int)&accWeight,                      //accWeight = FAbs(accWeight)
        F32_opFMin, (int)&accWeight,  (int)&const_F1, (int)&accWeight,              //accWeight = FMin( accWeight, 1.0 )
        F32_opSub, (int)&const_F1,  (int)&accWeight, (int)&accWeight,               //accWeight = 1.0 - accWeight                                                

   

  //errDiffX = fayn * m12 - fazn * m11
        F32_opMul, (int)&fayn,  (int)&m12, (int)&errDiffX, 
        F32_opMul, (int)&fazn,  (int)&m11, (int)&temp, 
        F32_opSub, (int)&errDiffX,  (int)&temp, (int)&errDiffX, 

  //errDiffY = fazn * m10 - faxn * m12
        F32_opMul, (int)&fazn,  (int)&m10, (int)&errDiffY, 
        F32_opMul, (int)&faxn,  (int)&m12, (int)&temp, 
        F32_opSub, (int)&errDiffY,  (int)&temp, (int)&errDiffY, 

  //errDiffZ = faxn * m11 - fayn * m10
        F32_opMul, (int)&faxn,  (int)&m11, (int)&errDiffZ, 
        F32_opMul, (int)&fayn,  (int)&m10, (int)&temp,
        F32_opSub, (int)&errDiffZ,  (int)&temp, (int)&errDiffZ, 

  //accWeight *= const_ErrScale   
        F32_opMul, (int)&const_ErrScale,  (int)&accWeight, (int)&accWeight,

  //Test: Does ErrCorr need to be rotated into the local frame from the world frame?


  //errCorr = errDiff * accWeight
        F32_opMul, (int)&errDiffX,  (int)&accWeight, (int)&errCorrX,  
        F32_opMul, (int)&errDiffY,  (int)&accWeight, (int)&errCorrY,  
        F32_opMul, (int)&errDiffZ,  (int)&accWeight, (int)&errCorrZ,  


    //tx := Flt.ASin( Flt.FFloatDiv28( DCM.GetM12 ) )     //Convert to float, then divide by (float)(1<<28)
    //tz := Flt.ASin( Flt.FFloatDiv28( DCM.GetM10 ) )     //Convert to float, then divide by (float)(1<<28) 

    //XAngle := Flt.FRound( Flt.FMul( tx,  constant( 320000.0 / (PI / 2.0)) ) ) 
    //ZAngle := Flt.FRound( Flt.FMul( tz,  constant(-320000.0 / (PI / 2.0)) ) )

    //if( DCM.GetMatrixvalue(4) < 0 )                     //If the Y value of the Y axis is negative, we//re upside down
    //  if( ||ZAngle > ||XAngle ) 
    //    ZAngle := ZAngle 

    //For heading, I want an actual angular value, so this returns me an int between 0 (int)& 65535, where 0 is forward
    //YAngle := Flt.FRound( Flt.FMul( Flt.Atan2( Flt.FFloat(DCM.GetM20), Flt.FFloat(DCM.GetM22)), constant(32768.0 / PI) ) ) (int)& 65535 


        F32_opASinCos, (int)&m12,  0, (int)&temp,  
        F32_opMul, (int)&temp,  (int)&const_outAngleScale, (int)&temp,
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&Pitch,  
    
        F32_opASinCos, (int)&m10,  0, (int)&temp,  
        F32_opMul, (int)&temp,  (int)&const_outNegAngleScale, (int)&temp, 
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&Roll,  
        
        F32_opATan2, (int)&m20,  (int)&m22, (int)&temp,  
        F32_opMul, (int)&temp,  (int)&const_outNegAngleScale, (int)&temp,    
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&Yaw,  


        F32_opDiv, (int)&const_F1,  (int)&m11, (int)&temp,                          // 1.0/m11 = scale factor for thrust - this will be infinite if perpendicular to ground   
        F32_opMul, (int)&temp,  (int)&const_ThrustScale, (int)&temp,                // *= 256.0  
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&ThrustFactor,  



  //Compute the running height estimate

  //force := acc / 4096.0
        F32_opShift, (int)&fax,  (int)&const_neg12, (int)&forceX,
        F32_opShift, (int)&fay,  (int)&const_neg12, (int)&forceY,
        F32_opShift, (int)&faz,  (int)&const_neg12, (int)&forceZ,

  //force -= m[1,0], m[1,1], m[1,2]  - Subtract gravity (1G, straight down)
        F32_opSub, (int)&forceX,  (int)&m10, (int)&forceX,    
        F32_opSub, (int)&forceY,  (int)&m11, (int)&forceY,    
        F32_opSub, (int)&forceZ,  (int)&m12, (int)&forceZ,    

  //forceWY := M.Transpose().Mul(Force).y                 //Orient force vector into world frame
  //forceWY = m01*forceX + m11*forceY + m21*forceZ

        F32_opMul, (int)&forceX,  (int)&m01, (int)&forceWY,  
   
        F32_opMul, (int)&forceY,  (int)&m11, (int)&temp,  
        F32_opAdd, (int)&forceWY,  (int)&temp, (int)&forceWY,  

        F32_opMul, (int)&forceZ,  (int)&m21, (int)&temp,  
        F32_opAdd, (int)&forceWY,  (int)&temp, (int)&forceWY,  

  //forceWY *= 9.8                                       //Convert to M/sec^2
        F32_opMul, (int)&forceWY,  (int)&const_GMetersPerSec, (int)&forceWY,  



        F32_opMul, (int)&forceWY,  (int)&const_UpdateScale, (int)&temp,            //temp := forceWY / UpdateRate
        F32_opAdd, (int)&velocityEstimate,  (int)&temp, (int)&velocityEstimate,     //velEstimate += forceWY / UpdateRate

  
        F32_opFloat, (int)&altRate,  0, (int)&altitudeVelocity,                //AltVelocity = float(altRate)
        F32_opMul, (int)&altitudeVelocity,  (int)&const_AltiVelScale, (int)&altitudeVelocity,   //Convert from mm/sec to m/sec   


  //VelocityEstimate := (VelocityEstimate * 0.9950) + (altVelocity * 0.0050)
        F32_opMul, (int)&velocityEstimate,  (int)&const_velAccScale, (int)&velocityEstimate, 
        F32_opMul, (int)&altitudeVelocity,  (int)&const_velAltiScale, (int)&temp,  
        F32_opAdd, (int)&velocityEstimate,  (int)&temp, (int)&velocityEstimate,   

  //altitudeEstimate += velocityEstimate / UpdateRate
        F32_opMul, (int)&velocityEstimate,  (int)&const_UpdateScale, (int)&temp , 
        F32_opAdd, (int)&altitudeEstimate,  (int)&temp, (int)&altitudeEstimate,   

  //altitudeEstimate := (altitudeEstimate * 0.9950) * (alti / 1000.0) * 0.0050
        F32_opMul, (int)&altitudeEstimate,  (int)&const_velAccTrust, (int)&altitudeEstimate, 

        F32_opFloat, (int)&alt,  0, (int)&temp,                               //temp := float(alt)
        F32_opDiv, (int)&temp,  (int)&const_m_to_mm, (int)&temp,                   //temp /= 1000.0    (alt now in m)
        F32_opMul, (int)&temp,  (int)&const_velAltiTrust, (int)&temp,              //temp *= 0.0050
        F32_opAdd, (int)&altitudeEstimate,  (int)&temp, (int)&altitudeEstimate,    //altEstimate += temp 


        F32_opMul, (int)&altitudeEstimate,  (int)&const_m_to_mm, (int)&temp,       //temp = altEst * 1000.0    (temp now in mm)
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&AltitudeEstMM, 

        F32_opMul, (int)&velocityEstimate,  (int)&const_m_to_mm, (int)&temp,       //temp = velEst * 1000.0    (temp now in mm/sec)
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&VelocityEstMM, 


  //Create a fixed point version of the orientation matrix
        F32_opShift, (int)&m00,  (int)&const_16, (int)&temp,   
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm00, 
        F32_opShift, (int)&m01,  (int)&const_16, (int)&temp,   
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm01, 
        F32_opShift, (int)&m02,  (int)&const_16, (int)&temp,     
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm02, 

        F32_opShift, (int)&m10,  (int)&const_16, (int)&temp,   
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm10, 
        F32_opShift, (int)&m11,  (int)&const_16, (int)&temp,   
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm11, 
        F32_opShift, (int)&m12,  (int)&const_16, (int)&temp,   
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm12, 

        F32_opShift, (int)&m20,  (int)&const_16, (int)&temp,   
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm20, 
        F32_opShift, (int)&m21,  (int)&const_16, (int)&temp,   
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm21, 
        F32_opShift, (int)&m22,  (int)&const_16, (int)&temp,
        F32_opTruncRound, (int)&temp,  (int)&const_0, (int)&fm22,
        0, 0, 0, 0
		};
//}