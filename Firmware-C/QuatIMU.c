#include <propeller.h>
#include <stdio.h>
#include "F32.h"
#include "QuatIMU.h"
#include "Constants.h"

#define RadToDeg (180.0 / 3.141592654)                         //Degrees per Radian
#define GyroToDeg  (1000.0 / 70.0)                             //Gyro units per degree @ 2000 deg/sec sens = 70 mdps/bit
#define AccToG  (float)(Const_OneG)                            //Accelerometer per G @ 8g sensitivity = ~0.24414 mg/bit 
#define GyroScale  (GyroToDeg * RadToDeg * (float)Const_UpdateRate)


// Working variables for the IMU code, in a struct so the compiler doesn't screw up the order,
// or remove some of them due to overly aggressive (IE wrong) optimization.

static struct IMU_VARS {
    int Roll, Pitch, Yaw;				       // Outputs, scaled units
    int ThrustFactor;

    // Inputs
    int zx, zy, zz;						       // Gyro zero readings

    int gx, gy, gz;
    int ax, ay, az;						       // Sensor inputs
    int mx, my, mz;
    int alt, altRate;


    // Internal orientation storage
    float qx, qy, qz, qw;				       // Body orientation quaternion

      
    float m00, m01, m02;
    float m10, m11, m12;                 // Body orientation as a 3x3 matrix
    float m20, m21, m22;


    int fm00, fm01, fm02;
    int fm10, fm11, fm12;					    // Body orientation as a 3x3 matrix in fixed integer form (+/- 65536 == +/- 1.0)
    int fm20, fm21, fm22;
      
      //Internal working variables - It isn't strictly necessary to break all of these out like this,
      //but it makes the code much more readable than having a bunch of temp variables
      
    float qdx, qdy, qdz, qdw;			    // Incremental rotation quaternion

    float fx2, fy2, fz2;
    float fwx, fwy, fwz;					    // Quaternion to matrix temp coefficients
    float fxy, fxz, fyz;

    float rx, ry, rz;					       // Float versions of rotation components
    float fax, fay, faz;					    // Float version of accelerometer vector

    float faxn, fayn, fazn;				    // Float version of accelerometer vector (normalized)
    float rmag, cosr, sinr;				    // magnitude, cos, sin values
    
    float errDiffX, errDiffY, errDiffZ;	 // holds difference vector between target and measured orientation
    float errCorrX, errCorrY, errCorrZ;	 // computed rotation correction factor
    
    float temp;							       // temp value for use in equations

    float axRot, ayRot, azRot;
    float accWeight;

    float accRollCorrSin;			          // used to correct the accelerometer vector angle offset
    float accRollCorrCos;
    float accPitchCorrSin;
    float accPitchCorrCos;

      //Terms used in complementary filter to compute altitude from accelerometer and pressure sensor altitude
    float velocityEstimate;
    float altitudeVelocity;
    float altitudeEstimate;
    int AltitudeEstMM;
    int VelocityEstMM;

    float forceX, forceY, forceZ;		      // Current forces acting on craft, excluding gravity
    float forceWX, forceWY, forceWZ;		   // Current forces acting on craft, excluding gravity, in world frame
} v;


void QuatIMU_Start(void)
{
  memset( &v, 0, sizeof(v) );

  v.qx = 0.0f;
  v.qy = 0.0f;
  v.qz = 0.0f;
  v.qw = 1.0f;

  v.accRollCorrSin = 0.0f;			           // used to correct the accelerometer vector angle offset
  v.accRollCorrCos = 1.0f;
  v.accPitchCorrSin = 0.0f;
  v.accPitchCorrCos = 1.0f;

  QuatIMU_InitFunctions();
}

int QuatIMU_GetPitch(void) {
  return v.Pitch;
}

int QuatIMU_GetRoll(void) {
  return v.Roll;
}

int QuatIMU_GetYaw(void) {
  return v.Yaw;
}

int QuatIMU_GetThrustFactor(void) {
  return v.ThrustFactor;
}

int * QuatIMU_GetSensors(void) {
  return &v.gx;
}

float * QuatIMU_GetMatrix(void) {
  return &v.m00;
}  

int * QuatIMU_GetFixedMatrix(void) {
  return &v.fm00;
}

float * QuatIMU_GetQuaternion(void) {
  return &v.qx;
}

int QuatIMU_GetVerticalVelocityEstimate(void) {
  return v.VelocityEstMM;
}

int QuatIMU_GetAltitudeEstimate(void) {
  return v.AltitudeEstMM;
}

void QuatIMU_SetInitialAltitudeGuess( int altiMM )
{
  //altitudeEstimate = F32_FDiv( F32_FFloat(altiMM) , const_m_to_mm );
    v.altitudeEstimate = (float)altiMM / 1000.0;
}  


void QuatIMU_SetRollCorrection( float * addr )
{
  v.accRollCorrSin = addr[0];
  v.accRollCorrCos = addr[1];
}

void QuatIMU_SetPitchCorrection( float * addr )
{
  v.accPitchCorrSin = addr[0];
  v.accPitchCorrCos = addr[1];
}


void QuatIMU_SetGyroZero( int x, int y, int z )
{
  v.zx = x;
  v.zy = y;
  v.zz = z;
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
        p[0] = (int)F32::GetCommandPtr( p[0] );             // Convert the instruction index into the address of a jump table instruction
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



int TestUpdate[] = {
        //F32_opFloat, (int)&v.gx, 0, (int)&v.rx,                          //rx = float(gx)
        //F32_opMul, (int)&v.rx, (int)&const_GyroScale, (int)&v.rx,        //rx /= GyroScale
        //F32_opAdd, (int)&v.rx, (int)&errCorrX, (int)&v.rx,               //rx += errCorrX

  //fgy = gy / GyroScale + errCorrY
        F32_opFloat, (int)&v.gz,  0, (int)&v.ry,                           //ry = float(gz)
        F32_opMul, (int)&v.ry, (int)&const_NegGyroScale, (int)&v.ry,       //ry /= GyroScale
        F32_opAdd, (int)&v.ry, (int)&v.errCorrY, (int)&v.ry,               //ry += errCorrY

  //fgz = gz / GyroScale + errCorrZ
        F32_opFloat, (int)&v.gy, 0, (int)&v.rz,                            //rz = float(gy)
        F32_opMul, (int)&v.rz, (int)&const_NegGyroScale, (int)&v.rz,       //rz /= GyroScale
        F32_opAdd, (int)&v.rz, (int)&v.errCorrZ, (int)&v.rz,               //rz += errCorrZ

        0, 0, 0, 0
        };
        
  



  //fgx = gx / GyroScale + errCorrX
              
int QuatUpdateCommands[] = {
        F32_opFloat, (int)&v.gx, 0, (int)&v.rx,                            //rx = float(gx)
        F32_opMul, (int)&v.rx, (int)&const_GyroScale, (int)&v.rx,          //rx /= GyroScale
        F32_opAdd, (int)&v.rx, (int)&v.errCorrX, (int)&v.rx,               //rx += errCorrX

  //fgy = gy / GyroScale + errCorrY
        F32_opFloat, (int)&v.gz,  0, (int)&v.ry,                           //ry = float(gz)
        F32_opMul, (int)&v.ry, (int)&const_NegGyroScale, (int)&v.ry,       //ry /= GyroScale
        F32_opAdd, (int)&v.ry, (int)&v.errCorrY, (int)&v.ry,               //ry += errCorrY

  //fgz = gz / GyroScale + errCorrZ
        F32_opFloat, (int)&v.gy, 0, (int)&v.rz,                            //rz = float(gy)
        F32_opMul, (int)&v.rz, (int)&const_NegGyroScale, (int)&v.rz,       //rz /= GyroScale
        F32_opAdd, (int)&v.rz, (int)&v.errCorrZ, (int)&v.rz,               //rz += errCorrZ

  //rmag = sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) * 0.5
        F32_opSqr, (int)&v.rx, 0, (int)&v.rmag,                            //rmag = fgx*fgx
        F32_opSqr, (int)&v.ry, 0, (int)&v.temp,                            //temp = fgy*fgy
        F32_opAdd, (int)&v.rmag, (int)&v.temp, (int)&v.rmag,               //rmag += temp
        F32_opSqr, (int)&v.rz, 0, (int)&v.temp,                            //temp = fgz*fgz
        F32_opAdd, (int)&v.rmag, (int)&v.temp, (int)&v.rmag,               //rmag += temp
        F32_opAdd, (int)&v.rmag, (int)&const_epsilon, (int)&v.rmag,        //rmag += 0.00000001
        F32_opSqrt, (int)&v.rmag, 0, (int)&v.rmag,                         //rmag = Sqrt(rmag)                                                  
        F32_opShift, (int)&v.rmag, (int)&const_neg1, (int)&v.rmag,         //rmag *= 0.5                                                  
  //8 instructions  (17)

  //cosr = Cos(rMag)
  //sinr = Sin(rMag) / rMag
        F32_opSinCos, (int)&v.rmag,  (int)&v.sinr, (int)&v.cosr,           //sinr = Sin(rmag), cosr = Cos(rmag)  
        F32_opDiv, (int)&v.sinr,  (int)&v.rmag, (int)&v.sinr,              //sinr /= rmag                                                  
  //3 instructions  (20)

  //qdot.w =  (r.x*x + r.y*y + r.z*z) * -0.5
        F32_opMul, (int)&v.rx,  (int)&v.qx, (int)&v.qdw,                   //qdw = rx*qx 
        F32_opMul, (int)&v.ry,  (int)&v.qy, (int)&v.temp,                  //temp = ry*qy
        F32_opAdd, (int)&v.qdw,  (int)&v.temp, (int)&v.qdw,                //qdw += temp
        F32_opMul, (int)&v.rz,  (int)&v.qz, (int)&v.temp,                  //temp = rz*qz
        F32_opAdd, (int)&v.qdw,  (int)&v.temp, (int)&v.qdw,                //qdw += temp
        F32_opMul, (int)&v.qdw,  (int)&const_neghalf, (int)&v.qdw,         //qdw *= -0.5
  //8 instructions  (28)

  //qdot.x =  (r.x*w + r.z*y - r.y*z) * 0.5
        F32_opMul, (int)&v.rx,  (int)&v.qw, (int)&v.qdx,                   //qdx = rx*qw 
        F32_opMul, (int)&v.rz,  (int)&v.qy, (int)&v.temp,                  //temp = rz*qy
        F32_opAdd, (int)&v.qdx,  (int)&v.temp, (int)&v.qdx,                //qdx += temp
        F32_opMul, (int)&v.ry,  (int)&v.qz, (int)&v.temp,                  //temp = ry*qz
        F32_opSub, (int)&v.qdx,  (int)&v.temp, (int)&v.qdx,                //qdx -= temp
        F32_opShift, (int)&v.qdx,  (int)&const_neg1, (int)&v.qdx,          //qdx *= 0.5
  //8 instructions  (36)

  //qdot.y =  (r.y*w - r.z*x + r.x*z) * 0.5
        F32_opMul, (int)&v.ry,  (int)&v.qw, (int)&v.qdy,                   //qdy = ry*qw 
        F32_opMul, (int)&v.rz,  (int)&v.qx, (int)&v.temp,                  //temp = rz*qx
        F32_opSub, (int)&v.qdy,  (int)&v.temp, (int)&v.qdy,                //qdy -= temp
        F32_opMul, (int)&v.rx,  (int)&v.qz, (int)&v.temp,                  //temp = rx*qz
        F32_opAdd, (int)&v.qdy,  (int)&v.temp, (int)&v.qdy,                //qdy += temp
        F32_opShift, (int)&v.qdy,  (int)&const_neg1, (int)&v.qdy,          //qdy *= 0.5
  //8 instructions  (44)

  //qdot.z =  (r.z*w + r.y*x - r.x*y) * 0.5
        F32_opMul, (int)&v.rz,  (int)&v.qw, (int)&v.qdz,                   //qdz = rz*qw 
        F32_opMul, (int)&v.ry,  (int)&v.qx, (int)&v.temp,                  //temp = ry*qx
        F32_opAdd, (int)&v.qdz,  (int)&v.temp, (int)&v.qdz,                //qdz += temp
        F32_opMul, (int)&v.rx,  (int)&v.qy, (int)&v.temp,                  //temp = rx*qy
        F32_opSub, (int)&v.qdz,  (int)&v.temp, (int)&v.qdz,                //qdz -= temp
        F32_opShift, (int)&v.qdz,  (int)&const_neg1, (int)&v.qdz,          //qdz *= 0.5
  //8 instructions  (52)
   
  //q.w = cosr * q.w + sinr * qdot.w
        F32_opMul, (int)&v.cosr,  (int)&v.qw, (int)&v.qw,                  //qw = cosr*qw 
        F32_opMul, (int)&v.sinr,  (int)&v.qdw, (int)&v.temp,               //temp = sinr*qdw
        F32_opAdd, (int)&v.qw,  (int)&v.temp, (int)&v.qw,                  //qw += temp

  //q.x = cosr * q.x + sinr * qdot.x
        F32_opMul, (int)&v.cosr,  (int)&v.qx, (int)&v.qx,                  //qx = cosr*qx 
        F32_opMul, (int)&v.sinr,  (int)&v.qdx, (int)&v.temp,               //temp = sinr*qdx
        F32_opAdd, (int)&v.qx,  (int)&v.temp, (int)&v.qx,                  //qx += temp

  //q.y = cosr * q.y + sinr * qdot.y
        F32_opMul, (int)&v.cosr,  (int)&v.qy, (int)&v.qy,                  //qy = cosr*qy 
        F32_opMul, (int)&v.sinr,  (int)&v.qdy, (int)&v.temp,               //temp = sinr*qdy
        F32_opAdd, (int)&v.qy,  (int)&v.temp, (int)&v.qy,                  //qy += temp

  //q.z = cosr * q.z + sinr * qdot.z
        F32_opMul, (int)&v.cosr,  (int)&v.qz, (int)&v.qz,                  //qz = cosr*qz 
        F32_opMul, (int)&v.sinr,  (int)&v.qdz, (int)&v.temp,               //temp = sinr*qdz
        F32_opAdd, (int)&v.qz,  (int)&v.temp, (int)&v.qz,                  //qz += temp
  //12 instructions  (64)

  //q = q.Normalize()
  //rmag = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w + 0.0000001)
        F32_opSqr, (int)&v.qx,  0, (int)&v.rmag,                           //rmag = qx*qx 
        F32_opSqr, (int)&v.qy,  0, (int)&v.temp,                           //temp = qy*qy 
        F32_opAdd, (int)&v.rmag,  (int)&v.temp, (int)&v.rmag,              //rmag += temp 
        F32_opSqr, (int)&v.qz,  0, (int)&v.temp,                           //temp = qz*qz 
        F32_opAdd, (int)&v.rmag,  (int)&v.temp, (int)&v.rmag,              //rmag += temp 
        F32_opSqr, (int)&v.qw,  0, (int)&v.temp,                           //temp = qw*qw 
        F32_opAdd, (int)&v.rmag,  (int)&v.temp, (int)&v.rmag,              //rmag += temp 
        F32_opAdd, (int)&v.rmag,  (int)&const_epsilon, (int)&v.rmag,       //rmag += 0.0000001 
        F32_opSqrt, (int)&v.rmag,  0, (int)&v.rmag,                        //sqrt(rmag) 
  //9 instructions (73)

  //q /= rmag   
        F32_opDiv, (int)&v.qw,  (int)&v.rmag, (int)&v.qw,                  //qw /= rmag 
        F32_opDiv, (int)&v.qx,  (int)&v.rmag, (int)&v.qx,                  //qx /= rmag 
        F32_opDiv, (int)&v.qy,  (int)&v.rmag, (int)&v.qy,                  //qy /= rmag 
        F32_opDiv, (int)&v.qz,  (int)&v.rmag, (int)&v.qz,                  //qz /= rmag 
  //4 instructions (77)


  //Now convert the updated quaternion to a rotation matrix 

  //fx2 = qx * qx;
  //fy2 = qy * qy;
  //fz2 = qz * qz;
        F32_opSqr, (int)&v.qx,  0, (int)&v.fx2,                            //fx2 = qx *qx
        F32_opSqr, (int)&v.qy,  0, (int)&v.fy2,                            //fy2 = qy *qy
        F32_opSqr, (int)&v.qz,  0, (int)&v.fz2,                            //fz2 = qz *qz
  //3 instructions (80)

  //fwx = qw * qx;
  //fwy = qw * qy;
  //fwz = qw * qz;
        F32_opMul, (int)&v.qw,  (int)&v.qx, (int)&v.fwx,                   //fwx = qw *qx
        F32_opMul, (int)&v.qw,  (int)&v.qy, (int)&v.fwy,                   //fwy = qw *qy
        F32_opMul, (int)&v.qw,  (int)&v.qz, (int)&v.fwz,                   //fwz = qw *qz
  //3 instructions (83)

  //fxy = qx * qy;
  //fxz = qx * qz;
  //fyz = qy * qz;
        F32_opMul, (int)&v.qx,  (int)&v.qy, (int)&v.fxy,                   //fxy = qx *qy
        F32_opMul, (int)&v.qx,  (int)&v.qz, (int)&v.fxz,                   //fxz = qx *qz
        F32_opMul, (int)&v.qy,  (int)&v.qz, (int)&v.fyz,                   //fyz = qy *qz
  //3 instructions (86)

   
  //m00 = 1.0f - 2.0f * (y2 + z2)
        F32_opAdd, (int)&v.fy2,  (int)&v.fz2, (int)&v.temp,                //temp = fy2+fz2
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.temp,           //temp *= 2.0
        F32_opSub, (int)&const_F1,  (int)&v.temp, (int)&v.m00,             //m00 = 1.0 - temp
     
  //m01 =        2.0f * (fxy - fwz)
        F32_opSub, (int)&v.fxy,  (int)&v.fwz, (int)&v.temp,                //temp = fxy-fwz
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.m01,            //m01 = 2.0 * temp

  //m02 =        2.0f * (fxz + fwy)
        F32_opAdd, (int)&v.fxz,  (int)&v.fwy, (int)&v.temp,                //temp = fxz+fwy
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.m02,            //m02 = 2.0 * temp
  //7 instructions (93)

   
  //m10 =        2.0f * (fxy + fwz)
        F32_opAdd, (int)&v.fxy,  (int)&v.fwz, (int)&v.temp,                //temp = fxy-fwz
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.m10,            //m10 = 2.0 * temp

  //m11 = 1.0f - 2.0f * (x2 + z2)
        F32_opAdd, (int)&v.fx2,  (int)&v.fz2, (int)&v.temp,                //temp = fx2+fz2
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.temp,           //temp *= 2.0
        F32_opSub, (int)&const_F1,  (int)&v.temp, (int)&v.m11,             //m11 = 1.0 - temp

  //m12 =        2.0f * (fyz - fwx)
        F32_opSub, (int)&v.fyz,  (int)&v.fwx, (int)&v.temp,                //temp = fyz-fwx
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.m12,            //m12 = 2.0 * temp
  //7 instructions (100)

   
  //m20 =        2.0f * (fxz - fwy)
        F32_opSub, (int)&v.fxz,  (int)&v.fwy, (int)&v.temp,                //temp = fxz-fwz
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.m20,            //m20 = 2.0 * temp

  //m21 =        2.0f * (fyz + fwx)
        F32_opAdd, (int)&v.fyz,  (int)&v.fwx, (int)&v.temp,                //temp = fyz+fwx
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.m21,            //m21 = 2.0 * temp

  //m22 = 1.0f - 2.0f * (x2 + y2)
        F32_opAdd, (int)&v.fx2,  (int)&v.fy2, (int)&v.temp,                //temp = fx2+fy2
        F32_opShift, (int)&v.temp,  (int)&const_1, (int)&v.temp,           //temp *= 2.0
        F32_opSub, (int)&const_F1,  (int)&v.temp, (int)&v.m22,             //m22 = 1.0 - temp
  //7 instructions (107)



  //fax =  packet.ax;           // Acceleration in X (left/right)
  //fay =  packet.az;           // Acceleration in Y (up/down)
  //faz =  packet.ay;           // Acceleration in Z (toward/away)
        F32_opFloat, (int)&v.ax,  0, (int)&v.fax,
        F32_opFloat, (int)&v.az,  0, (int)&v.fay,
        F32_opFloat, (int)&v.ay,  0, (int)&v.faz,
        F32_opNeg, (int)&v.fax,  0, (int)&v.fax,


//Rotation correction of the accelerometer vector - rotate around the pitch and roll axes by the specified amounts

  //axRot = (fax * accRollCorrCos) - (fay * accRollCorrSin)
        F32_opMul, (int)&v.fax,  (int)&v.accRollCorrCos, (int)&v.axRot,
        F32_opMul, (int)&v.fay,  (int)&v.accRollCorrSin, (int)&v.temp,
        F32_opSub, (int)&v.axRot,  (int)&v.temp, (int)&v.axRot,

  //ayRot = (fax * accRollCorrSin) + (fay * accRollCorrCos)
        F32_opMul, (int)&v.fax,  (int)&v.accRollCorrSin, (int)&v.ayRot,
        F32_opMul, (int)&v.fay,  (int)&v.accRollCorrCos, (int)&v.temp,
        F32_opAdd, (int)&v.ayRot,  (int)&v.temp, (int)&v.ayRot,

  //fax = axRot         
  //fay = ayRot
        F32_opMov, (int)&v.axRot,  0, (int)&v.fax,
        F32_opMov, (int)&v.ayRot,  0, (int)&v.fay,



  //axRot = (faz * accPitchCorrCos) - (fay * accPitchCorrSin)
        F32_opMul, (int)&v.faz,  (int)&v.accPitchCorrCos, (int)&v.axRot,
        F32_opMul, (int)&v.fay,  (int)&v.accPitchCorrSin, (int)&v.temp,
        F32_opSub, (int)&v.axRot,  (int)&v.temp, (int)&v.axRot, 

  //ayRot = (fax * accPitchCorrSin) + (fay * accPitchCorrCos)
        F32_opMul, (int)&v.faz,  (int)&v.accPitchCorrSin, (int)&v.ayRot,                           
        F32_opMul, (int)&v.fay,  (int)&v.accPitchCorrCos, (int)&v.temp,
        F32_opAdd, (int)&v.ayRot,  (int)&v.temp, (int)&v.ayRot,

  //faz = axRot         
  //fay = ayRot
        F32_opMov, (int)&v.axRot,  0, (int)&v.faz,          
        F32_opMov, (int)&v.ayRot,  0, (int)&v.fay,          



//Compute length of the accelerometer vector to decide weighting                                   

  //rmag = facc.length
        F32_opSqr, (int)&v.fax,  0, (int)&v.rmag,                          //rmag = fax*fax
        F32_opSqr, (int)&v.fay,  0, (int)&v.temp,                          //temp = fay*fay
        F32_opAdd, (int)&v.rmag,  (int)&v.temp, (int)&v.rmag,              //rmag += temp
        F32_opSqr, (int)&v.faz,  0, (int)&v.temp,                          //temp = faz*faz
        F32_opAdd, (int)&v.rmag,  (int)&v.temp, (int)&v.rmag,              //rmag += temp
        F32_opAdd, (int)&v.rmag,  (int)&const_epsilon, (int)&v.rmag,       //rmag += 0.00000001
        F32_opSqrt, (int)&v.rmag,  0, (int)&v.rmag,                        //rmag = Sqrt(rmag)                                                  

  //facc /= rmag
        F32_opDiv, (int)&v.fax,  (int)&v.rmag, (int)&v.faxn,               //faxn = fax / rmag 
        F32_opDiv, (int)&v.fay,  (int)&v.rmag, (int)&v.fayn,               //fayn = fay / rmag 
        F32_opDiv, (int)&v.faz,  (int)&v.rmag, (int)&v.fazn,               //fazn = faz / rmag 



  //accWeight = 1.0 - FMin( FAbs( 2.0 - accLen * 2.0 ), 1.0 )
        F32_opMul, (int)&v.rmag,  (int)&const_AccScale, (int)&v.rmag,      //rmag /= accScale (accelerometer to 1G units)
        F32_opShift, (int)&v.rmag,  (int)&const_1, (int)&v.accWeight,      //accWeight = rmag * 2.0
        F32_opSub, (int)&const_F2,  (int)&v.accWeight, (int)&v.accWeight,  //accWeight = 2.0 - accWeight
        F32_opFAbs, (int)&v.accWeight,  0, (int)&v.accWeight,              //accWeight = FAbs(accWeight)
        F32_opFMin, (int)&v.accWeight,  (int)&const_F1, (int)&v.accWeight, //accWeight = FMin( accWeight, 1.0 )
        F32_opSub, (int)&const_F1,  (int)&v.accWeight, (int)&v.accWeight,  //accWeight = 1.0 - accWeight                                                

   

  //errDiffX = fayn * m12 - fazn * m11
        F32_opMul, (int)&v.fayn,  (int)&v.m12, (int)&v.errDiffX, 
        F32_opMul, (int)&v.fazn,  (int)&v.m11, (int)&v.temp, 
        F32_opSub, (int)&v.errDiffX,  (int)&v.temp, (int)&v.errDiffX, 

  //errDiffY = fazn * m10 - faxn * m12
        F32_opMul, (int)&v.fazn,  (int)&v.m10, (int)&v.errDiffY, 
        F32_opMul, (int)&v.faxn,  (int)&v.m12, (int)&v.temp, 
        F32_opSub, (int)&v.errDiffY,  (int)&v.temp, (int)&v.errDiffY, 

  //errDiffZ = faxn * m11 - fayn * m10
        F32_opMul, (int)&v.faxn,  (int)&v.m11, (int)&v.errDiffZ, 
        F32_opMul, (int)&v.fayn,  (int)&v.m10, (int)&v.temp,
        F32_opSub, (int)&v.errDiffZ,  (int)&v.temp, (int)&v.errDiffZ, 

  //accWeight *= const_ErrScale   
        F32_opMul, (int)&const_ErrScale,  (int)&v.accWeight, (int)&v.accWeight,

  //Test: Does ErrCorr need to be rotated into the local frame from the world frame?


  //errCorr = errDiff * accWeight
        F32_opMul, (int)&v.errDiffX,  (int)&v.accWeight, (int)&v.errCorrX,  
        F32_opMul, (int)&v.errDiffY,  (int)&v.accWeight, (int)&v.errCorrY,  
        F32_opMul, (int)&v.errDiffZ,  (int)&v.accWeight, (int)&v.errCorrZ,  


    //tx := Flt.ASin( Flt.FFloatDiv28( DCM.GetM12 ) )     //Convert to float, then divide by (float)(1<<28)
    //tz := Flt.ASin( Flt.FFloatDiv28( DCM.GetM10 ) )     //Convert to float, then divide by (float)(1<<28) 

    //XAngle := Flt.FRound( Flt.FMul( tx,  constant( 320000.0 / (PI / 2.0)) ) ) 
    //ZAngle := Flt.FRound( Flt.FMul( tz,  constant(-320000.0 / (PI / 2.0)) ) )

    //if( DCM.GetMatrixvalue(4) < 0 )                     //If the Y value of the Y axis is negative, we//re upside down
    //  if( ||ZAngle > ||XAngle ) 
    //    ZAngle := ZAngle 

    //For heading, I want an actual angular value, so this returns me an int between 0 (int)&v. 65535, where 0 is forward
    //YAngle := Flt.FRound( Flt.FMul( Flt.Atan2( Flt.FFloat(DCM.GetM20), Flt.FFloat(DCM.GetM22)), constant(32768.0 / PI) ) ) (int)&v. 65535 


        F32_opASinCos, (int)&v.m12,  0, (int)&v.temp,  
        F32_opMul, (int)&v.temp,  (int)&const_outAngleScale, (int)&v.temp,
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.Pitch,  
    
        F32_opASinCos, (int)&v.m10,  0, (int)&v.temp,  
        F32_opMul, (int)&v.temp,  (int)&const_outNegAngleScale, (int)&v.temp, 
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.Roll,  
        
        F32_opATan2, (int)&v.m20,  (int)&v.m22, (int)&v.temp,  
        F32_opMul, (int)&v.temp,  (int)&const_outNegAngleScale, (int)&v.temp,    
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.Yaw,  


        F32_opDiv, (int)&const_F1,  (int)&v.m11, (int)&v.temp,                          // 1.0/m11 = scale factor for thrust - this will be infinite if perpendicular to ground   
        F32_opMul, (int)&v.temp,  (int)&const_ThrustScale, (int)&v.temp,                // *= 256.0  
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.ThrustFactor,  



  //Compute the running height estimate

  //force := acc / 4096.0
        F32_opShift, (int)&v.fax,  (int)&const_neg12, (int)&v.forceX,
        F32_opShift, (int)&v.fay,  (int)&const_neg12, (int)&v.forceY,
        F32_opShift, (int)&v.faz,  (int)&const_neg12, (int)&v.forceZ,

  //force -= m[1,0], m[1,1], m[1,2]  - Subtract gravity (1G, straight down)
        F32_opSub, (int)&v.forceX,  (int)&v.m10, (int)&v.forceX,    
        F32_opSub, (int)&v.forceY,  (int)&v.m11, (int)&v.forceY,    
        F32_opSub, (int)&v.forceZ,  (int)&v.m12, (int)&v.forceZ,    

  //forceWY := M.Transpose().Mul(Force).y                 //Orient force vector into world frame
  //forceWY = m01*forceX + m11*forceY + m21*forceZ

        F32_opMul, (int)&v.forceX,  (int)&v.m01, (int)&v.forceWY,  
   
        F32_opMul, (int)&v.forceY,  (int)&v.m11, (int)&v.temp,  
        F32_opAdd, (int)&v.forceWY,  (int)&v.temp, (int)&v.forceWY,  

        F32_opMul, (int)&v.forceZ,  (int)&v.m21, (int)&v.temp,  
        F32_opAdd, (int)&v.forceWY,  (int)&v.temp, (int)&v.forceWY,  

  //forceWY *= 9.8                                       //Convert to M/sec^2
        F32_opMul, (int)&v.forceWY,  (int)&const_GMetersPerSec, (int)&v.forceWY,  



        F32_opMul, (int)&v.forceWY,  (int)&const_UpdateScale, (int)&v.temp,            //temp := forceWY / UpdateRate
        F32_opAdd, (int)&v.velocityEstimate,  (int)&v.temp, (int)&v.velocityEstimate,     //velEstimate += forceWY / UpdateRate

  
        F32_opFloat, (int)&v.altRate,  0, (int)&v.altitudeVelocity,                //AltVelocity = float(altRate)
        F32_opMul, (int)&v.altitudeVelocity,  (int)&const_AltiVelScale, (int)&v.altitudeVelocity,   //Convert from mm/sec to m/sec   


  //VelocityEstimate := (VelocityEstimate * 0.9950) + (altVelocity * 0.0050)
        F32_opMul, (int)&v.velocityEstimate,  (int)&const_velAccScale, (int)&v.velocityEstimate, 
        F32_opMul, (int)&v.altitudeVelocity,  (int)&const_velAltiScale, (int)&v.temp,  
        F32_opAdd, (int)&v.velocityEstimate,  (int)&v.temp, (int)&v.velocityEstimate,   

  //altitudeEstimate += velocityEstimate / UpdateRate
        F32_opMul, (int)&v.velocityEstimate,  (int)&const_UpdateScale, (int)&v.temp , 
        F32_opAdd, (int)&v.altitudeEstimate,  (int)&v.temp, (int)&v.altitudeEstimate,   

  //altitudeEstimate := (altitudeEstimate * 0.9950) * (alti / 1000.0) * 0.0050
        F32_opMul, (int)&v.altitudeEstimate,  (int)&const_velAccTrust, (int)&v.altitudeEstimate, 

        F32_opFloat, (int)&v.alt,  0, (int)&v.temp,                                   //temp := float(alt)
        F32_opDiv, (int)&v.temp,  (int)&const_m_to_mm, (int)&v.temp,                  //temp /= 1000.0    (alt now in m)
        F32_opMul, (int)&v.temp,  (int)&const_velAltiTrust, (int)&v.temp,             //temp *= 0.0050
        F32_opAdd, (int)&v.altitudeEstimate,  (int)&v.temp, (int)&v.altitudeEstimate, //altEstimate += temp 


        F32_opMul, (int)&v.altitudeEstimate,  (int)&const_m_to_mm, (int)&v.temp,      //temp = altEst * 1000.0    (temp now in mm)
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.AltitudeEstMM, 

        F32_opMul, (int)&v.velocityEstimate,  (int)&const_m_to_mm, (int)&v.temp,      //temp = velEst * 1000.0    (temp now in mm/sec)
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.VelocityEstMM, 


  //Create a fixed point version of the orientation matrix
        F32_opShift, (int)&v.m00,  (int)&const_16, (int)&v.temp,   
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm00, 
        F32_opShift, (int)&v.m01,  (int)&const_16, (int)&v.temp,   
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm01, 
        F32_opShift, (int)&v.m02,  (int)&const_16, (int)&v.temp,     
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm02, 

        F32_opShift, (int)&v.m10,  (int)&const_16, (int)&v.temp,   
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm10, 
        F32_opShift, (int)&v.m11,  (int)&const_16, (int)&v.temp,   
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm11, 
        F32_opShift, (int)&v.m12,  (int)&const_16, (int)&v.temp,   
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm12, 

        F32_opShift, (int)&v.m20,  (int)&const_16, (int)&v.temp,   
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm20, 
        F32_opShift, (int)&v.m21,  (int)&const_16, (int)&v.temp,   
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm21, 
        F32_opShift, (int)&v.m22,  (int)&const_16, (int)&v.temp,
        F32_opTruncRound, (int)&v.temp,  (int)&const_0, (int)&v.fm22,
        0, 0, 0, 0
        };
//}



void QuatIMU_InitFunctions(void)
{
  QuatIMU_AdjustStreamPointers( QuatUpdateCommands );
}


static int cycleTimer;

void QuatIMU_Update( int * packetAddr )
{
  memcpy( &v.gx, packetAddr, 11 * sizeof(int) );

  //Subtract gyro bias.  Probably better to do this in the sensor code, and ditto for accelerometer offset

  v.gx -= v.zx;
  v.gy -= v.zy;              
  v.gz -= v.zz;

  cycleTimer = CNT;
  F32::RunStream( QuatUpdateCommands );
}

int QuatIMU_WaitForCompletion(void)
{
  F32::WaitStream();	// Wait for the stream to complete
  return CNT - cycleTimer;
}
