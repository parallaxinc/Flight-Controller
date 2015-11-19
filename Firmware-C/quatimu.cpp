#include <propeller.h>

#include "constants.h"
#include "f32.h"
#include "quatimu.h"


#define RadToDeg (180.0 / 3.141592654)                         //Degrees per Radian
#define GyroToDeg  (1000.0 / 70.0)                             //Gyro units per degree @ 2000 deg/sec sens = 70 mdps/bit
#define AccToG  (float)(Const_OneG)                            //Accelerometer per G @ 8g sensitivity = ~0.24414 mg/bit 
#define GyroScale  (GyroToDeg * RadToDeg * (float)Const_UpdateRate)

static int  zx, zy, zz;                          // Gyro zero readings


// Working variables for the IMU code, in a struct so the compiler doesn't screw up the order,
// or remove some of them due to overly aggressive (IE wrong) optimization.

enum IMU_VarLabels {

	// INTEGER data -------------------

    ConstNull,                                   // Basically a placeholder for real zero / null

    Yaw,                                         // Current heading (yaw), scaled units
    ThrustFactor,
    FloatYaw,                                    // Current heading (yaw) in floating point
    HalfYaw,                                     // Heading / 2, used for quaternion construction

    DebugFloat,                                  // value used for debugging - sent to groundstation
    
    // Inputs
    gx, gy, gz,
    ax, ay, az,                                  // Sensor inputs
    mx, my, mz,
    alt, altRate,

    // Integer constants used in computation
    const_0,
    const_1,
    const_neg1,
    const_neg12,
    const_11,
    const_16,


	// FLOAT data ---------------------

	// Internal orientation storage
    qx, qy, qz, qw,                              // Body orientation quaternion
      
    m00, m01, m02,
    m10, m11, m12,                               // Body orientation as a 3x3 matrix
    m20, m21, m22,

    //Internal working variables - It isn't strictly necessary to break all of these out like this,
    //but it makes the code much more readable than having a bunch of temp variables
      
    qdx, qdy, qdz, qdw,                          // Incremental rotation quaternion

    fx2, fy2, fz2,
    fwx, fwy, fwz,                               // Quaternion to matrix temp coefficients
    fxy, fxz, fyz,

    rx, ry, rz,                                  // Float versions of rotation components
    fax, fay, faz,                               // Float version of accelerometer vector

    faxn, fayn, fazn,                            // Float version of accelerometer vector (normalized)
    rmag, cosr, sinr,                            // magnitude, cos, sin values
    
    errDiffX, errDiffY, errDiffZ,                // holds difference vector between target and measured orientation
    errCorrX, errCorrY, errCorrZ,                // computed rotation correction factor
    
    temp,                                        // temp value for use in equations

    axRot, ayRot, azRot,
    accWeight,

    accRollCorrSin,                              // used to correct the accelerometer vector angle offset
    accRollCorrCos,
    accPitchCorrSin,
    accPitchCorrCos,

      //Terms used in complementary filter to compute altitude from accelerometer and pressure sensor altitude
    velocityEstimate,
    altitudeVelocity,
    altitudeEstimate,
    AltitudeEstMM,
    VelocityEstMM,

    forceX, forceY, forceZ,              // Current forces acting on craft, excluding gravity
    forceWX, forceWY, forceWZ,           // Current forces acting on craft, excluding gravity, in world frame


    In_Elev, In_Aile, In_Rudd,          // Input values for user controls

    csx, csy, csz,                      // cosines of user control values
    snx, sny, snz,                      // sines of user control values

    snycsx, snysnx,                     // working variables for control to quaternion code (re-used products)
    csycsz, csysnz,

    cqw, cqx, cqy, cqz,                 // Control Quaternion result
    qrw, qrx, qry, qrz,                 // Rotation quaternion between CQ and current orientation (Q)

    diffAxisX, diffAxisY, diffAxisZ,    // Axis around which QR rotates to get from Q to CQ
    diffAngle,                          // Amount of rotation required to get from Q to CQ

    PitchDiff, RollDiff, YawDiff,       // Difference between current orientation and desired
    Heading,                            // Computed heading for control updates
    
    // Float constants used in computation
    const_GyroScale,
    const_NegGyroScale,

    const_F1,
    const_F2,
    const_NegF1,

    const_epsilon,
    
    const_half,
    const_neghalf,

    const_ErrScale,
    const_AccScale,
    const_outAngleScale,
    const_outNegAngleScale,
    const_ThrustScale,
    const_GMetersPerSec,
    const_AltiVelScale,
    const_UpdateScale,
    const_m_to_mm,

    const_velAccScale,
    const_velAltiScale,

    const_velAccTrust,
    const_velAltiTrust,

    const_YawRateScale,
    const_ManualYawScale,
    const_AutoBankScale,
    const_ManualBankScale,
    const_TwoPI,
    
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


#define PI  3.151592654


void QuatIMU_Start(void)
{
  memset( &IMU_VARS, 0, sizeof(IMU_VARS) );

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

  INT_VARS[const_0]                 =    0;
  INT_VARS[const_1]                 =    1;
  INT_VARS[const_neg1]              =   -1;
  INT_VARS[const_neg12]             =   -12;              //Used to subtract from acc exponent, equivalent to /= 4096.0
  INT_VARS[const_11]                =    11;              //Used to add to exponents, equivalent to *= 4096.0
  INT_VARS[const_16]                =    16;              //Used to add to exponents, equivalent to *= 65536.0

  IMU_VARS[const_F1]                =    1.0f;
  IMU_VARS[const_F2]                =    2.0f;
  IMU_VARS[const_NegF1]             =    -1.0f;
  
  IMU_VARS[const_epsilon]           =    0.00000001f;     //Added to vector length value before inverting (1/X) to insure no divide-by-zero problems
  IMU_VARS[const_half]              =    0.5f;
  IMU_VARS[const_neghalf]           =   -0.5f;


  IMU_VARS[const_ErrScale]          =    1.0f/512.0f;       //How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
  IMU_VARS[const_AccScale]          =    1.0f/(float)AccToG;//Conversion factor from accel units to G's
  IMU_VARS[const_outAngleScale]     =    65536.0f / PI;               
  IMU_VARS[const_outNegAngleScale]  =    -65536.0f / PI;               
  IMU_VARS[const_ThrustScale]       =    256.0f;
  IMU_VARS[const_GMetersPerSec]     =    9.80665f;
  IMU_VARS[const_AltiVelScale]      =    1.0/1000.0f;      //Convert mm to m
  IMU_VARS[const_UpdateScale]       =    1.0f / (float)Const_UpdateRate;    //Convert units/sec to units/update
  IMU_VARS[const_m_to_mm]           =    1000.0f;

  IMU_VARS[const_velAccScale]       =    0.9995f;
  IMU_VARS[const_velAltiScale]      =    0.0005f;

  IMU_VARS[const_velAccTrust]       =    0.999f;
  IMU_VARS[const_velAltiTrust]      =    0.001f;

  IMU_VARS[const_YawRateScale]      =    ((120.0f / 250.0f) / 1024.0f) * (PI/180.f) * 0.5f; // 120 deg/sec / UpdateRate * Deg2Rad * HalfAngle
  IMU_VARS[const_AutoBankScale]     =    (45.0f / 1024.0f) * (PI/180.0f) * 0.5f;

  IMU_VARS[const_ManualYawScale]    =    2.0f;
  IMU_VARS[const_ManualBankScale]   =    1.0f;

  IMU_VARS[const_TwoPI]             =    2.0f * PI;

  QuatIMU_InitFunctions();
}


int QuatIMU_GetYaw(void) {
  return INT_VARS[ Yaw ];
}

int QuatIMU_GetThrustFactor(void) {
  return INT_VARS[ ThrustFactor ];
}

int * QuatIMU_GetSensors(void) {
  return &INT_VARS[gx];
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
  //altitudeEstimate = F32_FDiv( F32_FFloat(altiMM) , const_m_to_mm );
    IMU_VARS[altitudeEstimate] = (float)altiMM / 1000.0;
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


void QuatIMU_AdjustStreamPointers( short * p )
{
    while( p[0] != 0 )
    {
        p[0] = (short)(int)F32::GetCommandPtr( p[0] );             // Convert the instruction index into the address of a jump table instruction
        if( p[1] != 0 )  p[1] = (short)(int)(IMU_VARS + p[1]);
        if( p[2] != 0 )  p[2] = (short)(int)(IMU_VARS + p[2]);
        if( p[3] != 0 )  p[3] = (short)(int)(IMU_VARS + p[3]);
       
        p += 4;
    }
}   



  //fgx = gx / GyroScale + errCorrX
              
short QuatUpdateCommands[] = {
        F32_opFloat, gx, 0, rx,                           //rx = float(gx)
        F32_opMul, rx, const_GyroScale, rx,               //rx /= GyroScale
        F32_opAdd, rx, errCorrX, rx,                      //rx += errCorrX

  //fgy = gy / GyroScale + errCorrY
        F32_opFloat, gz,  0, ry,                          //ry = float(gz)
        F32_opMul, ry, const_NegGyroScale, ry,            //ry /= GyroScale
        F32_opAdd, ry, errCorrY, ry,                      //ry += errCorrY

  //fgz = gz / GyroScale + errCorrZ
        F32_opFloat, gy, 0, rz,                           //rz = float(gy)
        F32_opMul, rz, const_NegGyroScale, rz,            //rz /= GyroScale
        F32_opAdd, rz, errCorrZ, rz,                      //rz += errCorrZ

  //rmag = sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) * 0.5
        F32_opSqr, rx, 0, rmag,                           //rmag = fgx*fgx
        F32_opSqr, ry, 0, temp,                           //temp = fgy*fgy
        F32_opAdd, rmag, temp, rmag,                      //rmag += temp
        F32_opSqr, rz, 0, temp,                           //temp = fgz*fgz
        F32_opAdd, rmag, temp, rmag,                      //rmag += temp
        F32_opAdd, rmag, const_epsilon, rmag,             //rmag += 0.00000001
        F32_opSqrt, rmag, 0, rmag,                        //rmag = Sqrt(rmag)                                                  
        F32_opShift, rmag, const_neg1, rmag,              //rmag *= 0.5                                                  
  //8 instructions  (17)

  //cosr = Cos(rMag)
  //sinr = Sin(rMag) / rMag
        F32_opSinCos, rmag,  sinr, cosr,           //sinr = Sin(rmag), cosr = Cos(rmag)  
        F32_opDiv, sinr,  rmag, sinr,              //sinr /= rmag                                                  
  //3 instructions  (20)

  //qdot.w =  (r.x*x + r.y*y + r.z*z) * -0.5
        F32_opMul, rx,  qx, qdw,                   //qdw = rx*qx 
        F32_opMul, ry,  qy, temp,                  //temp = ry*qy
        F32_opAdd, qdw,  temp, qdw,                //qdw += temp
        F32_opMul, rz,  qz, temp,                  //temp = rz*qz
        F32_opAdd, qdw,  temp, qdw,                //qdw += temp
        F32_opMul, qdw,  const_neghalf, qdw,       //qdw *= -0.5
  //8 instructions  (28)

  //qdot.x =  (r.x*w + r.z*y - r.y*z) * 0.5
        F32_opMul, rx,  qw, qdx,                   //qdx = rx*qw 
        F32_opMul, rz,  qy, temp,                  //temp = rz*qy
        F32_opAdd, qdx,  temp, qdx,                //qdx += temp
        F32_opMul, ry,  qz, temp,                  //temp = ry*qz
        F32_opSub, qdx,  temp, qdx,                //qdx -= temp
        F32_opShift, qdx,  const_neg1, qdx,        //qdx *= 0.5
  //8 instructions  (36)

  //qdot.y =  (r.y*w - r.z*x + r.x*z) * 0.5
        F32_opMul, ry,  qw, qdy,                   //qdy = ry*qw 
        F32_opMul, rz,  qx, temp,                  //temp = rz*qx
        F32_opSub, qdy,  temp, qdy,                //qdy -= temp
        F32_opMul, rx,  qz, temp,                  //temp = rx*qz
        F32_opAdd, qdy,  temp, qdy,                //qdy += temp
        F32_opShift, qdy,  const_neg1, qdy,        //qdy *= 0.5
  //8 instructions  (44)

  //qdot.z =  (r.z*w + r.y*x - r.x*y) * 0.5
        F32_opMul, rz,  qw, qdz,                   //qdz = rz*qw 
        F32_opMul, ry,  qx, temp,                  //temp = ry*qx
        F32_opAdd, qdz,  temp, qdz,                //qdz += temp
        F32_opMul, rx,  qy, temp,                  //temp = rx*qy
        F32_opSub, qdz,  temp, qdz,                //qdz -= temp
        F32_opShift, qdz,  const_neg1, qdz,        //qdz *= 0.5
  //8 instructions  (52)
   
  //q.w = cosr * q.w + sinr * qdot.w
        F32_opMul, cosr,  qw, qw,                  //qw = cosr*qw 
        F32_opMul, sinr,  qdw, temp,               //temp = sinr*qdw
        F32_opAdd, qw,  temp, qw,                  //qw += temp

  //q.x = cosr * q.x + sinr * qdot.x
        F32_opMul, cosr,  qx, qx,                  //qx = cosr*qx 
        F32_opMul, sinr,  qdx, temp,               //temp = sinr*qdx
        F32_opAdd, qx,  temp, qx,                  //qx += temp

  //q.y = cosr * q.y + sinr * qdot.y
        F32_opMul, cosr,  qy, qy,                  //qy = cosr*qy 
        F32_opMul, sinr,  qdy, temp,               //temp = sinr*qdy
        F32_opAdd, qy,  temp, qy,                  //qy += temp

  //q.z = cosr * q.z + sinr * qdot.z
        F32_opMul, cosr,  qz, qz,                  //qz = cosr*qz 
        F32_opMul, sinr,  qdz, temp,               //temp = sinr*qdz
        F32_opAdd, qz,  temp, qz,                  //qz += temp
  //12 instructions  (64)

  //q = q.Normalize()
  //rmag = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w + 0.0000001)
        F32_opSqr, qx,  0, rmag,                   //rmag = qx*qx 
        F32_opSqr, qy,  0, temp,                   //temp = qy*qy 
        F32_opAdd, rmag,  temp, rmag,              //rmag += temp 
        F32_opSqr, qz,  0, temp,                   //temp = qz*qz 
        F32_opAdd, rmag,  temp, rmag,              //rmag += temp 
        F32_opSqr, qw,  0, temp,                   //temp = qw*qw 
        F32_opAdd, rmag,  temp, rmag,              //rmag += temp 
        F32_opAdd, rmag,  const_epsilon, rmag,     //rmag += 0.0000001 
        F32_opSqrt, rmag,  0, rmag,                //sqrt(rmag) 
  //9 instructions (73)

  //q /= rmag   
        F32_opDiv, qw,  rmag, qw,                  //qw /= rmag 
        F32_opDiv, qx,  rmag, qx,                  //qx /= rmag 
        F32_opDiv, qy,  rmag, qy,                  //qy /= rmag 
        F32_opDiv, qz,  rmag, qz,                  //qz /= rmag 
  //4 instructions (77)


  //Now convert the updated quaternion to a rotation matrix 

        F32_opSqr, qx,  0, fx2,                    //fx2 = qx *qx
        F32_opSqr, qy,  0, fy2,                    //fy2 = qy *qy
        F32_opSqr, qz,  0, fz2,                    //fz2 = qz *qz
  //3 instructions (80)

        F32_opMul, qw,  qx, fwx,                   //fwx = qw *qx
        F32_opMul, qw,  qy, fwy,                   //fwy = qw *qy
        F32_opMul, qw,  qz, fwz,                   //fwz = qw *qz
  //3 instructions (83)

        F32_opMul, qx,  qy, fxy,                   //fxy = qx *qy
        F32_opMul, qx,  qz, fxz,                   //fxz = qx *qz
        F32_opMul, qy,  qz, fyz,                   //fyz = qy *qz
  //3 instructions (86)

   
  //m00 = 1.0f - 2.0f * (y2 + z2)
        F32_opAdd, fy2,  fz2, temp,                //temp = fy2+fz2
        F32_opShift, temp,  const_1, temp,         //temp *= 2.0
        F32_opSub, const_F1,  temp, m00,           //m00 = 1.0 - temp
     
  //m01 =        2.0f * (fxy - fwz)
        F32_opSub, fxy,  fwz, temp,                //temp = fxy-fwz
        F32_opShift, temp,  const_1, m01,          //m01 = 2.0 * temp

  //m02 =        2.0f * (fxz + fwy)
        F32_opAdd, fxz,  fwy, temp,                //temp = fxz+fwy
        F32_opShift, temp,  const_1, m02,          //m02 = 2.0 * temp
  //7 instructions (93)

   
  //m10 =        2.0f * (fxy + fwz)
        F32_opAdd, fxy,  fwz, temp,                //temp = fxy-fwz
        F32_opShift, temp,  const_1, m10,          //m10 = 2.0 * temp

  //m11 = 1.0f - 2.0f * (x2 + z2)
        F32_opAdd, fx2,  fz2, temp,                //temp = fx2+fz2
        F32_opShift, temp,  const_1, temp,         //temp *= 2.0
        F32_opSub, const_F1,  temp, m11,           //m11 = 1.0 - temp

  //m12 =        2.0f * (fyz - fwx)
        F32_opSub, fyz,  fwx, temp,                //temp = fyz-fwx
        F32_opShift, temp,  const_1, m12,          //m12 = 2.0 * temp
  //7 instructions (100)

   
  //m20 =        2.0f * (fxz - fwy)
        F32_opSub, fxz,  fwy, temp,                //temp = fxz-fwz
        F32_opShift, temp,  const_1, m20,          //m20 = 2.0 * temp

  //m21 =        2.0f * (fyz + fwx)
        F32_opAdd, fyz,  fwx, temp,                //temp = fyz+fwx
        F32_opShift, temp,  const_1, m21,          //m21 = 2.0 * temp

  //m22 = 1.0f - 2.0f * (x2 + y2)
        F32_opAdd, fx2,  fy2, temp,                //temp = fx2+fy2
        F32_opShift, temp,  const_1, temp,         //temp *= 2.0
        F32_opSub, const_F1,  temp, m22,           //m22 = 1.0 - temp
  //7 instructions (107)



  //fax =  packet.ax;           // Acceleration in X (left/right)
  //fay =  packet.az;           // Acceleration in Y (up/down)
  //faz =  packet.ay;           // Acceleration in Z (toward/away)
        F32_opFloat, ax,  0, fax,
        F32_opFloat, az,  0, fay,
        F32_opFloat, ay,  0, faz,
        F32_opNeg,  fax,  0, fax,


//Rotation correction of the accelerometer vector - rotate around the pitch and roll axes by the specified amounts

  //axRot = (fax * accRollCorrCos) - (fay * accRollCorrSin)
        F32_opMul, fax,  accRollCorrCos, axRot,
        F32_opMul, fay,  accRollCorrSin, temp,
        F32_opSub, axRot,  temp, axRot,

  //ayRot = (fax * accRollCorrSin) + (fay * accRollCorrCos)
        F32_opMul, fax,  accRollCorrSin, ayRot,
        F32_opMul, fay,  accRollCorrCos, temp,
        F32_opAdd, ayRot,  temp, ayRot,

  //fax = axRot         
  //fay = ayRot
        F32_opMov, axRot,  0, fax,
        F32_opMov, ayRot,  0, fay,



  //axRot = (faz * accPitchCorrCos) - (fay * accPitchCorrSin)
        F32_opMul, faz,  accPitchCorrCos, axRot,
        F32_opMul, fay,  accPitchCorrSin, temp,
        F32_opSub, axRot,  temp, axRot, 

  //ayRot = (fax * accPitchCorrSin) + (fay * accPitchCorrCos)
        F32_opMul, faz,  accPitchCorrSin, ayRot,                           
        F32_opMul, fay,  accPitchCorrCos, temp,
        F32_opAdd, ayRot,  temp, ayRot,

  //faz = axRot         
  //fay = ayRot
        F32_opMov, axRot,  0, faz,          
        F32_opMov, ayRot,  0, fay,          



//Compute length of the accelerometer vector to decide weighting                                   

  //rmag = facc.length
        F32_opSqr, fax,  0, rmag,                  //rmag = fax*fax
        F32_opSqr, fay,  0, temp,                  //temp = fay*fay
        F32_opAdd, rmag,  temp, rmag,              //rmag += temp
        F32_opSqr, faz,  0, temp,                  //temp = faz*faz
        F32_opAdd, rmag,  temp, rmag,              //rmag += temp
        F32_opAdd, rmag,  const_epsilon, rmag,     //rmag += 0.00000001
        F32_opSqrt, rmag,  0, rmag,                //rmag = Sqrt(rmag)                                                  

  //facc /= rmag
        F32_opDiv, fax,  rmag, faxn,               //faxn = fax / rmag 
        F32_opDiv, fay,  rmag, fayn,               //fayn = fay / rmag 
        F32_opDiv, faz,  rmag, fazn,               //fazn = faz / rmag 



  //accWeight = 1.0 - FMin( FAbs( 2.0 - accLen * 2.0 ), 1.0 )
        F32_opMul, rmag,  const_AccScale, rmag,    //rmag /= accScale (accelerometer to 1G units)
        F32_opShift, rmag,  const_1, accWeight,    //accWeight = rmag * 2.0
        F32_opSub, const_F2,  accWeight, accWeight,//accWeight = 2.0 - accWeight
        F32_opFAbs, accWeight,  0, accWeight,      //accWeight = FAbs(accWeight)
        F32_opFMin, accWeight, const_F1, accWeight,//accWeight = FMin( accWeight, 1.0 )
        F32_opSub, const_F1,  accWeight, accWeight,//accWeight = 1.0 - accWeight                                                

   

  //errDiffX = fayn * m12 - fazn * m11
        F32_opMul, fayn,  m12, errDiffX, 
        F32_opMul, fazn,  m11, temp, 
        F32_opSub, errDiffX,  temp, errDiffX, 

  //errDiffY = fazn * m10 - faxn * m12
        F32_opMul, fazn,  m10, errDiffY, 
        F32_opMul, faxn,  m12, temp, 
        F32_opSub, errDiffY,  temp, errDiffY, 

  //errDiffZ = faxn * m11 - fayn * m10
        F32_opMul, faxn,  m11, errDiffZ, 
        F32_opMul, fayn,  m10, temp,
        F32_opSub, errDiffZ,  temp, errDiffZ, 

  //accWeight *= const_ErrScale   
        F32_opMul, const_ErrScale,  accWeight, accWeight,

  //Test: Does ErrCorr need to be rotated into the local frame from the world frame?


  //errCorr = errDiff * accWeight
        F32_opMul, errDiffX,  accWeight, errCorrX,  
        F32_opMul, errDiffY,  accWeight, errCorrY,  
        F32_opMul, errDiffZ,  accWeight, errCorrZ,  


    //For heading, I want an actual angular value, so this returns me an int between 0  65535, where 0 is forward
    //YAngle := Flt.FRound( Flt.FMul( Flt.Atan2( Flt.FFloat(DCM.GetM20), Flt.FFloat(DCM.GetM22)), constant(32768.0 / PI) ) )  65535 

        
        F32_opATan2, m20,  m22, FloatYaw,
        F32_opNeg, FloatYaw, 0, FloatYaw,
        F32_opMul, FloatYaw,  const_outAngleScale, temp,
        F32_opTruncRound, temp,  const_0, Yaw,

        // When switching between manual and auto, or just lifting off, I need to
        // know the half-angle of the craft so I can use it as my initial Heading value
        // to be fed into the quaternion construction code.  This HalfYaw value serves that purpose
        F32_opShift, FloatYaw, const_neg1, HalfYaw,


        F32_opDiv, const_F1,  m11, temp,                          // 1.0/m11 = scale factor for thrust - this will be infinite if perpendicular to ground   
        F32_opMul, temp,  const_ThrustScale, temp,                // *= 256.0  
        F32_opTruncRound, temp,  const_0, ThrustFactor,  



  //Compute the running height estimate

  //force := acc / 4096.0
        F32_opShift, fax,  const_neg12, forceX,
        F32_opShift, fay,  const_neg12, forceY,
        F32_opShift, faz,  const_neg12, forceZ,

  //force -= m[1,0], m[1,1], m[1,2]  - Subtract gravity (1G, straight down)
        F32_opSub, forceX,  m10, forceX,    
        F32_opSub, forceY,  m11, forceY,    
        F32_opSub, forceZ,  m12, forceZ,    

  //forceWY := M.Transpose().Mul(Force).y                 //Orient force vector into world frame
  //forceWY = m01*forceX + m11*forceY + m21*forceZ

        F32_opMul, forceX,  m01, forceWY,  
   
        F32_opMul, forceY,  m11, temp,  
        F32_opAdd, forceWY, temp, forceWY,  

        F32_opMul, forceZ,  m21, temp,  
        F32_opAdd, forceWY, temp, forceWY,  

  //forceWY *= 9.8                                       //Convert to M/sec^2
        F32_opMul, forceWY,  const_GMetersPerSec, forceWY,  



        F32_opMul, forceWY,  const_UpdateScale, temp,            //temp := forceWY / UpdateRate
        F32_opAdd, velocityEstimate,  temp, velocityEstimate,     //velEstimate += forceWY / UpdateRate

  
        F32_opFloat, altRate,  0, altitudeVelocity,                //AltVelocity = float(altRate)
        F32_opMul, altitudeVelocity,  const_AltiVelScale, altitudeVelocity,   //Convert from mm/sec to m/sec   


  //VelocityEstimate := (VelocityEstimate * 0.9950) + (altVelocity * 0.0050)
        F32_opMul, velocityEstimate,  const_velAccScale, velocityEstimate, 
        F32_opMul, altitudeVelocity,  const_velAltiScale, temp,  
        F32_opAdd, velocityEstimate,  temp, velocityEstimate,   

  //altitudeEstimate += velocityEstimate / UpdateRate
        F32_opMul, velocityEstimate,  const_UpdateScale, temp , 
        F32_opAdd, altitudeEstimate,  temp, altitudeEstimate,   

  //altitudeEstimate := (altitudeEstimate * 0.9950) * (alti / 1000.0) * 0.0050
        F32_opMul, altitudeEstimate,  const_velAccTrust, altitudeEstimate, 

        F32_opFloat, alt,  0, temp,                             //temp := float(alt)
        F32_opDiv, temp,  const_m_to_mm, temp,                  //temp /= 1000.0    (alt now in m)
        F32_opMul, temp,  const_velAltiTrust, temp,             //temp *= 0.0050
        F32_opAdd, altitudeEstimate,  temp, altitudeEstimate,   //altEstimate += temp 


        F32_opMul, altitudeEstimate,  const_m_to_mm, temp,      //temp = altEst * 1000.0    (temp now in mm)
        F32_opTruncRound, temp,  const_0, AltitudeEstMM, 

        F32_opMul, velocityEstimate,  const_m_to_mm, temp,      //temp = velEst * 1000.0    (temp now in mm/sec)
        F32_opTruncRound, temp,  const_0, VelocityEstMM, 

        0, 0, 0, 0
        };
//}



short UpdateControls_Manual[] = {

  // float xrot = (float)radio.Elev * const_ManualBankScale;	// Individual scalars for channel sensitivity
  // float yrot = (float)radio.Rudd * const_ManualBankScale;
  // float zrot = (float)radio.Aile * -const_ManualBankScale;

  F32_opFloat,  In_Elev, 0, In_Elev,                  // Elev to float
  F32_opFloat,  In_Aile, 0, In_Aile,                  // Aile to float
  F32_opFloat,  In_Rudd, 0, In_Rudd,                  // Rudd to float

  F32_opMul,    In_Elev, const_ManualBankScale, rx,   // rx = (Elev scaled to incremental update angle)
  F32_opMul,    In_Aile, const_ManualBankScale, rz,   // rz = (Aile scaled to incremental update angle)
  F32_opMul,    In_Rudd, const_ManualYawScale, ry,    // Scale rudd by maximum yaw rate scale

  F32_opNeg,    rx, 0, rx,
  F32_opNeg,    ry, 0, ry,

  F32_opTruncRound, rx, const_0, PitchDiff,
  F32_opTruncRound, rz, const_0, RollDiff,
  F32_opTruncRound, ry, const_0, YawDiff,

  0, 0, 0, 0,
};


short UpdateControlQuaternion_AutoLevel[] = {

  // Convert radio inputs to float, scale them to get them into the range we want

  F32_opFloat,  In_Elev, 0, In_Elev,                  // Elev to float
  F32_opFloat,  In_Aile, 0, In_Aile,                  // Aile to float
  F32_opFloat,  In_Rudd, 0, In_Rudd,                  // Rudd to float

  F32_opMul,    In_Elev, const_AutoBankScale, rx,     // rx = (Elev scaled to bank angle)
  F32_opMul,    In_Aile, const_AutoBankScale, rz,     // rz = (Aile scaled to bank angle)
  F32_opNeg,    rz, 0, rz,                            // rz = -rz

  F32_opMul,    In_Rudd, const_YawRateScale, In_Rudd, // Scale rudd by maximum yaw rate scale
  F32_opAdd,    Heading, In_Rudd, Heading,            // Add scaled rudd to desired Heading   (may need to range check - keep in +/- PI ?)

  // Keep Heading in the range of -PI to PI
  F32_opDiv,    Heading, const_TwoPI, temp,           // temp = Heading/PI
  F32_opTruncRound, temp, const_0, temp,              // temp = (int)(Heading/PI)
  F32_opFloat,  temp, 0, temp,                        // temp = (float)((int)(Heading/PI))
  F32_opMul,    temp, const_TwoPI, temp,              // temp is now the integer multiple of PI in heading
  F32_opSub,    Heading, temp, Heading,               // Remove the part that's out of range


  // Compute sines and cosines of scaled control input values

  F32_opSinCos, rx, snx, csx,                         // snx = Sin(rx), csx = Cos(rx)
  F32_opSinCos, Heading, sny, csy,                    // sny = Sin(ry), csy = Cos(ry)   (ry is heading)
  F32_opSinCos, rz, snz, csz,                         // snz = Sin(rz), csz = Cos(rz)

  // Pre-compute some re-used terms to save computation time

  F32_opMul,    sny, csx, snycsx,                     // snycsx = sny * csx
  F32_opMul,    sny, snx, snysnx,                     // snysnx = sny * snx
  F32_opMul,    csy, csz, csycsz,                     // csycsz = csy * csz
  F32_opMul,    csy, snz, csysnz,                     // scssnz = csy * snz

  // Compute the quaternion that represents our new desired orientation  ((CQ = Control Quaternion))

  F32_opMul,    snycsx, snz, cqx,                     // cqx =  snycsx * snz + csycsz * snx
  F32_opMul,    csycsz, snx, temp,
  F32_opAdd,    cqx, temp, cqx,

  F32_opMul,    snycsx, csz, cqy,                     // cqy =  snycsx * csz + csysnz * snx
  F32_opMul,    csysnz, snx, temp,
  F32_opAdd,    cqy, temp, cqy,

  F32_opMul,    csysnz, csx, cqz,                     // cqz = -snysnx * csz + csysnz * csx
  F32_opMul,    snysnx, csz, temp,
  F32_opSub,    cqz, temp, cqz,

  F32_opMul,    csycsz, csx, cqw,                     // cqw = -snysnx * snz + csycsz * csx
  F32_opMul,    snysnx, snz, temp,
  F32_opSub,    cqw, temp, cqw,


  //---------------------------------------------------------------------------
  // Compute the quaternion which is the rotation from our current orientation (Q)
  // to our desired one (CQ)
  //
  // IE, QR = rotation from (Q) to (CQ)
  // Computation is QR = CQ.Conjugate() * Q,  where Conjugate is just (w, -x, -y, -z)
  //---------------------------------------------------------------------------

  // With all the appropriate sign flips, the formula becomes:

  // qrx = -cqx * qw - cqy * qz + cqz * qy + cqw * qx;
  // qry =  cqx * qz - cqy * qw - cqz * qx + cqw * qy;
  // qrz = -cqx * qy + cqy * qx - cqz * qw + cqw * qz;
  // qrw =  cqx * qx + cqy * qy + cqz * qz + cqw * qw;
  
  

  // qrx = -cqx * qw - cqy * qz + cqz * qy + cqw * qx;

  F32_opMul,    cqx, qw, qrx,           // qrx =  cqx*qw
  F32_opNeg,    qrx, 0, qrx,            // qrx = -qrx
  F32_opMul,    cqy, qz, temp,
  F32_opSub,    qrx, temp, qrx,         // qrx -= cqy*qz
  F32_opMul,    cqz, qy, temp,
  F32_opAdd,    qrx, temp, qrx,         // qrx += cqz*qy
  F32_opMul,    cqw, qx, temp,
  F32_opAdd,    qrx, temp, qrx,         // qrx += cqw*qx


  // qry =  cqx * qz - cqy * qw - cqz * qx + cqw * qy;

  F32_opMul,    cqx, qz, qry,           // qry =  cqx*qz
  F32_opMul,    cqy, qw, temp,
  F32_opSub,    qry, temp, qry,         // qry -= cqy*qw
  F32_opMul,    cqz, qx, temp,
  F32_opSub,    qry, temp, qry,         // qry -= cqz*qx
  F32_opMul,    cqw, qy, temp,
  F32_opAdd,    qry, temp, qry,         // qry += cqw*qy

  
  // qrz = -cqx * qy + cqy * qx - cqz * qw + cqw * qz;

  F32_opMul,    cqx, qy, qrz,           // qrz =  cqx*qy
  F32_opNeg,    qrz, 0, qrz,            // qrz = -qrz
  F32_opMul,    cqy, qx, temp,
  F32_opAdd,    qrz, temp, qrz,         // qrz += cqy*qx
  F32_opMul,    cqz, qw, temp,
  F32_opSub,    qrz, temp, qrz,         // qrz -= cqz*qw
  F32_opMul,    cqw, qz, temp,
  F32_opAdd,    qrz, temp, qrz,         // qrz += cqw*qz


  // qrw =  cqx * qx + cqy * qy + cqz * qz + cqw * qw;

  F32_opMul,    cqx, qx, qrw,           // qrw =  cqx*qx
  F32_opMul,    cqy, qy, temp,
  F32_opAdd,    qrw, temp, qrw,         // qrw += cqy*qy
  F32_opMul,    cqz, qz, temp,
  F32_opAdd,    qrw, temp, qrw,         // qrw += cqz*qz
  F32_opMul,    cqw, qw, temp,
  F32_opAdd,    qrw, temp, qrw,         // qrw += cqw*qw


  F32_opCmp,    qrw, const_0, temp,     // qrw < 0?

  // Conditionally negate QR if QR.w < 0
  F32_opCNeg,   qrw, temp, qrw,
  F32_opCNeg,   qrx, temp, qrx,
  F32_opCNeg,   qry, temp, qry,
  F32_opCNeg,   qrz, temp, qrz,

  // float diffAngle = qrot.ToAngleAxis( out DiffAxis );

  // Converts to:

  // float diffAngle = 2.0f * Acos(qrw);
  // float rmag = Sqrt( 1.0f - qrw*qrw );	  // assuming quaternion normalised then w is less than 1, so term always positive.
  // rmag = min( rmag, 0.001 )
  // DiffAxis.x = qrx / rmag; // normalise axis
  // DiffAxis.y = qry / rmag;
  // DiffAxis.z = qrz / rmag;
  
  // PitchDiff = DiffAxis.x * diffAngle
  // RollDiff =  DiffAxis.z * diffAngle
  // YawDiff =   DiffAxis.y * diffAngle


  // float diffAngle = 2.0f * Acos(qrw);
  F32_opFMin,   qrw, const_F1, qrw,        // clamp qrw to -1.0 to +1.0 range
  F32_opNeg,    qrw, 0, qrw,
  F32_opFMin,   qrw, const_F1, qrw,        // clamp qrw to -1.0 to +1.0 range
  F32_opNeg,    qrw, 0, qrw,

  F32_opASinCos, qrw, const_0, diffAngle,
  F32_opShift,  diffAngle, const_1, diffAngle,         // diffAngle *= 2.0

  F32_opMov,    diffAngle, 0, DebugFloat,

  
  // float rmag = Sqrt( 1.0f - qrw*qrw );	  // assuming quaternion normalised then w is less than 1, so term always positive.
  F32_opSqr,    qrw, 0, temp,
  F32_opSub,    const_F1, temp, temp,

  F32_opNeg,    temp, 0, temp,
  F32_opFMin,   temp, 0, temp,              // make sure temp is >= 0.0 (don't have FMax, so negate, use FMin, negate again)
  F32_opNeg,    temp, 0, temp,

  F32_opSqrt,   temp, 0, rmag,


  // rmag = max( rmag, 0.0000001 )
  F32_opAdd,    rmag, const_epsilon, rmag,
  F32_opDiv,    diffAngle, rmag, rmag,        // rmag = (1.0/rmag * diffAngle)  equivalent to rmag = (diffAngle / rmag)
  F32_opShift,  rmag, const_11, rmag,         // rmag *= 2048

  // Simplified this a little by changing  X / rmag * diffAngle into X * (1.0/rmag * diffAngle)
  // PitchDiff = qrx / rmag * diffAngle
  // RollDiff =  qry / rmag * diffAngle
  // YawDiff =   qrz / rmag * diffAngle

  F32_opMul,    qrx, rmag, PitchDiff,
  F32_opMul,    qrz, rmag, RollDiff,
  F32_opMul,    qry, rmag, YawDiff,


  F32_opTruncRound, PitchDiff, const_0, PitchDiff,
  F32_opTruncRound, RollDiff, const_0, RollDiff,
  F32_opTruncRound, YawDiff, const_0, YawDiff,  

  0, 0, 0, 0
};



void QuatIMU_InitFunctions(void)
{
  QuatIMU_AdjustStreamPointers( QuatUpdateCommands );
  QuatIMU_AdjustStreamPointers( UpdateControls_Manual );
  QuatIMU_AdjustStreamPointers( UpdateControlQuaternion_AutoLevel );
}


static int cycleTimer;


void QuatIMU_Update( int * packetAddr )
{
  memcpy( &IMU_VARS[gx], packetAddr, 11 * sizeof(int) );

  //Subtract gyro bias.  Probably better to do this in the sensor code, and ditto for accelerometer offset

  ((int*)IMU_VARS)[gx] -= zx;
  ((int*)IMU_VARS)[gy] -= zy;
  ((int*)IMU_VARS)[gz] -= zz;

  cycleTimer = CNT;
  F32::RunStream( QuatUpdateCommands );
}

inline static int abs( int v )
{
  return (v < 0) ? -v : v;
}

void QuatIMU_UpdateControls( RADIO * Radio , int ManualMode )
{
  ((int*)IMU_VARS)[In_Elev] = Radio->Elev;
  ((int*)IMU_VARS)[In_Aile] = Radio->Aile;
  ((int*)IMU_VARS)[In_Rudd] = Radio->Rudd;

  /*
  if( abs(INT_VARS[In_Elev]) < 10 ) {
    INT_VARS[In_Elev] = 0;
  }

  if( abs(INT_VARS[In_Aile]) < 10 ) {
    INT_VARS[In_Aile] = 0;
  }

  if( abs(INT_VARS[In_Rudd]) < 10 ) {
    INT_VARS[In_Rudd] = 0;
  }
  */

  cycleTimer = CNT;

  if( ManualMode ) {
    F32::RunStream( UpdateControls_Manual );
  }
  else {
    F32::RunStream( UpdateControlQuaternion_AutoLevel );
  }
}


int QuatIMU_WaitForCompletion(void)
{
  F32::WaitStream();    // Wait for the stream to complete
  return CNT - cycleTimer;
}
