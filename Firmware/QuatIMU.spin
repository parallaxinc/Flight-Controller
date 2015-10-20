{
  QuatIMU -
}


OBJ
  FLT           : "F32_1_6"
  Const         : "Constants.spin"

  'Debug         : "FullDuplexSerial"

CON
  _clkmode = xtal1 + pll16x                             'Standard clock mode * crystal frequency = 80 MHz
  _xinfreq = 5_000_000

  RadToDeg = 180.0 / 3.141592654                        'Degrees per Radian
  GyroToDeg = 1000.0 / 70.0                             'Gyro units per degree @ 2000 deg/sec sens = 70 mdps/bit
  AccToG = float(Const#OneG)                            'Accelerometer per G @ 8g sensitivity = ~0.24414 mg/bit 

  GyroScale = GyroToDeg * RadToDeg * float(Const#UpdateRate)

  
DAT
 'These would normally be declared in a VAR section, but there will only ever be one IMU object, and
 'the way the FPU instruction stream is built at compile time requires DAT variables so the memory addresses
 'can be resolved at compile time too.

  Roll                  long    0
  Pitch                 long    0
  Yaw                   long    0
  ThrustFactor          long    0                       'Outputs, scaled units

  'Inputs
  zx                    long    0
  zy                    long    0
  zz                    long    0                       'Gyro zero readings
  gx                    long    0
  gy                    long    0
  gz                    long    0
  ax                    long    0
  ay                    long    0
  az                    long    0
  mx                    long    0
  my                    long    0
  mz                    long    0
  alt                   long    0
  altRate               long    0                       'Sensor inputs

  'Internal orientation storage
  qx                    long    0
  qy                    long    0
  qz                    long    0
  qw                    long    0                       'Body orientation quaternion
  
                        long    0
  m00                   long    0
  m01                   long    0
  m02                   long    0                       'Body orientation as a 3x3 matrix
  m10                   long    0
  m11                   long    0
  m12                   long    0
  m20                   long    0
  m21                   long    0
  m22

  fm00                  long    0
  fm01                  long    0
  fm02                  long    0                       'Body orientation as a 3x3 matrix in fixed integer form (+/- 65536 == +/- 1.0)
  fm10                  long    0
  fm11                  long    0
  fm12                  long    0
  fm20                  long    0
  fm21                  long    0
  fm22                  long    0
  
  'Internal working variables - It isn't strictly necessary to break all of these out like this,
  'but it makes the code much more readable than having a bunch of temp variables
  
  qdx                   long    0
  qdy                   long    0
  qdz                   long    0
  qdw                   long    0                       'Incremental rotation quaternion
  fx2                   long    0
  fy2                   long    0
  fz2                   long    0
  fwx                   long    0
  fwy                   long    0
  fwz                   long    0
  fxy                   long    0
  fxz                   long    0
  fyz                   long    0                       'Quaternion to matrix temp coefficients

  rx                    long    0
  ry                    long    0
  rz                    long    0                       'Float versions of rotation components

  fax                   long    0
  fay                   long    0
  faz                   long    0                       'Float version of accelerometer vector

  faxn                  long    0
  fayn                  long    0
  fazn                  long    0                       'Float version of accelerometer vector (normalized)
  
  rmag                  long    0
  cosr                  long    0
  sinr                  long    0                       'magnitude, cos, sin values
  
  errDiffX              long    0
  errDiffY              long    0
  errDiffZ              long    0                       'holds difference vector between target and measured orientation
  
  errCorrX              long    0
  errCorrY              long    0
  errCorrZ              long    0                       'computed rotation correction factor
  
  temp                  long    0                       'temp value for use in equations
  axRot                 long    0                                
  ayRot                 long    0                                
  azRot                 long    0                                
  accWeight             long    0

  accRollCorrSin        long    0.0                     'used to correct the accelerometer vector angle offset
  accRollCorrCos        long    1.0
  accPitchCorrSin       long    0.0
  accPitchCorrCos       long    1.0  

  'Terms used in complementary filter to compute altitude from accelerometer and pressure sensor altitude
  velocityEstimate      long    0
  altitudeVelocity      long    0 
  altitudeEstimate      long    0
  AltitudeEstMM         long    0
  VelocityEstMM         long    0
  
  forceX                long    0
  forceY                long    0
  forceZ                long    0                       'Current forces acting on craft, excluding gravity

  forceWX               long    0
  forceWY               long    0
  forceWZ               long    0                       'Current forces acting on craft, excluding gravity, in world frame

  UpdateCount           long    0
  

VAR
  'long  QuatUpdateCommands[300 + 284 + 310]
  'long  QuatUpdateLen
  

PUB Start
  FLT.Start
  'Debug.start(31, 30, 0, 115200)
  UpdateCount := 0

  qx := 0.0
  qy := 0.0
  qz := 0.0
  qw := 1.0

  velocityEstimate := 0.0
  altitudeEstimate := 0.0
  
  InitFunctions


PUB GetPitch
  return Pitch

PUB GetRoll
  return Roll

PUB GetYaw
  return Yaw

PUB GetThrustFactor
  return ThrustFactor

PUB GetSensors
  return @gx

PUB GetMatrix
  return @m00

PUB GetFixedMatrix
  return @fm00

PUB GetQuaternion
  return @qx

PUB GetVerticalVelocityEstimate
  return VelocityEstMM

PUB GetAltitudeEstimate
  return AltitudeEstMM    

PUB SetInitialAltitudeGuess( altiMM )
  altitudeEstimate := FLT.FDiv( FLT.FFloat(altiMM) , const_m_to_mm )


PUB SetRollCorrection( addr )

  accRollCorrSin := long[addr][0]                   
  accRollCorrCos := long[addr][1]                   

PUB SetPitchCorrection( addr )

  accPitchCorrSin := long[addr][0]                   
  accPitchCorrCos := long[addr][1]                   


PUB InitFunctions

  AdjustStreamPointers( @commandStream_0 ) 


PUB SetGyroZero( _x, _y, _z )
  zx := _x
  zy := _y
  zz := _z
  

PUB Send( v )
  'Debug.tx(v >> 8)
  'Debug.tx(v & 255)



'If the body is mostly level, use the incoming mag readings as the compass vector
'Compute the difference between the actual compass heading and estimated heading,
'and apply that as an additional part of the errCorr vector (?)


PUB Update_Part1( packetAddr ) | v, t


  LongMove( @gx, packetAddr, 11 )

  'Subtract gyro bias.  Probably better to do this in the sensor code, and ditto for accelerometer offset

  gx -= zx
  gy -= zy              
  gz -= zz


  t := cnt
  FLT.RunStream( @QuatUpdateCommands )
  t := cnt-t                    'Resulting t value is the number of cycles taken to execute the IMU code 

  return t     



PUB WaitForCompletion
  FLT.WaitStream                'Wait for the stream to complete



  'Send matrix outputs to Visualizer
  {
  'Only transmit the complete matrix every 8 updates (500Hz/8 = 62.5Hz), and do it in two parts to keep the speed up
  if( UpdateCount == 0 )
    Send( $7878 )
    Send( FLT.FTrunc( FLT.FMul(m00, 16383.0)) )
    Send( FLT.FTrunc( FLT.FMul(m01, 16383.0)) )
    Send( FLT.FTrunc( FLT.FMul(m02, 16383.0)) )
  if( UpdateCount == 1 )
    Send( FLT.FTrunc( FLT.FMul(m10, 16383.0)) )
    Send( FLT.FTrunc( FLT.FMul(m11, 16383.0)) )
    Send( FLT.FTrunc( FLT.FMul(m12, 16383.0)) )
  '}
  
    

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


PRI AdjustStreamPointers( p ) | diff
  ' Make sure the subroutine doesn't get called for the same table twice
  ' The first pointer in the table points to itself, let's see if it's already correct
  if long[p] == p
    return

  diff := @@long[p] - long[p]
  long[p] += diff             'fix the table startup pointer
  p += 4                      'advance to the first instruction in the stream                                    
   
  repeat until word[p] == 0                             'Zero indicates end-of-stream
    word[p] := FLT.GetCommandPtr( word[p] )             'Turn this into a memory address into the JMPRET instruction table in the FPU
    p += 2                                              'Advance to the first set of arguments
    word[p] += diff                                     'Replace the relative pointer (word) with an absolute pointer
    p += 2                                              'advance to the next instruction in the list      
    word[p] += diff                                     'Replace the relative pointer (word) with an absolute pointer
    p += 2                                              'advance to the next instruction in the list      
    word[p] += diff                                     'Replace the relative pointer (word) with an absolute pointer
    p += 2                                              'advance to the next instruction in the list      
   
   

DAT

'Various constants used by the float math engine - Every command in the instruction stream reads two
'arguments from memory using memory addresses, so the values actually need to exist somewhere

const_GyroScale         long    1.0 / GyroScale    
const_NegGyroScale      long   -1.0 / GyroScale

const_0                 long    0
const_1                 long    1
const_neg1              long   -1
const_Neg12             long   -12              'Used to subtract from acc exponent, equivalent to /= 4096.0
const_16                long    16              'Used to add to exponents, equivalent to *= 65536.0
const_F1                long    1.0
const_F2                long    2.0

const_epsilon           long    0.0000000001    'Added to vector length value before inverting (1/X) to insure no divide-by-zero problems
const_half              long    0.5
const_neghalf           long   -0.5


const_ErrScale          long    1.0/512.0       'How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
const_AccScale          long    1.0/AccToG      'Conversion factor from accel units to G's
const_outAngleScale     long    65536.0 / PI               
const_outNegAngleScale  long    -65536.0 / PI               
const_ThrustScale       long    256.0
const_GMetersPerSec     long    9.80665
const_AltiVelScale      long    1.0/1000.0      'Convert mm to m
const_UpdateScale       long    1.0 / float(Const#UpdateRate) 'Convert units/sec to units/update
const_m_to_mm           long    1000.0

const_velAccScale       long    0.9995
const_velAltiScale      long    0.0005
 
const_velAccTrust       long    0.999
const_velAltiTrust      long    0.001  




'{

commandStream_0
        long  @commandStream_0                'this value is used to determine the offset to add to subsequent instructions                                                
                                                        'SEE:  http://forums.parallax.com/discussion/148580/fyi-using-the-operator-in-a-dat-section
  'fgx = gx / GyroScale + errCorrX
              
QuatUpdateCommands
        word  FLT#opFloat, @gx, 0, @rx                         'rx = float(gx)
        word  FLT#opMul, @rx, @const_GyroScale, @rx            'rx /= GyroScale
        word  FLT#opAdd, @rx, @errCorrX, @rx                   'rx += errCorrX

  'fgy = gy / GyroScale + errCorrY
        word  FLT#opFloat, @gz,  0, @ry                         'ry = float(gz)
        word  FLT#opMul, @ry, @const_NegGyroScale, @ry         'ry /= GyroScale
        word  FLT#opAdd, @ry, @errCorrY, @ry                   'ry += errCorrY

  'fgz = gz / GyroScale + errCorrZ
        word  FLT#opFloat, @gy, 0, @rz                         'rz = float(gy)
        word  FLT#opMul, @rz, @const_NegGyroScale, @rz         'rz /= GyroScale
        word  FLT#opAdd, @rz, @errCorrZ, @rz                   'rz += errCorrZ

  'rmag = sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) * 0.5
        word  FLT#opSqr, @rx, 0, @rmag                                  'rmag = fgx*fgx
        word  FLT#opSqr, @ry, 0, @temp                                  'temp = fgy*fgy
        word  FLT#opAdd, @rmag, @temp, @rmag                            'rmag += temp
        word  FLT#opSqr, @rz, 0, @temp                                  'temp = fgz*fgz
        word  FLT#opAdd, @rmag, @temp, @rmag                            'rmag += temp
        word  FLT#opAdd, @rmag, @const_epsilon, @rmag                   'rmag += 0.00000001
        word  FLT#opSqrt, @rmag, 0, @rmag                               'rmag = Sqrt(rmag)                                                  
        word  FLT#opShift, @rmag, @const_neg1, @rmag                    'rmag *= 0.5                                                  
  '8 instructions  (17)

  'cosr = Cos(rMag)
  'sinr = Sin(rMag) / rMag
        word  FLT#opSinCos, @rmag,  @sinr, @cosr                         'sinr = Sin(rmag), cosr = Cos(rmag)  
        word  FLT#opDiv, @sinr,  @rmag, @sinr                            'sinr /= rmag                                                  
  '3 instructions  (20)

  'qdot.w =  (r.x*x + r.y*y + r.z*z) * -0.5
        word  FLT#opMul, @rx,  @qx, @qdw                                 'qdw = rx*qx 
        word  FLT#opMul, @ry,  @qy, @temp                                'temp = ry*qy
        word  FLT#opAdd, @qdw,  @temp, @qdw                              'qdw += temp
        word  FLT#opMul, @rz,  @qz, @temp                                'temp = rz*qz
        word  FLT#opAdd, @qdw,  @temp, @qdw                              'qdw += temp
        word  FLT#opMul, @qdw,  @const_neghalf, @qdw                     'qdw *= -0.5
  '8 instructions  (28)

  'qdot.x =  (r.x*w + r.z*y - r.y*z) * 0.5
        word  FLT#opMul, @rx,  @qw, @qdx                                 'qdx = rx*qw 
        word  FLT#opMul, @rz,  @qy, @temp                                'temp = rz*qy
        word  FLT#opAdd, @qdx,  @temp, @qdx                              'qdx += temp
        word  FLT#opMul, @ry,  @qz, @temp                                'temp = ry*qz
        word  FLT#opSub, @qdx,  @temp, @qdx                              'qdx -= temp
        word  FLT#opShift, @qdx,  @const_neg1, @qdx                      'qdx *= 0.5
  '8 instructions  (36)

  'qdot.y =  (r.y*w - r.z*x + r.x*z) * 0.5
        word  FLT#opMul, @ry,  @qw, @qdy                                 'qdy = ry*qw 
        word  FLT#opMul, @rz,  @qx, @temp                                'temp = rz*qx
        word  FLT#opSub, @qdy,  @temp, @qdy                              'qdy -= temp
        word  FLT#opMul, @rx,  @qz, @temp                                'temp = rx*qz
        word  FLT#opAdd, @qdy,  @temp, @qdy                              'qdy += temp
        word  FLT#opShift, @qdy,  @const_neg1, @qdy                      'qdy *= 0.5
  '8 instructions  (44)

  'qdot.z =  (r.z*w + r.y*x - r.x*y) * 0.5
        word  FLT#opMul, @rz,  @qw, @qdz                                 'qdz = rz*qw 
        word  FLT#opMul, @ry,  @qx, @temp                                'temp = ry*qx
        word  FLT#opAdd, @qdz,  @temp, @qdz                              'qdz += temp
        word  FLT#opMul, @rx,  @qy, @temp                                'temp = rx*qy
        word  FLT#opSub, @qdz,  @temp, @qdz                              'qdz -= temp
        word  FLT#opShift, @qdz,  @const_neg1, @qdz                      'qdz *= 0.5
  '8 instructions  (52)
   
  'q.w = cosr * q.w + sinr * qdot.w
        word  FLT#opMul, @cosr,  @qw, @qw                                'qw = cosr*qw 
        word  FLT#opMul, @sinr,  @qdw, @temp                             'temp = sinr*qdw
        word  FLT#opAdd, @qw,  @temp, @qw                                'qw += temp

  'q.x = cosr * q.x + sinr * qdot.x
        word  FLT#opMul, @cosr,  @qx, @qx                                'qx = cosr*qx 
        word  FLT#opMul, @sinr,  @qdx, @temp                             'temp = sinr*qdx
        word  FLT#opAdd, @qx,  @temp, @qx                                'qx += temp

  'q.y = cosr * q.y + sinr * qdot.y
        word  FLT#opMul, @cosr,  @qy, @qy                                'qy = cosr*qy 
        word  FLT#opMul, @sinr,  @qdy, @temp                             'temp = sinr*qdy
        word  FLT#opAdd, @qy,  @temp, @qy                                'qy += temp

  'q.z = cosr * q.z + sinr * qdot.z
        word  FLT#opMul, @cosr,  @qz, @qz                                'qz = cosr*qz 
        word  FLT#opMul, @sinr,  @qdz, @temp                             'temp = sinr*qdz
        word  FLT#opAdd, @qz,  @temp, @qz                                'qz += temp
  '12 instructions  (64)

  'q = q.Normalize()
  'rmag = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w + 0.0000001)
        word  FLT#opSqr, @qx,  0, @rmag                                  'rmag = qx*qx 
        word  FLT#opSqr, @qy,  0, @temp                                  'temp = qy*qy 
        word  FLT#opAdd, @rmag,  @temp, @rmag                            'rmag += temp 
        word  FLT#opSqr, @qz,  0, @temp                                  'temp = qz*qz 
        word  FLT#opAdd, @rmag,  @temp, @rmag                            'rmag += temp 
        word  FLT#opSqr, @qw,  0, @temp                                  'temp = qw*qw 
        word  FLT#opAdd, @rmag,  @temp, @rmag                            'rmag += temp 
        word  FLT#opAdd, @rmag,  @const_epsilon, @rmag                   'rmag += 0.0000001 
        word  FLT#opSqrt, @rmag,  0, @rmag                               'sqrt(rmag) 
  '9 instructions (73)

  'q /= rmag   
        word  FLT#opDiv, @qw,  @rmag, @qw                                'qw /= rmag 
        word  FLT#opDiv, @qx,  @rmag, @qx                                'qx /= rmag 
        word  FLT#opDiv, @qy,  @rmag, @qy                                'qy /= rmag 
        word  FLT#opDiv, @qz,  @rmag, @qz                                'qz /= rmag 
  '4 instructions (77)


  'Now convert the updated quaternion to a rotation matrix 

  'fx2 = qx * qx;
  'fy2 = qy * qy;
  'fz2 = qz * qz;
        word  FLT#opSqr, @qx,  0, @fx2                                   'fx2 = qx *qx
        word  FLT#opSqr, @qy,  0, @fy2                                   'fy2 = qy *qy
        word  FLT#opSqr, @qz,  0, @fz2                                   'fz2 = qz *qz
  '3 instructions (80)

  'fwx = qw * qx;
  'fwy = qw * qy;
  'fwz = qw * qz;
        word  FLT#opMul, @qw,  @qx, @fwx                                 'fwx = qw *qx
        word  FLT#opMul, @qw,  @qy, @fwy                                 'fwy = qw *qy
        word  FLT#opMul, @qw,  @qz, @fwz                                 'fwz = qw *qz
  '3 instructions (83)

  'fxy = qx * qy;
  'fxz = qx * qz;
  'fyz = qy * qz;
        word  FLT#opMul, @qx,  @qy, @fxy                                 'fxy = qx *qy
        word  FLT#opMul, @qx,  @qz, @fxz                                 'fxz = qx *qz
        word  FLT#opMul, @qy,  @qz, @fyz                                 'fyz = qy *qz
  '3 instructions (86)

   
  'm00 = 1.0f - 2.0f * (y2 + z2)
        word  FLT#opAdd, @fy2,  @fz2, @temp                              'temp = fy2+fz2
        word  FLT#opShift, @temp,  @const_1, @temp                       'temp *= 2.0
        word  FLT#opSub, @const_F1,  @temp, @m00                         'm00 = 1.0 - temp
     
  'm01 =        2.0f * (fxy - fwz)
        word  FLT#opSub, @fxy,  @fwz, @temp                              'temp = fxy-fwz
        word  FLT#opShift, @temp,  @const_1, @m01                        'm01 = 2.0 * temp

  'm02 =        2.0f * (fxz + fwy)
        word  FLT#opAdd, @fxz,  @fwy, @temp                              'temp = fxz+fwy
        word  FLT#opShift, @temp,  @const_1, @m02                        'm02 = 2.0 * temp
  '7 instructions (93)

   
  'm10 =        2.0f * (fxy + fwz)
        word  FLT#opAdd, @fxy,  @fwz, @temp                              'temp = fxy-fwz
        word  FLT#opShift, @temp,  @const_1, @m10                        'm10 = 2.0 * temp

  'm11 = 1.0f - 2.0f * (x2 + z2)
        word  FLT#opAdd, @fx2,  @fz2, @temp                              'temp = fx2+fz2
        word  FLT#opShift, @temp,  @const_1, @temp                       'temp *= 2.0
        word  FLT#opSub, @const_F1,  @temp, @m11                         'm11 = 1.0 - temp

  'm12 =        2.0f * (fyz - fwx)
        word  FLT#opSub, @fyz,  @fwx, @temp                              'temp = fyz-fwx
        word  FLT#opShift, @temp,  @const_1, @m12                        'm12 = 2.0 * temp
  '7 instructions (100)

   
  'm20 =        2.0f * (fxz - fwy)
        word  FLT#opSub, @fxz,  @fwy, @temp                              'temp = fxz-fwz
        word  FLT#opShift, @temp,  @const_1, @m20                        'm20 = 2.0 * temp

  'm21 =        2.0f * (fyz + fwx)
        word  FLT#opAdd, @fyz,  @fwx, @temp                              'temp = fyz+fwx
        word  FLT#opShift, @temp,  @const_1, @m21                        'm21 = 2.0 * temp

  'm22 = 1.0f - 2.0f * (x2 + y2)
        word  FLT#opAdd, @fx2,  @fy2, @temp                              'temp = fx2+fy2
        word  FLT#opShift, @temp,  @const_1, @temp                       'temp *= 2.0
        word  FLT#opSub, @const_F1,  @temp, @m22                         'm22 = 1.0 - temp
  '7 instructions (107)



  'fax =  packet.ax;           // Acceleration in X (left/right)
  'fay =  packet.az;           // Acceleration in Y (up/down)
  'faz =  packet.ay;           // Acceleration in Z (toward/away)
        word  FLT#opFloat, @ax,  0, @fax  
        word  FLT#opFloat, @az,  0, @fay  
        word  FLT#opFloat, @ay,  0, @faz  
        word  FLT#opNeg, @fax,  0, @fax


'Rotation correction of the accelerometer vector - rotate around the pitch and roll axes by the specified amounts

  'axRot = (fax * accRollCorrCos) - (fay * accRollCorrSin)
        word  FLT#opMul, @fax,  @accRollCorrCos, @axRot                           
        word  FLT#opMul, @fay,  @accRollCorrSin, @temp
        word  FLT#opSub, @axRot,  @temp, @axRot 

  'ayRot = (fax * accRollCorrSin) + (fay * accRollCorrCos)
        word  FLT#opMul, @fax,  @accRollCorrSin, @ayRot                           
        word  FLT#opMul, @fay,  @accRollCorrCos, @temp
        word  FLT#opAdd, @ayRot,  @temp, @ayRot

  'fax = axRot         
  'fay = ayRot
        word  FLT#opMov, @axRot,  0, @fax          
        word  FLT#opMov, @ayRot,  0, @fay          



  'axRot = (faz * accPitchCorrCos) - (fay * accPitchCorrSin)
        word  FLT#opMul, @faz,  @accPitchCorrCos, @axRot
        word  FLT#opMul, @fay,  @accPitchCorrSin, @temp
        word  FLT#opSub, @axRot,  @temp, @axRot 

  'ayRot = (fax * accPitchCorrSin) + (fay * accPitchCorrCos)
        word  FLT#opMul, @faz,  @accPitchCorrSin, @ayRot                           
        word  FLT#opMul, @fay,  @accPitchCorrCos, @temp
        word  FLT#opAdd, @ayRot,  @temp, @ayRot

  'faz = axRot         
  'fay = ayRot
        word  FLT#opMov, @axRot,  0, @faz          
        word  FLT#opMov, @ayRot,  0, @fay          



'Compute length of the accelerometer vector to decide weighting                                   

  'rmag = facc.length
        word  FLT#opSqr, @fax,  0, @rmag                                  'rmag = fax*fax
        word  FLT#opSqr, @fay,  0, @temp                                  'temp = fay*fay
        word  FLT#opAdd, @rmag,  @temp, @rmag                             'rmag += temp
        word  FLT#opSqr, @faz,  0, @temp                                  'temp = faz*faz
        word  FLT#opAdd, @rmag,  @temp, @rmag                             'rmag += temp
        word  FLT#opAdd, @rmag,  @const_epsilon, @rmag                    'rmag += 0.00000001
        word  FLT#opSqrt, @rmag,  0, @rmag                                'rmag = Sqrt(rmag)                                                  

  'facc /= rmag
        word  FLT#opDiv, @fax,  @rmag, @faxn                              'faxn = fax / rmag 
        word  FLT#opDiv, @fay,  @rmag, @fayn                              'fayn = fay / rmag 
        word  FLT#opDiv, @faz,  @rmag, @fazn                              'fazn = faz / rmag 



  'accWeight = 1.0 - FMin( FAbs( 2.0 - accLen * 2.0 ), 1.0 )
        word  FLT#opMul, @rmag,  @const_AccScale, @rmag                   'rmag /= accScale (accelerometer to 1G units)
        word  FLT#opShift, @rmag,  @const_1, @accWeight                   'accWeight = rmag * 2.0
        word  FLT#opSub, @const_F2,  @accWeight, @accWeight               'accWeight = 2.0 - accWeight
        word  FLT#opFAbs, @accWeight,  0, @accWeight                      'accWeight = FAbs(accWeight)
        word  FLT#opFMin, @accWeight,  @const_F1, @accWeight              'accWeight = FMin( accWeight, 1.0 )
        word  FLT#opSub, @const_F1,  @accWeight, @accWeight               'accWeight = 1.0 - accWeight                                                

   

  'errDiffX = fayn * m12 - fazn * m11
        word  FLT#opMul, @fayn,  @m12, @errDiffX 
        word  FLT#opMul, @fazn,  @m11, @temp 
        word  FLT#opSub, @errDiffX,  @temp, @errDiffX 

  'errDiffY = fazn * m10 - faxn * m12
        word  FLT#opMul, @fazn,  @m10, @errDiffY 
        word  FLT#opMul, @faxn,  @m12, @temp 
        word  FLT#opSub, @errDiffY,  @temp, @errDiffY 

  'errDiffZ = faxn * m11 - fayn * m10
        word  FLT#opMul, @faxn,  @m11, @errDiffZ 
        word  FLT#opMul, @fayn,  @m10, @temp 
        word  FLT#opSub, @errDiffZ,  @temp, @errDiffZ 

  'accWeight *= const_ErrScale   
        word  FLT#opMul, @const_ErrScale,  @accWeight, @accWeight

  'Test: Does ErrCorr need to be rotated into the local frame from the world frame?


  'errCorr = errDiff * accWeight
        word  FLT#opMul, @errDiffX,  @accWeight, @errCorrX  
        word  FLT#opMul, @errDiffY,  @accWeight, @errCorrY  
        word  FLT#opMul, @errDiffZ,  @accWeight, @errCorrZ  


    'tx := Flt.ASin( Flt.FFloatDiv28( DCM.GetM12 ) )     'Convert to float, then divide by (float)(1<<28)
    'tz := Flt.ASin( Flt.FFloatDiv28( DCM.GetM10 ) )     'Convert to float, then divide by (float)(1<<28) 

    'XAngle := Flt.FRound( Flt.FMul( tx,  constant( 320000.0 / (PI / 2.0)) ) ) 
    'ZAngle := Flt.FRound( Flt.FMul( tz,  constant(-320000.0 / (PI / 2.0)) ) )

    'if( DCM.GetMatrixvalue(4) < 0 )                     'If the Y value of the Y axis is negative, we're upside down
    '  if( ||ZAngle > ||XAngle ) 
    '    ZAngle := ZAngle 

    'For heading, I want an actual angular value, so this returns me an int between 0 & 65535, where 0 is forward
    'YAngle := Flt.FRound( Flt.FMul( Flt.Atan2( Flt.FFloat(DCM.GetM20), Flt.FFloat(DCM.GetM22)), constant(32768.0 / PI) ) ) & 65535 


        word  FLT#opASinCos, @m12,  0, @temp  
        word  FLT#opMul, @temp,  @const_outAngleScale, @temp
        word  FLT#opTruncRound, @temp,  @const_0, @Pitch  
    
        word  FLT#opASinCos, @m10,  0, @temp  
        word  FLT#opMul, @temp,  @const_outNegAngleScale, @temp  
        word  FLT#opTruncRound, @temp,  @const_0, @Roll  
              
        word  FLT#opATan2, @m20,  @m22, @temp  
        word  FLT#opMul, @temp,  @const_outNegAngleScale, @temp    
        word  FLT#opTruncRound, @temp,  @const_0, @Yaw  


        word  FLT#opDiv, @const_F1,  @m11, @temp                          '1.0/m11 = scale factor for thrust - this will be infinite if perpendicular to ground   
        word  FLT#opMul, @temp,  @const_ThrustScale, @temp                '*= 256.0  
        word  FLT#opTruncRound, @temp,  @const_0, @ThrustFactor  



  'Compute the running height estimate

  'force := acc / 4096.0
        word  FLT#opShift, @fax,  @const_neg12, @forceX    
        word  FLT#opShift, @fay,  @const_neg12, @forceY    
        word  FLT#opShift, @faz,  @const_neg12, @forceZ    

  'force -= m[1,0], m[1,1], m[1,2]  - Subtract gravity (1G, straight down)
        word  FLT#opSub, @forceX,  @m10, @forceX    
        word  FLT#opSub, @forceY,  @m11, @forceY    
        word  FLT#opSub, @forceZ,  @m12, @forceZ    

  'forceWY := M.Transpose().Mul(Force).y                 'Orient force vector into world frame
  'forceWY = m01*forceX + m11*forceY + m21*forceZ

        word  FLT#opMul, @forceX,  @m01, @forceWY  
   
        word  FLT#opMul, @forceY,  @m11, @temp  
        word  FLT#opAdd, @forceWY,  @temp, @forceWY  

        word  FLT#opMul, @forceZ,  @m21, @temp  
        word  FLT#opAdd, @forceWY,  @temp, @forceWY  

  'forceWY *= 9.8                                       'Convert to M/sec^2
        word  FLT#opMul, @forceWY,  @const_GMetersPerSec, @forceWY  



        word  FLT#opMul, @forceWY,  @const_UpdateScale, @temp             'temp := forceWY / UpdateRate
        word  FLT#opAdd, @velocityEstimate,  @temp, @velocityEstimate     'velEstimate += forceWY / UpdateRate

  
        word  FLT#opFloat, @altRate,  0, @altitudeVelocity                'AltVelocity = float(altRate)
        word  FLT#opMul, @altitudeVelocity,  @const_AltiVelScale, @altitudeVelocity   'Convert from mm/sec to m/sec   


  'VelocityEstimate := (VelocityEstimate * 0.9950) + (altVelocity * 0.0050)
        word  FLT#opMul, @velocityEstimate,  @const_velAccScale, @velocityEstimate 
        word  FLT#opMul, @altitudeVelocity,  @const_velAltiScale, @temp  
        word  Flt#opAdd, @velocityEstimate,  @temp, @velocityEstimate   

  'altitudeEstimate += velocityEstimate / UpdateRate
        word  FLT#opMul, @velocityEstimate,  @const_UpdateScale, @temp  
        word  FLT#opAdd, @altitudeEstimate,  @temp, @altitudeEstimate   

  'altitudeEstimate := (altitudeEstimate * 0.9950) * (alti / 1000.0) * 0.0050
        word  FLT#opMul, @altitudeEstimate,  @const_velAccTrust, @altitudeEstimate 

        word  FLT#opFloat, @alt,  0, @temp                               'temp := float(alt)
        word  FLT#opDiv, @temp,  @const_m_to_mm, @temp                   'temp /= 1000.0    (alt now in m)
        word  FLT#opMul, @temp,  @const_velAltiTrust, @temp              'temp *= 0.0050
        word  FLT#opAdd, @altitudeEstimate,  @temp, @altitudeEstimate    'altEstimate += temp 


        word  FLT#opMul, @altitudeEstimate,  @const_m_to_mm, @temp       'temp = altEst * 1000.0    (temp now in mm)
        word  FLT#opTruncRound, @temp,  @const_0, @AltitudeEstMM 

        word  FLT#opMul, @velocityEstimate,  @const_m_to_mm, @temp       'temp = velEst * 1000.0    (temp now in mm/sec)
        word  FLT#opTruncRound, @temp,  @const_0, @VelocityEstMM 


  'Create a fixed point version of the orientation matrix
        word  FLT#opShift, @m00,  @const_16, @temp   
        word  FLT#opTruncRound, @temp,  @const_0, @fm00 
        word  FLT#opShift, @m01,  @const_16, @temp   
        word  FLT#opTruncRound, @temp,  @const_0, @fm01 
        word  FLT#opShift, @m02,  @const_16, @temp     
        word  FLT#opTruncRound, @temp,  @const_0, @fm02 

        word  FLT#opShift, @m10,  @const_16, @temp   
        word  FLT#opTruncRound, @temp,  @const_0, @fm10 
        word  FLT#opShift, @m11,  @const_16, @temp   
        word  FLT#opTruncRound, @temp,  @const_0, @fm11 
        word  FLT#opShift, @m12,  @const_16, @temp   
        word  FLT#opTruncRound, @temp,  @const_0, @fm12 

        word  FLT#opShift, @m20,  @const_16, @temp   
        word  FLT#opTruncRound, @temp,  @const_0, @fm20 
        word  FLT#opShift, @m21,  @const_16, @temp   
        word  FLT#opTruncRound, @temp,  @const_0, @fm21 
        word  FLT#opShift, @m22,  @const_16, @temp   
        word  FLT#opTruncRound, @temp,  @const_0, @fm22
        word  0, 0, 0, 0 
'}