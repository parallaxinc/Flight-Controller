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

  {
  FLT.StartStream( 0, @QuatUpdateCommands )

  'rx = rotation around X axis (pitch, + == forward)
  'ry = rotation around Y axis (yaw, + == clockwise from top) 
  'rz = rotation around Z axis (roll, + == rolling left) 

  'fgx = gx / GyroScale + errCorrX
  FLT.AddCommand( 0, FLT#opFloat, @gx, 0, @rx )                                 'rx = float(gx)
  FLT.AddCommand( 0, FLT#opMul, @rx, @const_GyroScale, @rx )                    'rx /= GyroScale
  FLT.AddCommand( 0, FLT#opAdd, @rx, @errCorrX, @rx )                           'rx += errCorrX

  'fgy = gy / GyroScale + errCorrY
  FLT.AddCommand( 0, FLT#opFloat, @gz, 0, @ry )                                 'ry = float(gz)
  FLT.AddCommand( 0, FLT#opMul, @ry, @const_NegGyroScale, @ry )                 'ry /= GyroScale
  FLT.AddCommand( 0, FLT#opAdd, @ry, @errCorrY, @ry )                           'ry += errCorrY

  'fgz = gz / GyroScale + errCorrZ
  FLT.AddCommand( 0, FLT#opFloat, @gy, 0, @rz )                                 'rz = float(gy)
  FLT.AddCommand( 0, FLT#opMul, @rz, @const_NegGyroScale, @rz )                 'rz /= GyroScale
  FLT.AddCommand( 0, FLT#opAdd, @rz, @errCorrZ, @rz )                           'rz += errCorrZ
  '9 instructions

  'rmag = sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) * 0.5 
  FLT.AddCommand( 0, FLT#opSqr, @rx, 0, @rmag )                                 'rmag = fgx*fgx
  FLT.AddCommand( 0, FLT#opSqr, @ry, 0, @temp )                                 'temp = fgy*fgy
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @temp, @rmag )                           'rmag += temp
  FLT.AddCommand( 0, FLT#opSqr, @rz, 0, @temp )                                 'temp = fgz*fgz
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @temp, @rmag )                           'rmag += temp
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @const_epsilon, @rmag )                  'rmag += 0.00000001
  FLT.AddCommand( 0, FLT#opSqrt, @rmag, 0, @rmag )                              'rmag = Sqrt(rmag)                                                  
  FLT.AddCommand( 0, FLT#opShift, @rmag, @const_neg1, @rmag )                   'rmag *= 0.5                                                  
  '8 instructions  (17)

  'cosr = Cos(rMag)
  'sinr = Sin(rMag) / rMag
  FLT.AddCommand( 0, FLT#opSinCos, @rmag, @sinr, @cosr )                        'sinr = Sin(rmag), cosr = Cos(rmag)  
  FLT.AddCommand( 0, FLT#opDiv, @sinr, @rmag, @sinr )                           'sinr /= rmag                                                  
  '3 instructions  (20)

  'qdot.w =  (r.x*x + r.y*y + r.z*z) * -0.5
  FLT.AddCommand( 0, FLT#opMul, @rx, @qx, @qdw )                                'qdw = rx*qx 
  FLT.AddCommand( 0, FLT#opMul, @ry, @qy, @temp )                               'temp = ry*qy
  FLT.AddCommand( 0, FLT#opAdd, @qdw, @temp, @qdw )                             'qdw += temp
  FLT.AddCommand( 0, FLT#opMul, @rz, @qz, @temp )                               'temp = rz*qz
  FLT.AddCommand( 0, FLT#opAdd, @qdw, @temp, @qdw )                             'qdw += temp
  FLT.AddCommand( 0, FLT#opMul, @qdw, @const_neghalf, @qdw )                    'qdw *= -0.5
  '8 instructions  (28)

  'qdot.x =  (r.x*w + r.z*y - r.y*z) * 0.5
  FLT.AddCommand( 0, FLT#opMul, @rx, @qw, @qdx )                                'qdx = rx*qw 
  FLT.AddCommand( 0, FLT#opMul, @rz, @qy, @temp )                               'temp = rz*qy
  FLT.AddCommand( 0, FLT#opAdd, @qdx, @temp, @qdx )                             'qdx += temp
  FLT.AddCommand( 0, FLT#opMul, @ry, @qz, @temp )                               'temp = ry*qz
  FLT.AddCommand( 0, FLT#opSub, @qdx, @temp, @qdx )                             'qdx -= temp
  FLT.AddCommand( 0, FLT#opShift, @qdx, @const_neg1, @qdx )                     'qdx *= 0.5
  '8 instructions  (36)

  'qdot.y =  (r.y*w - r.z*x + r.x*z) * 0.5
  FLT.AddCommand( 0, FLT#opMul, @ry, @qw, @qdy )                                'qdy = ry*qw 
  FLT.AddCommand( 0, FLT#opMul, @rz, @qx, @temp )                               'temp = rz*qx
  FLT.AddCommand( 0, FLT#opSub, @qdy, @temp, @qdy )                             'qdy -= temp
  FLT.AddCommand( 0, FLT#opMul, @rx, @qz, @temp )                               'temp = rx*qz
  FLT.AddCommand( 0, FLT#opAdd, @qdy, @temp, @qdy )                             'qdy += temp
  FLT.AddCommand( 0, FLT#opShift, @qdy, @const_neg1, @qdy )                     'qdy *= 0.5
  '8 instructions  (44)

  'qdot.z =  (r.z*w + r.y*x - r.x*y) * 0.5
  FLT.AddCommand( 0, FLT#opMul, @rz, @qw, @qdz )                                'qdz = rz*qw 
  FLT.AddCommand( 0, FLT#opMul, @ry, @qx, @temp )                               'temp = ry*qx
  FLT.AddCommand( 0, FLT#opAdd, @qdz, @temp, @qdz )                             'qdz += temp
  FLT.AddCommand( 0, FLT#opMul, @rx, @qy, @temp )                               'temp = rx*qy
  FLT.AddCommand( 0, FLT#opSub, @qdz, @temp, @qdz )                             'qdz -= temp
  FLT.AddCommand( 0, FLT#opShift, @qdz, @const_neg1, @qdz )                     'qdz *= 0.5
  '8 instructions  (52)
   
  'q.w = cosr * q.w + sinr * qdot.w
  FLT.AddCommand( 0, FLT#opMul, @cosr, @qw, @qw )                               'qw = cosr*qw 
  FLT.AddCommand( 0, FLT#opMul, @sinr, @qdw, @temp )                            'temp = sinr*qdw
  FLT.AddCommand( 0, FLT#opAdd, @qw, @temp, @qw )                               'qw += temp

  'q.x = cosr * q.x + sinr * qdot.x
  FLT.AddCommand( 0, FLT#opMul, @cosr, @qx, @qx )                               'qx = cosr*qx 
  FLT.AddCommand( 0, FLT#opMul, @sinr, @qdx, @temp )                            'temp = sinr*qdx
  FLT.AddCommand( 0, FLT#opAdd, @qx, @temp, @qx )                               'qx += temp

  'q.y = cosr * q.y + sinr * qdot.y
  FLT.AddCommand( 0, FLT#opMul, @cosr, @qy, @qy )                               'qy = cosr*qy 
  FLT.AddCommand( 0, FLT#opMul, @sinr, @qdy, @temp )                            'temp = sinr*qdy
  FLT.AddCommand( 0, FLT#opAdd, @qy, @temp, @qy )                               'qy += temp

  'q.z = cosr * q.z + sinr * qdot.z
  FLT.AddCommand( 0, FLT#opMul, @cosr, @qz, @qz )                               'qz = cosr*qz 
  FLT.AddCommand( 0, FLT#opMul, @sinr, @qdz, @temp )                            'temp = sinr*qdz
  FLT.AddCommand( 0, FLT#opAdd, @qz, @temp, @qz )                               'qz += temp
  '12 instructions  (64)

  
  'q = q.Normalize()
  'rmag = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w + 0.0000001)
  FLT.AddCommand( 0, FLT#opSqr, @qx, 0, @rmag )                                 'rmag = qx*qx 
  FLT.AddCommand( 0, FLT#opSqr, @qy, 0, @temp )                                 'temp = qy*qy 
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @temp, @rmag )                           'rmag += temp 
  FLT.AddCommand( 0, FLT#opSqr, @qz, 0, @temp )                                 'temp = qz*qz 
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @temp, @rmag )                           'rmag += temp 
  FLT.AddCommand( 0, FLT#opSqr, @qw, 0, @temp )                                 'temp = qw*qw 
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @temp, @rmag )                           'rmag += temp 
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @const_epsilon, @rmag )                  'rmag += 0.0000001 
  FLT.AddCommand( 0, FLT#opSqrt, @rmag, 0, @rmag )                              'sqrt(rmag) 
  '9 instructions (73)

  'q /= rmag   
  FLT.AddCommand( 0, FLT#opDiv, @qw, @rmag, @qw )                               'qw /= rmag 
  FLT.AddCommand( 0, FLT#opDiv, @qx, @rmag, @qx )                               'qx /= rmag 
  FLT.AddCommand( 0, FLT#opDiv, @qy, @rmag, @qy )                               'qy /= rmag 
  FLT.AddCommand( 0, FLT#opDiv, @qz, @rmag, @qz )                               'qz /= rmag 
  '4 instructions (77)


  'Now convert the updated quaternion to a rotation matrix 

  'fx2 = qx * qx;
  'fy2 = qy * qy;
  'fz2 = qz * qz;
  FLT.AddCommand( 0, FLT#opSqr, @qx, 0, @fx2 )                                  'fx2 = qx *qx
  FLT.AddCommand( 0, FLT#opSqr, @qy, 0, @fy2 )                                  'fy2 = qy *qy
  FLT.AddCommand( 0, FLT#opSqr, @qz, 0, @fz2 )                                  'fz2 = qz *qz
  '3 instructions (80)

  'fwx = qw * qx;
  'fwy = qw * qy;
  'fwz = qw * qz;
  FLT.AddCommand( 0, FLT#opMul, @qw, @qx, @fwx )                                'fwx = qw *qx
  FLT.AddCommand( 0, FLT#opMul, @qw, @qy, @fwy )                                'fwy = qw *qy
  FLT.AddCommand( 0, FLT#opMul, @qw, @qz, @fwz )                                'fwz = qw *qz
  '3 instructions (83)

  'fxy = qx * qy;
  'fxz = qx * qz;
  'fyz = qy * qz;
  FLT.AddCommand( 0, FLT#opMul, @qx, @qy, @fxy )                                'fxy = qx *qy
  FLT.AddCommand( 0, FLT#opMul, @qx, @qz, @fxz )                                'fxz = qx *qz
  FLT.AddCommand( 0, FLT#opMul, @qy, @qz, @fyz )                                'fyz = qy *qz
  '3 instructions (86)

   
  'm00 = 1.0f - 2.0f * (y2 + z2)
  FLT.AddCommand( 0, FLT#opAdd, @fy2, @fz2, @temp )                             'temp = fy2+fz2
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @temp )                      'temp *= 2.0
  FLT.AddCommand( 0, FLT#opSub, @const_F1, @temp, @m00 )                        'm00 = 1.0 - temp
     
  'm01 =        2.0f * (fxy - fwz)
  FLT.AddCommand( 0, FLT#opSub, @fxy, @fwz, @temp )                             'temp = fxy-fwz
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @m01 )                       'm01 = 2.0 * temp

  'm02 =        2.0f * (fxz + fwy)
  FLT.AddCommand( 0, FLT#opAdd, @fxz, @fwy, @temp )                             'temp = fxz+fwy
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @m02 )                       'm02 = 2.0 * temp
  '7 instructions (93)

   
  'm10 =        2.0f * (fxy + fwz)
  FLT.AddCommand( 0, FLT#opAdd, @fxy, @fwz, @temp )                             'temp = fxy-fwz
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @m10 )                       'm10 = 2.0 * temp

  'm11 = 1.0f - 2.0f * (x2 + z2)
  FLT.AddCommand( 0, FLT#opAdd, @fx2, @fz2, @temp )                             'temp = fx2+fz2
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @temp )                      'temp *= 2.0
  FLT.AddCommand( 0, FLT#opSub, @const_F1, @temp, @m11 )                        'm11 = 1.0 - temp

  'm12 =        2.0f * (fyz - fwx)
  FLT.AddCommand( 0, FLT#opSub, @fyz, @fwx, @temp )                             'temp = fyz-fwx
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @m12 )                       'm12 = 2.0 * temp
  '7 instructions (100)

   
  'm20 =        2.0f * (fxz - fwy)
  FLT.AddCommand( 0, FLT#opSub, @fxz, @fwy, @temp )                             'temp = fxz-fwz
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @m20 )                       'm20 = 2.0 * temp

  'm21 =        2.0f * (fyz + fwx)
  FLT.AddCommand( 0, FLT#opAdd, @fyz, @fwx, @temp )                             'temp = fyz+fwx
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @m21 )                       'm21 = 2.0 * temp

  'm22 = 1.0f - 2.0f * (x2 + y2)
  FLT.AddCommand( 0, FLT#opAdd, @fx2, @fy2, @temp )                             'temp = fx2+fy2
  FLT.AddCommand( 0, FLT#opShift, @temp, @const_1, @temp )                      'temp *= 2.0
  FLT.AddCommand( 0, FLT#opSub, @const_F1, @temp, @m22 )                        'm22 = 1.0 - temp
  '7 instructions (107)



  'fax =  packet.ax;           // Acceleration in X (left/right)
  'fay =  packet.az;           // Acceleration in Y (up/down)
  'faz =  packet.ay;           // Acceleration in Z (toward/away)
  FLT.AddCommand( 0, FLT#opFloat, @ax, 0, @fax )
  FLT.AddCommand( 0, FLT#opFloat, @az, 0, @fay )
  FLT.AddCommand( 0, FLT#opFloat, @ay, 0, @faz )
  FLT.AddCommand( 0, FLT#opNeg, @fax, 0, @fax )

  'rmag = facc.length
  FLT.AddCommand( 0, FLT#opSqr, @fax, 0, @rmag )                                'rmag = fax*fax
  FLT.AddCommand( 0, FLT#opSqr, @fay, 0, @temp )                                'temp = fay*fay
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @temp, @rmag )                           'rmag += temp
  FLT.AddCommand( 0, FLT#opSqr, @faz, 0, @temp )                                'temp = faz*faz
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @temp, @rmag )                           'rmag += temp
  FLT.AddCommand( 0, FLT#opAdd, @rmag, @const_epsilon, @rmag )                  'rmag += 0.00000001
  FLT.AddCommand( 0, FLT#opSqrt, @rmag, 0, @rmag )                              'rmag = Sqrt(rmag)                                                  

  'facc /= rmag
  FLT.AddCommand( 0, FLT#opDiv, @fax, @rmag, @faxn )                            'faxn = fax / rmag 
  FLT.AddCommand( 0, FLT#opDiv, @fay, @rmag, @fayn )                            'fayn = fay / rmag 
  FLT.AddCommand( 0, FLT#opDiv, @faz, @rmag, @fazn )                            'fazn = faz / rmag 



  'accWeight = 1.0 - FMin( FAbs( 2.0 - accLen * 2.0 ), 1.0 )
  FLT.AddCommand( 0, FLT#opMul, @rmag, @const_AccScale, @rmag )                 'rmag /= accScale (accelerometer to 1G units)
  FLT.AddCommand( 0, FLT#opShift, @rmag, @const_1, @accWeight )                 'accWeight = rmag * 2.0
  FLT.AddCommand( 0, FLT#opSub, @const_F2, @accWeight, @accWeight )             'accWeight = 2.0 - accWeight
  FLT.AddCommand( 0, FLT#opFAbs, @accWeight, 0, @accWeight )                    'accWeight = FAbs(accWeight)
  FLT.AddCommand( 0, FLT#opFMin, @accWeight, @const_F1, @accWeight )            'accWeight = FMin( accWeight, 1.0 )
  FLT.AddCommand( 0, FLT#opSub, @const_F1, @accWeight, @accWeight )             'accWeight = 1.0 - accWeight                                                

   

  'errDiffX = fayn * m12 - fazn * m11
  FLT.AddCommand( 0, FLT#opMul, @fayn, @m12, @errDiffX )
  FLT.AddCommand( 0, FLT#opMul, @fazn, @m11, @temp )
  FLT.AddCommand( 0, FLT#opSub, @errDiffX, @temp, @errDiffX )

  'errDiffY = fazn * m10 - faxn * m12
  FLT.AddCommand( 0, FLT#opMul, @fazn, @m10, @errDiffY )
  FLT.AddCommand( 0, FLT#opMul, @faxn, @m12, @temp )
  FLT.AddCommand( 0, FLT#opSub, @errDiffY, @temp, @errDiffY )

  'errDiffZ = faxn * m11 - fayn * m10
  FLT.AddCommand( 0, FLT#opMul, @faxn, @m11, @errDiffZ )
  FLT.AddCommand( 0, FLT#opMul, @fayn, @m10, @temp )
  FLT.AddCommand( 0, FLT#opSub, @errDiffZ, @temp, @errDiffZ )

  'accWeight *= const_ErrScale   
  FLT.AddCommand( 0, FLT#opMul, @const_ErrScale, @accWeight, @accWeight )


  'Continue from here ------
  
  'Test: Does ErrCorr need to be rotated into the local frame from the world frame?

  'errCorr = errDiff * accWeight
  FLT.AddCommand( 0, FLT#opMul, @errDiffX, @accWeight, @errCorrX )
  FLT.AddCommand( 0, FLT#opMul, @errDiffY, @accWeight, @errCorrY )
  FLT.AddCommand( 0, FLT#opMul, @errDiffZ, @accWeight, @errCorrZ )
  

    'tx := Flt.ASin( Flt.FFloatDiv28( DCM.GetM12 ) )     'Convert to float, then divide by (float)(1<<28)
    'tz := Flt.ASin( Flt.FFloatDiv28( DCM.GetM10 ) )     'Convert to float, then divide by (float)(1<<28) 

    'XAngle := Flt.FRound( Flt.FMul( tx , constant( 320000.0 / (PI / 2.0)) ) ) 
    'ZAngle := Flt.FRound( Flt.FMul( tz , constant(-320000.0 / (PI / 2.0)) ) )

    'if( DCM.GetMatrixvalue(4) < 0 )                     'If the Y value of the Y axis is negative, we're upside down
    '  if( ||ZAngle > ||XAngle ) 
    '    ZAngle := ZAngle 

    'For heading, I want an actual angular value, so this returns me an int between 0 & 65535, where 0 is forward
    'YAngle := Flt.FRound( Flt.FMul( Flt.Atan2( Flt.FFloat(DCM.GetM20), Flt.FFloat(DCM.GetM22)), constant(32768.0 / PI) ) ) & 65535 


  FLT.AddCommand( 0, FLT#opASinCos, @m12, 0, @temp )
  FLT.AddCommand( 0, FLT#opMul, @temp, @const_outAngleScale, @temp )
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @Pitch )
    
  FLT.AddCommand( 0, FLT#opASinCos, @m10, 0, @temp )
  FLT.AddCommand( 0, FLT#opMul, @temp, @const_outAngleScale, @temp )
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @Roll )
    
  FLT.AddCommand( 0, FLT#opATan2, @m20, @m22, @temp )
  FLT.AddCommand( 0, FLT#opMul, @temp, @const_outAngleScale, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @Yaw )


  FLT.AddCommand( 0, FLT#opDiv, @const_F1, @m11, @temp )                        '1.0/m11 = scale factor for thrust - this will be infinite if perpendicular to ground   
  FLT.AddCommand( 0, FLT#opMul, @temp, @const_ThrustScale, @temp )              '*= 256.0  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @ThrustFactor )



  'Compute the running height estimate

  'force := acc / 4096.0
  FLT.AddCommand( 0, FLT#opShift, @fax, @const_neg12, @forceX )  
  FLT.AddCommand( 0, FLT#opShift, @fay, @const_neg12, @forceY )  
  FLT.AddCommand( 0, FLT#opShift, @faz, @const_neg12, @forceZ )  

  'force -= m[1,0], m[1,1], m[1,2]  - Subtract gravity (1G, straight down)
  FLT.AddCommand( 0, FLT#opSub, @forceX, @m10, @forceX )  
  FLT.AddCommand( 0, FLT#opSub, @forceY, @m11, @forceY )  
  FLT.AddCommand( 0, FLT#opSub, @forceZ, @m12, @forceZ )  

  'forceWY := M.Transpose().Mul(Force).y                 'Orient force vector into world frame
  'forceWY = m01*forceX + m11*forceY + m21*forceZ

  FLT.AddCommand( 0, FLT#opMul, @forceX, @m01, @forceWY )
   
  FLT.AddCommand( 0, FLT#opMul, @forceY, @m11, @temp )
  FLT.AddCommand( 0, FLT#opAdd, @forceWY, @temp, @forceWY )

  FLT.AddCommand( 0, FLT#opMul, @forceZ, @m21, @temp )
  FLT.AddCommand( 0, FLT#opAdd, @forceWY, @temp, @forceWY )

  'forceWY *= 9.8                                       'Convert to M/sec^2
  Flt.AddCommand( 0, FLT#opMul, @forceWY, @const_GMetersPerSec, @forceWY )



  Flt.AddCommand( 0, FLT#opMul, @forceWY, @const_UpdateScale, @temp )           'temp := forceWY / UpdateRate
  Flt.AddCommand( 0, FLT#opAdd, @velocityEstimate, @temp, @velocityEstimate )   'velEstimate += forceWY / UpdateRate

  
  Flt.AddCommand( 0, FLT#opFloat, @altRate, 0, @altitudeVelocity )              'AltVelocity = float(altRate)
  Flt.AddCommand( 0, FLT#opMul, @altitudeVelocity, @const_AltiVelScale, @altitudeVelocity ) 'Convert from mm/sec to m/sec   


  'VelocityEstimate := (VelocityEstimate * 0.9950) + (altVelocity * 0.0050)
  Flt.AddCommand( 0, FLT#opMul, @velocityEstimate, @const_velAccScale, @velocityEstimate )
  Flt.AddCommand( 0, FLT#opMul, @altitudeVelocity, @const_velAltiScale, @temp )
  Flt.AddCommand( 0, Flt#opAdd, @velocityEstimate, @temp, @velocityEstimate ) 

  'altitudeEstimate += velocityEstimate / UpdateRate
  Flt.AddCommand( 0, FLT#opMul, @velocityEstimate, @const_UpdateScale, @temp )
  Flt.AddCommand( 0, FLT#opAdd, @altitudeEstimate, @temp, @altitudeEstimate ) 

  'altitudeEstimate := (altitudeEstimate * 0.9950) * (alti / 1000.0) * 0.0050
  Flt.AddCommand( 0, FLT#opMul, @altitudeEstimate, @const_velAccTrust, @altitudeEstimate )

  Flt.AddCommand( 0, FLT#opFloat, @alt, 0, @temp )                              'temp := float(alt)
  Flt.AddCommand( 0, FLT#opDiv, @temp, @const_m_to_mm, @temp )                  'temp /= 1000.0    (alt now in m)
  Flt.AddCommand( 0, FLT#opMul, @temp, @const_velAltiTrust, @temp )             'temp *= 0.0050
  Flt.AddCommand( 0, FLT#opAdd, @altitudeEstimate, @temp, @altitudeEstimate )   'altEstimate += temp 


  Flt.AddCommand( 0, FLT#opMul, @altitudeEstimate, @const_m_to_mm, @temp )      'temp = altEst * 1000.0    (temp now in mm)
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @AltitudeEstMM )

  Flt.AddCommand( 0, FLT#opMul, @velocityEstimate, @const_m_to_mm, @temp )      'temp = velEst * 1000.0    (temp now in mm/sec)
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @VelocityEstMM )


  'Create a fixed point version of the orientation matrix
  FLT.AddCommand( 0, FLT#opShift, @m00, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm00 )
  FLT.AddCommand( 0, FLT#opShift, @m01, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm01 )
  FLT.AddCommand( 0, FLT#opShift, @m02, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm02 )

  FLT.AddCommand( 0, FLT#opShift, @m10, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm10 )
  FLT.AddCommand( 0, FLT#opShift, @m11, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm11 )
  FLT.AddCommand( 0, FLT#opShift, @m12, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm12 )

  FLT.AddCommand( 0, FLT#opShift, @m20, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm20 )
  FLT.AddCommand( 0, FLT#opShift, @m21, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm21 )
  FLT.AddCommand( 0, FLT#opShift, @m22, @const_16, @temp )  
  FLT.AddCommand( 0, FLT#opTruncRound, @temp, @const_0, @fm22 )


  QuatUpdateLen := (FLT.EndStream( 0 ) - @QuatUpdateCommands) / 4 
  '}
    


PUB SetGyroZero( _x, _y, _z )
  zx := _x
  zy := _y
  zz := _z
  

PUB Send( v )
  'Debug.tx(v >> 8)
  'Debug.tx(v & 255)



'PUB GetQuatUpdateLen
'  return QuatUpdateLen

'PUB GetCalcErrorLen
'  return CalcErrorLen


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


{
PUB Update_Part2 | t


  t := cnt
  FLT.WaitStream                'Wait for the previous stream to complete
  
  FLT.RunStream( @CalcErrorUpdateAngles )
  t := cnt-t                    'Resulting t value is the number of cycles taken to execute the IMU code 

  return t     
}

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
   
  repeat until long[p+4] == 0                           'A pointer to 0 indicates the end of the table
    long[p] := FLT.GetCommandInstruction( long[p] )     'Replace the instruction index with the actual JMPRET instruction
    p += 4                                              'Advance to the first set of arguments
    long[p] += diff | (diff << 16)                      'Replace the two relative pointers (words) with absolute pointers
    p += 4                                              'Advance to the last argument (a single long)
    long[p] += diff                                     'Replace the relative pointer (long) with an absolute pointer
    p += 4                                              'advance to the next instruction in the list      
   
   

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
const_outAngleScale     long    -65536.0 / PI               
const_ThrustScale       long    256.0
const_GMetersPerSec     long    9.80665
const_AltiVelScale      long    1.0/1000.0      'Convert mm to m
const_UpdateScale       long    1.0 / float(Const#UpdateRate) 'Convert units/sec to units/update
const_m_to_mm           long    1000.0

const_velAccScale       long    0.9995
const_velAltiScale      long    0.0005
 
const_velAccTrust       long    0.999
const_velAltiTrust      long    0.001  



'To do this, I'll need to move ALL variables into the DAT section, then write a routine to convert the relative pointers to absolute
'It'll be a pain, but it'll save a huge chunk of program space and RAM

'SEE:  http://forums.parallax.com/discussion/148580/fyi-using-the-operator-in-a-dat-section


'cmdStream_RelToAbs      long    @cmdStream_RelToAbs

'{
  'fgx = gx / GyroScale + errCorrX

commandStream_0
              long      @commandStream_0                'this value is used to determine the offset to add to subsequent instructions                                                
              
QuatUpdateCommands
              long      FLT#opFloat, @gx | (0<<16), @rx                         'rx = float(gx)
              long      FLT#opMul, @rx | (@const_GyroScale<<16), @rx            'rx /= GyroScale
              long      FLT#opAdd, @rx | (@errCorrX<<16), @rx                   'rx += errCorrX

  'fgy = gy / GyroScale + errCorrY
              long      FLT#opFloat, @gz | (0<<16), @ry                         'ry = float(gz)
              long      FLT#opMul, @ry | (@const_NegGyroScale<<16), @ry         'ry /= GyroScale
              long      FLT#opAdd, @ry | (@errCorrY<<16), @ry                   'ry += errCorrY

  'fgz = gz / GyroScale + errCorrZ
              long      FLT#opFloat, @gy | (0<<16), @rz                         'rz = float(gy)
              long      FLT#opMul, @rz | (@const_NegGyroScale<<16), @rz         'rz /= GyroScale
              long      FLT#opAdd, @rz | (@errCorrZ<<16), @rz                   'rz += errCorrZ

  'rmag = sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) * 0.5 
                long      FLT#opSqr, @rx | (0<<16), @rmag                                  'rmag = fgx*fgx
                long      FLT#opSqr, @ry | (0<<16), @temp                                  'temp = fgy*fgy
                long      FLT#opAdd, @rmag | (@temp<<16), @rmag                            'rmag += temp
                long      FLT#opSqr, @rz | (0<<16), @temp                                  'temp = fgz*fgz
                long      FLT#opAdd, @rmag | (@temp<<16), @rmag                            'rmag += temp
                long      FLT#opAdd, @rmag | (@const_epsilon<<16), @rmag                   'rmag += 0.00000001
                long      FLT#opSqrt, @rmag | (0<<16), @rmag                               'rmag = Sqrt(rmag)                                                  
                long      FLT#opShift, @rmag | (@const_neg1<<16), @rmag                    'rmag *= 0.5                                                  
  '8 instructions  (17)

  'cosr = Cos(rMag)
  'sinr = Sin(rMag) / rMag
                long      FLT#opSinCos, @rmag | (@sinr<<16), @cosr                         'sinr = Sin(rmag), cosr = Cos(rmag)  
                long      FLT#opDiv, @sinr | (@rmag<<16), @sinr                            'sinr /= rmag                                                  
  '3 instructions  (20)

  'qdot.w =  (r.x*x + r.y*y + r.z*z) * -0.5
                long      FLT#opMul, @rx | (@qx<<16), @qdw                                 'qdw = rx*qx 
                long      FLT#opMul, @ry | (@qy<<16), @temp                                'temp = ry*qy
                long      FLT#opAdd, @qdw | (@temp<<16), @qdw                              'qdw += temp
                long      FLT#opMul, @rz | (@qz<<16), @temp                                'temp = rz*qz
                long      FLT#opAdd, @qdw | (@temp<<16), @qdw                              'qdw += temp
                long      FLT#opMul, @qdw | (@const_neghalf<<16), @qdw                     'qdw *= -0.5
  '8 instructions  (28)

  'qdot.x =  (r.x*w + r.z*y - r.y*z) * 0.5
                long      FLT#opMul, @rx | (@qw<<16), @qdx                                 'qdx = rx*qw 
                long      FLT#opMul, @rz | (@qy<<16), @temp                                'temp = rz*qy
                long      FLT#opAdd, @qdx | (@temp<<16), @qdx                              'qdx += temp
                long      FLT#opMul, @ry | (@qz<<16), @temp                                'temp = ry*qz
                long      FLT#opSub, @qdx | (@temp<<16), @qdx                              'qdx -= temp
                long      FLT#opShift, @qdx | (@const_neg1<<16), @qdx                      'qdx *= 0.5
  '8 instructions  (36)

  'qdot.y =  (r.y*w - r.z*x + r.x*z) * 0.5
                long      FLT#opMul, @ry | (@qw<<16), @qdy                                 'qdy = ry*qw 
                long      FLT#opMul, @rz | (@qx<<16), @temp                                'temp = rz*qx
                long      FLT#opSub, @qdy | (@temp<<16), @qdy                              'qdy -= temp
                long      FLT#opMul, @rx | (@qz<<16), @temp                                'temp = rx*qz
                long      FLT#opAdd, @qdy | (@temp<<16), @qdy                              'qdy += temp
                long      FLT#opShift, @qdy | (@const_neg1<<16), @qdy                      'qdy *= 0.5
  '8 instructions  (44)

  'qdot.z =  (r.z*w + r.y*x - r.x*y) * 0.5
                long      FLT#opMul, @rz | (@qw<<16), @qdz                                 'qdz = rz*qw 
                long      FLT#opMul, @ry | (@qx<<16), @temp                                'temp = ry*qx
                long      FLT#opAdd, @qdz | (@temp<<16), @qdz                              'qdz += temp
                long      FLT#opMul, @rx | (@qy<<16), @temp                                'temp = rx*qy
                long      FLT#opSub, @qdz | (@temp<<16), @qdz                              'qdz -= temp
                long      FLT#opShift, @qdz | (@const_neg1<<16), @qdz                      'qdz *= 0.5
  '8 instructions  (52)
   
  'q.w = cosr * q.w + sinr * qdot.w
                long      FLT#opMul, @cosr | (@qw<<16), @qw                                'qw = cosr*qw 
                long      FLT#opMul, @sinr | (@qdw<<16), @temp                             'temp = sinr*qdw
                long      FLT#opAdd, @qw | (@temp<<16), @qw                                'qw += temp

  'q.x = cosr * q.x + sinr * qdot.x
                long      FLT#opMul, @cosr | (@qx<<16), @qx                                'qx = cosr*qx 
                long      FLT#opMul, @sinr | (@qdx<<16), @temp                             'temp = sinr*qdx
                long      FLT#opAdd, @qx | (@temp<<16), @qx                                'qx += temp

  'q.y = cosr * q.y + sinr * qdot.y
                long      FLT#opMul, @cosr | (@qy<<16), @qy                                'qy = cosr*qy 
                long      FLT#opMul, @sinr | (@qdy<<16), @temp                             'temp = sinr*qdy
                long      FLT#opAdd, @qy | (@temp<<16), @qy                                'qy += temp

  'q.z = cosr * q.z + sinr * qdot.z
                long      FLT#opMul, @cosr | (@qz<<16), @qz                                'qz = cosr*qz 
                long      FLT#opMul, @sinr | (@qdz<<16), @temp                             'temp = sinr*qdz
                long      FLT#opAdd, @qz | (@temp<<16), @qz                                'qz += temp
  '12 instructions  (64)

  'q = q.Normalize()
  'rmag = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w + 0.0000001)
                long      FLT#opSqr, @qx | (0<<16), @rmag                                  'rmag = qx*qx 
                long      FLT#opSqr, @qy | (0<<16), @temp                                  'temp = qy*qy 
                long      FLT#opAdd, @rmag | (@temp<<16), @rmag                            'rmag += temp 
                long      FLT#opSqr, @qz | (0<<16), @temp                                  'temp = qz*qz 
                long      FLT#opAdd, @rmag | (@temp<<16), @rmag                            'rmag += temp 
                long      FLT#opSqr, @qw | (0<<16), @temp                                  'temp = qw*qw 
                long      FLT#opAdd, @rmag | (@temp<<16), @rmag                            'rmag += temp 
                long      FLT#opAdd, @rmag | (@const_epsilon<<16), @rmag                   'rmag += 0.0000001 
                long      FLT#opSqrt, @rmag | (0<<16), @rmag                               'sqrt(rmag) 
  '9 instructions (73)

  'q /= rmag   
                long      FLT#opDiv, @qw | (@rmag<<16), @qw                                'qw /= rmag 
                long      FLT#opDiv, @qx | (@rmag<<16), @qx                                'qx /= rmag 
                long      FLT#opDiv, @qy | (@rmag<<16), @qy                                'qy /= rmag 
                long      FLT#opDiv, @qz | (@rmag<<16), @qz                                'qz /= rmag 
  '4 instructions (77)


  'Now convert the updated quaternion to a rotation matrix 

  'fx2 = qx * qx;
  'fy2 = qy * qy;
  'fz2 = qz * qz;
                long    FLT#opSqr, @qx | (0<<16), @fx2                                   'fx2 = qx *qx
                long    FLT#opSqr, @qy | (0<<16), @fy2                                   'fy2 = qy *qy
                long    FLT#opSqr, @qz | (0<<16), @fz2                                   'fz2 = qz *qz
  '3 instructions (80)

  'fwx = qw * qx;
  'fwy = qw * qy;
  'fwz = qw * qz;
                long    FLT#opMul, @qw | (@qx<<16), @fwx                                 'fwx = qw *qx
                long    FLT#opMul, @qw | (@qy<<16), @fwy                                 'fwy = qw *qy
                long    FLT#opMul, @qw | (@qz<<16), @fwz                                 'fwz = qw *qz
  '3 instructions (83)

  'fxy = qx * qy;
  'fxz = qx * qz;
  'fyz = qy * qz;
                long    FLT#opMul, @qx | (@qy<<16), @fxy                                 'fxy = qx *qy
                long    FLT#opMul, @qx | (@qz<<16), @fxz                                 'fxz = qx *qz
                long    FLT#opMul, @qy | (@qz<<16), @fyz                                 'fyz = qy *qz
  '3 instructions (86)

   
  'm00 = 1.0f - 2.0f * (y2 + z2)
                long    FLT#opAdd, @fy2 | (@fz2<<16), @temp                              'temp = fy2+fz2
                long    FLT#opShift, @temp | (@const_1<<16), @temp                       'temp *= 2.0
                long    FLT#opSub, @const_F1 | (@temp<<16), @m00                         'm00 = 1.0 - temp
     
  'm01 =        2.0f * (fxy - fwz)
                long    FLT#opSub, @fxy | (@fwz<<16), @temp                              'temp = fxy-fwz
                long    FLT#opShift, @temp | (@const_1<<16), @m01                        'm01 = 2.0 * temp

  'm02 =        2.0f * (fxz + fwy)
                long    FLT#opAdd, @fxz | (@fwy<<16), @temp                              'temp = fxz+fwy
                long    FLT#opShift, @temp | (@const_1<<16), @m02                        'm02 = 2.0 * temp
  '7 instructions (93)

   
  'm10 =        2.0f * (fxy + fwz)
                long    FLT#opAdd, @fxy | (@fwz<<16), @temp                              'temp = fxy-fwz
                long    FLT#opShift, @temp | (@const_1<<16), @m10                        'm10 = 2.0 * temp

  'm11 = 1.0f - 2.0f * (x2 + z2)
                long    FLT#opAdd, @fx2 | (@fz2<<16), @temp                              'temp = fx2+fz2
                long    FLT#opShift, @temp | (@const_1<<16), @temp                       'temp *= 2.0
                long    FLT#opSub, @const_F1 | (@temp<<16), @m11                         'm11 = 1.0 - temp

  'm12 =        2.0f * (fyz - fwx)
                long    FLT#opSub, @fyz | (@fwx<<16), @temp                              'temp = fyz-fwx
                long    FLT#opShift, @temp | (@const_1<<16), @m12                        'm12 = 2.0 * temp
  '7 instructions (100)

   
  'm20 =        2.0f * (fxz - fwy)
                long    FLT#opSub, @fxz | (@fwy<<16), @temp                              'temp = fxz-fwz
                long    FLT#opShift, @temp | (@const_1<<16), @m20                        'm20 = 2.0 * temp

  'm21 =        2.0f * (fyz + fwx)
                long    FLT#opAdd, @fyz | (@fwx<<16), @temp                              'temp = fyz+fwx
                long    FLT#opShift, @temp | (@const_1<<16), @m21                        'm21 = 2.0 * temp

  'm22 = 1.0f - 2.0f * (x2 + y2)
                long    FLT#opAdd, @fx2 | (@fy2<<16), @temp                              'temp = fx2+fy2
                long    FLT#opShift, @temp | (@const_1<<16), @temp                       'temp *= 2.0
                long    FLT#opSub, @const_F1 | (@temp<<16), @m22                         'm22 = 1.0 - temp
  '7 instructions (107)



  'fax =  packet.ax;           // Acceleration in X (left/right)
  'fay =  packet.az;           // Acceleration in Y (up/down)
  'faz =  packet.ay;           // Acceleration in Z (toward/away)
               long     FLT#opFloat, @ax | (0<<16), @fax  
               long     FLT#opFloat, @az | (0<<16), @fay  
               long     FLT#opFloat, @ay | (0<<16), @faz  
               long     FLT#opNeg, @fax | (0<<16), @fax


'Rotation correction of the accelerometer vector - rotate around the pitch and roll axes by the specified amounts

  'axRot = (fax * accRollCorrCos) - (fay * accRollCorrSin)
              long  FLT#opMul, @fax | (@accRollCorrCos<<16), @axRot                           
              long  FLT#opMul, @fay | (@accRollCorrSin<<16), @temp
              long  FLT#opSub, @axRot | (@temp<<16), @axRot 

  'ayRot = (fax * accRollCorrSin) + (fay * accRollCorrCos)
              long  FLT#opMul, @fax | (@accRollCorrSin<<16), @ayRot                           
              long  FLT#opMul, @fay | (@accRollCorrCos<<16), @temp
              long  FLT#opAdd, @ayRot | (@temp<<16), @ayRot

  'fax = axRot         
  'fay = ayRot
              long  FLT#opMov, @axRot | (0<<16), @fax          
              long  FLT#opMov, @ayRot | (0<<16), @fay          



  'axRot = (faz * accPitchCorrCos) - (fay * accPitchCorrSin)
              long  FLT#opMul, @faz | (@accPitchCorrCos<<16), @axRot
              long  FLT#opMul, @fay | (@accPitchCorrSin<<16), @temp
              long  FLT#opSub, @axRot | (@temp<<16), @axRot 

  'ayRot = (fax * accPitchCorrSin) + (fay * accPitchCorrCos)
              long  FLT#opMul, @faz | (@accPitchCorrSin<<16), @ayRot                           
              long  FLT#opMul, @fay | (@accPitchCorrCos<<16), @temp
              long  FLT#opAdd, @ayRot | (@temp<<16), @ayRot

  'faz = axRot         
  'fay = ayRot
              long  FLT#opMov, @axRot | (0<<16), @faz          
              long  FLT#opMov, @ayRot | (0<<16), @fay          



'Compute length of the accelerometer vector to decide weighting                                   

  'rmag = facc.length
               long     FLT#opSqr, @fax | (0<<16), @rmag                                  'rmag = fax*fax
               long     FLT#opSqr, @fay | (0<<16), @temp                                  'temp = fay*fay
               long     FLT#opAdd, @rmag | (@temp<<16), @rmag                             'rmag += temp
               long     FLT#opSqr, @faz | (0<<16), @temp                                  'temp = faz*faz
               long     FLT#opAdd, @rmag | (@temp<<16), @rmag                             'rmag += temp
               long     FLT#opAdd, @rmag | (@const_epsilon<<16), @rmag                    'rmag += 0.00000001
               long     FLT#opSqrt, @rmag | (0<<16), @rmag                                'rmag = Sqrt(rmag)                                                  

  'facc /= rmag
               long     FLT#opDiv, @fax | (@rmag<<16), @faxn                              'faxn = fax / rmag 
               long     FLT#opDiv, @fay | (@rmag<<16), @fayn                              'fayn = fay / rmag 
               long     FLT#opDiv, @faz | (@rmag<<16), @fazn                              'fazn = faz / rmag 



  'accWeight = 1.0 - FMin( FAbs( 2.0 - accLen * 2.0 ), 1.0 )
               long     FLT#opMul, @rmag | (@const_AccScale<<16), @rmag                   'rmag /= accScale (accelerometer to 1G units)
               long     FLT#opShift, @rmag | (@const_1<<16), @accWeight                   'accWeight = rmag * 2.0
               long     FLT#opSub, @const_F2 | (@accWeight<<16), @accWeight               'accWeight = 2.0 - accWeight
               long     FLT#opFAbs, @accWeight | (0<<16), @accWeight                      'accWeight = FAbs(accWeight)
               long     FLT#opFMin, @accWeight | (@const_F1<<16), @accWeight              'accWeight = FMin( accWeight, 1.0 )
               long     FLT#opSub, @const_F1 | (@accWeight<<16), @accWeight               'accWeight = 1.0 - accWeight                                                

   

  'errDiffX = fayn * m12 - fazn * m11
               long     FLT#opMul, @fayn | (@m12<<16), @errDiffX 
               long     FLT#opMul, @fazn | (@m11<<16), @temp 
               long     FLT#opSub, @errDiffX | (@temp<<16), @errDiffX 

  'errDiffY = fazn * m10 - faxn * m12
               long     FLT#opMul, @fazn | (@m10<<16), @errDiffY 
               long     FLT#opMul, @faxn | (@m12<<16), @temp 
               long     FLT#opSub, @errDiffY | (@temp<<16), @errDiffY 

  'errDiffZ = faxn * m11 - fayn * m10
               long     FLT#opMul, @faxn | (@m11<<16), @errDiffZ 
               long     FLT#opMul, @fayn | (@m10<<16), @temp 
               long     FLT#opSub, @errDiffZ | (@temp<<16), @errDiffZ 

  'accWeight *= const_ErrScale   
               long     FLT#opMul, @const_ErrScale | (@accWeight<<16), @accWeight

  'Test: Does ErrCorr need to be rotated into the local frame from the world frame?


  'errCorr = errDiff * accWeight
               long      FLT#opMul, @errDiffX | (@accWeight<<16), @errCorrX  
               long      FLT#opMul, @errDiffY | (@accWeight<<16), @errCorrY  
               long      FLT#opMul, @errDiffZ | (@accWeight<<16), @errCorrZ  


    'tx := Flt.ASin( Flt.FFloatDiv28( DCM.GetM12 ) )     'Convert to float, then divide by (float)(1<<28)
    'tz := Flt.ASin( Flt.FFloatDiv28( DCM.GetM10 ) )     'Convert to float, then divide by (float)(1<<28) 

    'XAngle := Flt.FRound( Flt.FMul( tx , constant( 320000.0 / (PI / 2.0)) ) ) 
    'ZAngle := Flt.FRound( Flt.FMul( tz , constant(-320000.0 / (PI / 2.0)) ) )

    'if( DCM.GetMatrixvalue(4) < 0 )                     'If the Y value of the Y axis is negative, we're upside down
    '  if( ||ZAngle > ||XAngle ) 
    '    ZAngle := ZAngle 

    'For heading, I want an actual angular value, so this returns me an int between 0 & 65535, where 0 is forward
    'YAngle := Flt.FRound( Flt.FMul( Flt.Atan2( Flt.FFloat(DCM.GetM20), Flt.FFloat(DCM.GetM22)), constant(32768.0 / PI) ) ) & 65535 


               long      FLT#opASinCos, @m12 | (0<<16), @temp  
               long      FLT#opMul, @temp | (@const_outAngleScale<<16), @temp  
               long      FLT#opTruncRound, @temp | (@const_0<<16), @Pitch  
    
               long      FLT#opASinCos, @m10 | (0<<16), @temp  
               long      FLT#opMul, @temp | (@const_outAngleScale<<16), @temp  
               long      FLT#opTruncRound, @temp | (@const_0<<16), @Roll  
    
               long      FLT#opATan2, @m20 | (@m22<<16), @temp  
               long      FLT#opMul, @temp | (@const_outAngleScale<<16), @temp    
               long      FLT#opTruncRound, @temp | (@const_0<<16), @Yaw  


               long      FLT#opDiv, @const_F1 | (@m11<<16), @temp                          '1.0/m11 = scale factor for thrust - this will be infinite if perpendicular to ground   
               long      FLT#opMul, @temp | (@const_ThrustScale<<16), @temp                '*= 256.0  
               long      FLT#opTruncRound, @temp | (@const_0<<16), @ThrustFactor  



  'Compute the running height estimate

  'force := acc / 4096.0
               long      FLT#opShift, @fax | (@const_neg12<<16), @forceX    
               long      FLT#opShift, @fay | (@const_neg12<<16), @forceY    
               long      FLT#opShift, @faz | (@const_neg12<<16), @forceZ    

  'force -= m[1,0], m[1,1], m[1,2]  - Subtract gravity (1G, straight down)
               long      FLT#opSub, @forceX | (@m10<<16), @forceX    
               long      FLT#opSub, @forceY | (@m11<<16), @forceY    
               long      FLT#opSub, @forceZ | (@m12<<16), @forceZ    

  'forceWY := M.Transpose().Mul(Force).y                 'Orient force vector into world frame
  'forceWY = m01*forceX + m11*forceY + m21*forceZ

               long      FLT#opMul, @forceX | (@m01<<16), @forceWY  
   
               long      FLT#opMul, @forceY | (@m11<<16), @temp  
               long      FLT#opAdd, @forceWY | (@temp<<16), @forceWY  

               long      FLT#opMul, @forceZ | (@m21<<16), @temp  
               long      FLT#opAdd, @forceWY | (@temp<<16), @forceWY  

  'forceWY *= 9.8                                       'Convert to M/sec^2
               long      FLT#opMul, @forceWY | (@const_GMetersPerSec<<16), @forceWY  



               long      FLT#opMul, @forceWY | (@const_UpdateScale<<16), @temp             'temp := forceWY / UpdateRate
               long      FLT#opAdd, @velocityEstimate | (@temp<<16), @velocityEstimate     'velEstimate += forceWY / UpdateRate

  
               long      FLT#opFloat, @altRate | (0<<16), @altitudeVelocity                'AltVelocity = float(altRate)
               long      FLT#opMul, @altitudeVelocity | (@const_AltiVelScale<<16), @altitudeVelocity   'Convert from mm/sec to m/sec   


  'VelocityEstimate := (VelocityEstimate * 0.9950) + (altVelocity * 0.0050)
               long      FLT#opMul, @velocityEstimate | (@const_velAccScale<<16), @velocityEstimate 
               long      FLT#opMul, @altitudeVelocity | (@const_velAltiScale<<16), @temp  
               long      Flt#opAdd, @velocityEstimate | (@temp<<16), @velocityEstimate   

  'altitudeEstimate += velocityEstimate / UpdateRate
               long      FLT#opMul, @velocityEstimate | (@const_UpdateScale<<16), @temp  
               long      FLT#opAdd, @altitudeEstimate | (@temp<<16), @altitudeEstimate   

  'altitudeEstimate := (altitudeEstimate * 0.9950) * (alti / 1000.0) * 0.0050
               long      FLT#opMul, @altitudeEstimate | (@const_velAccTrust<<16), @altitudeEstimate 

               long      FLT#opFloat, @alt | (0<<16), @temp                               'temp := float(alt)
               long      FLT#opDiv, @temp | (@const_m_to_mm<<16), @temp                   'temp /= 1000.0    (alt now in m)
               long      FLT#opMul, @temp | (@const_velAltiTrust<<16), @temp              'temp *= 0.0050
               long      FLT#opAdd, @altitudeEstimate | (@temp<<16), @altitudeEstimate    'altEstimate += temp 


               long      FLT#opMul, @altitudeEstimate | (@const_m_to_mm<<16), @temp       'temp = altEst * 1000.0    (temp now in mm)
               long      FLT#opTruncRound, @temp | (@const_0<<16), @AltitudeEstMM 

               long      FLT#opMul, @velocityEstimate | (@const_m_to_mm<<16), @temp       'temp = velEst * 1000.0    (temp now in mm/sec)
               long      FLT#opTruncRound, @temp | (@const_0<<16), @VelocityEstMM 


  'Create a fixed point version of the orientation matrix
               long      FLT#opShift, @m00 | (@const_16<<16), @temp   
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm00 
               long      FLT#opShift, @m01 | (@const_16<<16), @temp   
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm01 
               long      FLT#opShift, @m02 | (@const_16<<16), @temp     
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm02 

               long      FLT#opShift, @m10 | (@const_16<<16), @temp   
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm10 
               long      FLT#opShift, @m11 | (@const_16<<16), @temp   
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm11 
               long      FLT#opShift, @m12 | (@const_16<<16), @temp   
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm12 

               long      FLT#opShift, @m20 | (@const_16<<16), @temp   
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm20 
               long      FLT#opShift, @m21 | (@const_16<<16), @temp   
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm21 
               long      FLT#opShift, @m22 | (@const_16<<16), @temp   
               long      FLT#opTruncRound, @temp | (@const_0<<16), @fm22
               long      0, 0, 0 
'}