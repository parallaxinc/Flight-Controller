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
  AccToG = 1000.0 / 0.122                               'Accelerometer per G @ 4g sensitivity = 0.122 mg/bit 

  GyroScale = GyroToDeg * RadToDeg * float(Const#UpdateRate)
  
VAR
  long Roll, Pitch, Yaw                                 'Outputs, scaled units (

  'Inputs
  long zx, zy, zz                                       'Gyro zero readings
  long gx, gy, gz, ax, ay, az, mx, my, mz               'Sensor inputs 

  'Internal orientation storage
  long  qx, qy, qz, qw                                  'Body orientation quaternion

  long  m00, m01, m02                                   'Body orientation as a 3x3 matrix
  long  m10, m11, m12
  long  m20, m21, m22
  
  'Internal working variables - It isn't strictly necessary to break all of these out like this,
  'but it makes the code much more readable than having a bunch of temp variables
  
  long  qdx, qdy, qdz, qdw                              'Incremental rotation quaternion
  long  fx2, fy2, fz2, fwx, fwy, fwz, fxy, fxz, fyz     'Quaternion to matrix temp coefficients

  long rx, ry, rz                                       'Float versions of rotation components
  long fax, fay, faz                                    'Float version of accelerometer vector
  long rmag, cosr, sinr                                 'magnitude, cos, sin values
  long errDiffX, errDiffY, errDiffZ                     'holds difference vector between target and measured orientation
  long errCorrX, errCorrY, errCorrZ                     'computed rotation correction factor
  long temp                                             'temp value for use in equations                        
  long accWeight

  'Various constants used by the float math engine - Every command in the instruction stream reads two
  'arguments from memory using memory addresses, so the values actually need to exist somewhere
  long const_GyroScale, const_NegGyroScale, const_InvGyroScale
  long const_0, const_1, const_F1, const_F2
  long const_epsilon, const_half, const_neghalf
  long const_neg1, const_ErrScale, const_AccScale, const_outAngleScale

  long UpdateCount

  

VAR
  long  QuatUpdateCommands[300 + 132]
  'long  CalcErrorUpdateAngles[132]

  long  QuatUpdateLen   '295 longs
  'long  CalcErrorLen    '127 longs
  

PUB Start
  FLT.Start
  'Debug.start(31, 30, 0, 115200)
  UpdateCount := 0

  qx := 0.0
  qy := 0.0
  qz := 0.0
  qw := 1.0

  const_GyroScale := constant(1.0 / GyroScale)
  const_NegGyroScale := constant(-1.0 / GyroScale) 
  const_0 := 0
  const_1 := 1
  const_neg1 := -1
  
  const_F1 := 1.0
  const_F2 := 2.0
  const_epsilon := 0.0000000001
  const_half := 0.5
  const_neghalf := -0.5
  const_ErrScale := constant(1.0/512.0)                 'How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
  const_AccScale := constant(1.0/AccToG)                'Conversion factor from accel units to G's
  const_outAngleScale := constant(-65536.0 / PI)               
  InitFunctions


PUB GetPitch
  return Pitch

PUB GetRoll
  return Roll

PUB GetYaw
  return Yaw


PUB GetSensors
  return @gx

PUB GetMatrix
  return @m00

PUB GetQuaternion
  return @qx  


PUB InitFunctions

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

  'QuatUpdateLen := (FLT.EndStream( 0 ) - @QuatUpdateCommands) / 4 
   


  'FLT.StartStream( 1, @CalcErrorUpdateAngles )

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
  FLT.AddCommand( 0, FLT#opDiv, @fax, @rmag, @fax )                             'fax /= rmag 
  FLT.AddCommand( 0, FLT#opDiv, @fay, @rmag, @fay )                             'fay /= rmag 
  FLT.AddCommand( 0, FLT#opDiv, @faz, @rmag, @faz )                             'faz /= rmag 



  'accWeight = 1.0 - FMin( FAbs( 2.0 - accLen * 2.0 ), 1.0 )
  FLT.AddCommand( 0, FLT#opMul, @rmag, @const_AccScale, @rmag )                 'rmag /= accScale (accelerometer to 1G units)
  FLT.AddCommand( 0, FLT#opShift, @rmag, @const_1, @accWeight )                 'accWeight = rmag * 2.0
  FLT.AddCommand( 0, FLT#opSub, @const_F2, @accWeight, @accWeight )             'accWeight = 2.0 - accWeight
  FLT.AddCommand( 0, FLT#opFAbs, @accWeight, 0, @accWeight )                    'accWeight = FAbs(accWeight)
  FLT.AddCommand( 0, FLT#opFMin, @accWeight, @const_F1, @accWeight )            'accWeight = FMin( accWeight, 1.0 )
  FLT.AddCommand( 0, FLT#opSub, @const_F1, @accWeight, @accWeight )             'accWeight = 1.0 - accWeight                                                

   

  'errDiffX = fay * m12 - faz * m11
  FLT.AddCommand( 0, FLT#opMul, @fay, @m12, @errDiffX )
  FLT.AddCommand( 0, FLT#opMul, @faz, @m11, @temp )
  FLT.AddCommand( 0, FLT#opSub, @errDiffX, @temp, @errDiffX )

  'errDiffY = faz * m10 - fax * m12
  FLT.AddCommand( 0, FLT#opMul, @faz, @m10, @errDiffY )
  FLT.AddCommand( 0, FLT#opMul, @fax, @m12, @temp )
  FLT.AddCommand( 0, FLT#opSub, @errDiffY, @temp, @errDiffY )

  'errDiffZ = fax * m11 - fay * m10
  FLT.AddCommand( 0, FLT#opMul, @fax, @m11, @errDiffZ )
  FLT.AddCommand( 0, FLT#opMul, @fay, @m10, @temp )
  FLT.AddCommand( 0, FLT#opSub, @errDiffZ, @temp, @errDiffZ )

  'accWeight *= const_ErrScale   
  FLT.AddCommand( 0, FLT#opMul, @const_ErrScale, @accWeight, @accWeight )


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

  QuatUpdateLen := (FLT.EndStream( 0 ) - @QuatUpdateCommands) / 4 

  'CalcErrorLen := (FLT.EndStream( 1 ) - @CalcErrorUpdateAngles) / 4 
  


PUB SetGyroZero( _x, _y, _z )
  zx := _x
  zy := _y
  zz := _z
  

PUB Send( v )
  'Debug.tx(v >> 8)
  'Debug.tx(v & 255)



PUB GetQuatUpdateLen
  return QuatUpdateLen

'PUB GetCalcErrorLen
'  return CalcErrorLen


'If the body is mostly level, use the incoming mag readings as the compass vector
'Compute the difference between the actual compass heading and estimated heading,
'and apply that as an additional part of the errCorr vector (?)


PUB Update_Part1( packetAddr ) | v, t


  LongMove( @gx, packetAddr, 9 )

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
   