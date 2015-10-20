''
''  Elev8 Flight Controller
''  Main flight core
''

CON
  _clkmode = xtal1 + pll16x
  '_xinfreq = 5_000_000
  _clkfreq = 80_000_000

  RC_THRO = 0
  RC_AILE = 1
  RC_ELEV = 2
  RC_RUDD = 3   'R/C input channel assignments (pin values are specified in the RC_Receiver object)
  RC_GEAR = 4
  RC_AUX1 = 5
  RC_AUX2 = 6
  RC_AUX3 = 7 

  'ESC output array indices for corresponding motors
  OUT_FL = 0
  OUT_FR = 1
  OUT_BR = 2
  OUT_BL = 3

  LED_COUNT = 2


  'Debug output modes for the Elev8-FC-Config application
  MODE_None = 0
  MODE_RadioTest = 1
  MODE_SensorTest = 2
  MODE_MotorTest = 3
  MODE_IMUTest = 4
  MODE_IMUComp = 5
  MODE_VibrationTest = 6


  'LED Color values
  LED_Red   = $00_ff_00
  LED_Green = $ff_00_00
  LED_Blue  = $00_00_ff
  LED_Yellow = LED_Red + LED_Green
  LED_Violet = LED_Red + LED_Blue
  LED_Cyan =   LED_Blue + LED_Green
  LED_DimCyan =((LED_Blue + LED_Green) & $FEFEFE) >> 1
  LED_White   = $ff_ff_ff

  'LED Brightness values - AND with color values to dim them
  LED_Full    = $ff_ff_ff  
  LED_Half    = $7f_7f_7f
  LED_Quarter = $3f_3f_3f
  LED_Eighth  = $1f_1f_1f
  LED_Dim     = $0f_0f_0f

  'For example...
  LED_Orange  = (LED_Red & LED_Full) | (LED_Green & LED_Half)  


  'Values used to when conditioning heading and desired heading to be within 180 degrees of each other 
  YawCircle = $2_0000           'IMU Yaw output reading is from 0 to YawCircle-1 (change here and in IMU if desired)
  YawMask = YawCircle - 1
  YawCircleHalf = YawCircle >> 1


  FlightMode_Assisted = 0
  FlightMode_Automatic = 1
  FlightMode_Manual = 2
  FlightMode_CalibrateCompass = 3

  

OBJ
  PIN  : "Pins-V3.spin"

  RC :   "RC_Receiver.spin"                             '1 cog
  SBUS : "SBUS-Receiver.spin"                           '   (shared cog with RC Receiver - only one runs at a time)
  
  Sens : "Sensors.spin"                                 '1 cog
  IMU :  "QuatIMU.spin"                                 '1 cogs (float command stream processor) 
  ESC :  "Servo32-HighRes.spin"                         '1 cog
  
  Dbg:   "FullDuplexSerial-32.spin"                     '1 cog (32-byte buffers, instead of 16)
  Const: "Constants.spin" 

  RollPID   : "IntPID.spin"
  PitchPID  : "IntPID.spin"
  YawPID    : "IntPID.spin"
  AscentPID : "IntPID.spin"

  TestPID   : "IntPID.spin"
  Ping      : "Ping.spin"

  Settings : "Settings.spin"
  eeprom : "Propeller Eeprom.spin"



VAR
  'Receiver inputs
  long  Thro, Aile, Elev, Rudd, Gear, Aux1, Aux2, Aux3
  long  iRudd         'Integrated rudder input value

  'Sensor inputs, in order of outputs from the Sensors cog, so they can be bulk copied for speed
  long  Temperature, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, MagX, MagY, MagZ, Alt, AltRate, AltTemp, Pressure
  long  SensorTime    'How long sensors took to read (debug / optimization test value)
  long  GyroZX, GyroZY, GyroZZ
  long  AccelZSmooth

  'Debug output mode, working variables  
  long  Mode, counter, NudgeMotor
  word  TxData[10]     'Word-sized copies of Temp, Gyro, Accel, Mag, for debug transfer speed
  byte  Quat[16]      'Current quaternion from the IMU functions

  'Current IMU values for orientation estimate
  long  Pitch, Roll, Yaw, AltiEst, AscentEst

  'Working variables - used to convert receiver inputs to desired ranges for the PIDs  
  long  DesiredRoll, DesiredPitch, DesiredYaw, DesiredAltitude, DesiredAltitudeFractional

  long  RollDifference, PitchDifference                 'Delta between current measured roll/pitch and desired roll/pitch                         
  long  GyroRoll, GyroPitch                             'Raw gyro values altered by desired pitch & roll targets
  long  GyroYaw   

  long  GyroRPFilter
  long  GyroYawFilter


  long  PitchOut, RollOut, YawOut                       'Output values from the PIDs
  long  ThroMix                           

  long  Motor[4]                                        'Motor output values  
  long  LEDValue[LED_COUNT]                             'LED outputs (copied to the LEDs by the Sensors cog)
  
  long loopTimer                'Master flight loop counter - used to keep a steady update rate

  long PingZero, PingHeight

  word FlightEnableStep         'Flight arm/disarm counter
  word CompassConfigStep        'Compass configure mode counter
    
  byte FlightEnabled            'Flight arm/disarm flag
  byte FlightMode
  byte NewFlightMode

  byte DoIntegrate              'Integration enabled in the flight PIDs
  byte UsePing              
  byte MotorIndex[4]            'Motor index to pin index table


  long RollPitch_P, RollPitch_D 
  long UseSBUS, SBUSCenter
  long LEDModeColor


  byte calib_startQuadrant
  byte calib_Quadrants
  byte calib_step
  long c_xmin, c_ymin, c_xmax, c_ymax, c_zmin, c_zmax



PUB Main | Cycles, PingCycle

  'Initialize everything - First reset all variables to known states

  MotorIndex[0] := PIN#MOTOR_FL
  MotorIndex[1] := PIN#MOTOR_FR
  MotorIndex[2] := PIN#MOTOR_BR
  MotorIndex[3] := PIN#MOTOR_BL


  Mode := MODE_None
  NudgeMotor := -1                                      'No motor to nudge
  FlightEnabled := FALSE 

  DesiredRoll := DesiredPitch := DesiredYaw := 0
  FlightEnableStep := 0                                 'Counter to know which section of enable/disable sequence we're in
  CompassConfigStep := 0
  FlightMode := FlightMode_Assisted
  GyroRPFilter := 192                                   'Tunable damping filters for gyro noise, 1 (HEAVY) to 256 (NO) filtering 
  GyroYawFilter := 192
  

  InitializeSettings
  

  'Configure and start all the objects
  Dbg.Start( 31, 30, 0, 115200 )
  'Dbg.Start( 24, 25, 0, 115200 )

  if( UseSBUS )
    SBUS.Start( PIN#RC_0 , SBUSCenter )                 'SBUS input is on RC_0 input (Throttle channel)
  else  
    RC.Start

  All_LED( LED_Red & LED_Half )                         'LED red on startup
    
  Sens.Start(PIN#SDI, PIN#SDO, PIN#SCL, PIN#CS_AG, PIN#CS_M, PIN#CS_ALT, PIN#LED_PIN, @LEDValue, LED_COUNT)  
  IMU.Start

  DIRA[PIN#BUZZER_1] := 1           'Enable buzzer pins    
  DIRA[PIN#BUZZER_2] := 1


  'Debug code - Display the length of the FPU programs
  'Dbg.rx
  'Dbg.dec( IMU.GetQuatUpdateLen )
  'Dbg.tx(13)
  'Dbg.dec( IMU.GetCalcErrorLen )
  'dbg.tx( 13 )
  'dbg.rx


  'Initialize the ESC driver, specify 400Hz outputs for 4 motor pins
  ESC.Init( 400 )
  ESC.AddFastPin(PIN#MOTOR_FL)
  ESC.AddFastPin(PIN#MOTOR_FR)
  ESC.AddFastPin(PIN#MOTOR_BR)
  ESC.AddFastPin(PIN#MOTOR_BL)

  ESC.Set(PIN#MOTOR_FL, 8000) 'Throttle ranges from 8000 to 16000 - 8000 is "off"
  ESC.Set(PIN#MOTOR_FR, 8000)
  ESC.Set(PIN#MOTOR_BR, 8000)
  ESC.Set(PIN#MOTOR_BL, 8000)
  ESC.Start


  RollPitch_P := 10000           'Set here to allow an in-flight tuning baseline
  RollPitch_D := 30000 * 250 


  RollPID.Init( RollPitch_P, 0,  RollPitch_D , Const#UpdateRate )               
  RollPID.SetPrecision( 12 ) 
  RollPID.SetMaxOutput( 3000 )
  RollPID.SetPIMax( 100 )
  RollPID.SetMaxIntegral( 1900 )
  RollPID.SetDervativeFilter( 96 ) 


  PitchPID.Init( RollPitch_P, 0,  RollPitch_D , Const#UpdateRate )               
  PitchPID.SetPrecision( 12 ) 
  PitchPID.SetMaxOutput( 3000 )
  PitchPID.SetPIMax( 100 )
  PitchPID.SetMaxIntegral( 1900 )
  PitchPID.SetDervativeFilter( 96 ) 


  YawPID.Init(  5000,   0,  -3750000 , Const#UpdateRate ) 
  YawPID.SetPrecision( 12 ) 
  YawPID.SetMaxOutput( 5000 )
  YawPID.SetDervativeFilter( 96 ) 


  'Altitude hold PID object
  AscentPID.Init( 1000, 0, 250000 , Const#UpdateRate )
  AscentPID.SetPrecision( 12 ) 
  AscentPID.SetMaxOutput( 4000 )
  AscentPID.SetPIMax( 100 )
  AscentPID.SetMaxIntegral( 3000 )
  AscentPID.SetDervativeFilter( 96 ) 

  if( UsePing )
    Ping.Fire(PIN#PING)


  FindGyroZero     'Get a gyro baseline - We'll re-do this on flight arming, but this helps settle the IMU

  InitTestPID

  if( UsePing )
    PingZero := Ping.Millimeters(PIN#PING)

  
  'Grab the first set of sensor readings (should be ready by now)
  longmove( @Temperature, Sens.Address, constant(Sens#ParamsSize) ) 

  'Set a reasonable starting point for the altitude computation
  IMU.SetInitialAltitudeGuess( Alt )


  counter := 0
  loopTimer := cnt                                      'Timekeeping value - tracks the next 400th/sec interval for IMU accuracy

  repeat
    Cycles := cnt

    'Read ALL inputs from the sensors into local memory, starting at Temperature
    longmove( @Temperature, Sens.Address, constant(Sens#ParamsSize) ) 

    IMU.Update_Part1( @GyroX )        'Entire IMU takes ~92000 cycles

    AccelZSmooth += (AccelZ - AccelZSmooth) / 16    


    if( UseSBUS )
      Thro :=  SBUS.GetRC( RC_THRO )
      Aile :=  SBUS.GetRC( RC_AILE )
      Elev :=  SBUS.GetRC( RC_ELEV )
      Rudd :=  SBUS.GetRC( RC_RUDD )
      Gear :=  SBUS.GetRC( RC_GEAR )
      Aux1 :=  SBUS.GetRC( RC_AUX1 )
      Aux2 :=  SBUS.GetRC( RC_AUX2 )
      Aux3 :=  SBUS.GetRC( RC_AUX3 )
    else
      Thro :=  RC.GetRC( RC_THRO )
      Aile :=  RC.GetRC( RC_AILE )
      Elev :=  RC.GetRC( RC_ELEV )
      Rudd :=  RC.GetRC( RC_RUDD )
      Gear :=  RC.GetRC( RC_GEAR )
      Aux1 :=  RC.GetRC( RC_AUX1 )
      Aux2 :=  RC.GetRC( RC_AUX2 )
      Aux3 :=  RC.GetRC( RC_AUX3 )


    if( UsePing )
      PingCycle := counter & 15
      if( PingCycle == 0 )
        Ping.Fire( PIN#PING )
      elseif( PingCycle == 15 )
        PingHeight := Ping.Millimeters( PIN#PING ) - PingZero 
       

      '-------------------------------------------------
    if( FlightMode == FlightMode_CalibrateCompass )

      DoCompassCalibrate

      '-------------------------------------------------
    else
      '-------------------------------------------------
      if( Gear < -512 )
        NewFlightMode := FlightMode_Assisted
      elseif( Gear > 512 )
        NewFlightMode := FlightMode_Manual
      else
        NewFlightMode := FlightMode_Automatic


      if( NewFlightMode <> FlightMode )

        if( NewFlightMode == FlightMode_Automatic )
          DesiredAltitude := AltiEst                     

        FlightMode := NewFlightMode
        

      UpdateFlightLoop            '~82000 cycles when in flight mode
      '-------------------------------------------------

    All_LED( LEDModeColor )

    'FlightModeTest

    IMU.WaitForCompletion

    Pitch := IMU.GetPitch
    Roll  := IMU.GetRoll        'Yes, this puts these 1 cycle behind, but the flight loop gets to use current gyro values
    Yaw   := IMU.GetYaw
    AltiEst := IMU.GetAltitudeEstimate
    AscentEst := IMU.GetVerticalVelocityEstimate 

    CheckDebugMode
    DoDebugModeOutput

    'dbg.dec( AltiEst )
    'dbg.tx( 32 )
    'dbg.dec( AscentEst )
    'dbg.tx( 13 )


    Cycles := cnt - Cycles
    'dbg.tx(1)
    'dbg.dec( Cycles )
    'dbg.tx(13)


    ++counter
    loopTimer += Const#UpdateCycles
    waitcnt( loopTimer )



PUB InitializeSettings
  ''Load user settings from EEPROM and apply

  Settings.Load
  ApplySettings


PUB ApplySettings
  ''Apply the settings from the settings object to all the objects that need the values

  Sens.SetDriftValues( Settings.GetAddress(Settings#DriftScalePref) )
  Sens.SetAccelOffsetValues( Settings.GetAddress(Settings#AccelOffsetPref) )
  Sens.SetMagnetometerScaleOffsets( Settings.GetAddress(Settings#MagScaleOfsPref) )

  IMU.SetRollCorrection( Settings.GetAddress(Settings#RollCorrectPref) )
  IMU.SetPitchCorrection( Settings.GetAddress(Settings#PitchCorrectPref) )

  UseSBUS := Settings.GetValue(Settings#UseSBUSPref)
  SBUSCenter := Settings.GetValue(Settings#SBUSCenterPref)
  UsePing := Settings.GetValue(Settings#UsePingPref)



PUB FindGyroZero | i

  GyroZX := 128
  GyroZY := 128
  GyroZZ := 128
  waitcnt( cnt + constant(_clkfreq / 4) )
  
  repeat i from 0 to 255
    GyroZX += Sens.In(1)
    GyroZY += Sens.In(2)
    GyroZZ += Sens.In(3)
    AccelX += Sens.In(4)
    AccelY += Sens.In(5)
    AccelZ += Sens.In(6)

    waitcnt( cnt + constant(_clkfreq / 2000) )      

  GyroZX /= 256  
  GyroZY /= 256  
  GyroZZ /= 256  
  AccelX /= 256  
  AccelY /= 256  
  AccelZ /= 256
  IMU.SetGyroZero( GyroZX, GyroZY, GyroZZ )   



PUB UpdateFlightLoop | ThroOut, T1, T2, ThrustMul, AltiThrust, v, gr, gp, gy

  UpdateFlightLEDColor

  'Test for flight mode change-----------------------------------------------
    
  if( FlightEnabled == FALSE )

    'Are the sticks being pushed down and toward the center?

    if( Thro < -750  AND  Elev < -750 )

      if( Rudd > 750  AND  Aile < -750 )
        FlightEnableStep++
        CompassConfigStep := 0
        LEDModeColor := LED_Yellow & LED_Half 
                
        if( FlightEnableStep == 250 )   'Hold for 1 second
          ArmFlightMode
        
      elseif( Rudd > 750  AND  Aile > 750 )
        CompassConfigStep++
        FlightEnableStep := 0

        LEDModeColor := (LED_Blue | LED_Red) & LED_Half 
                
        if( CompassConfigStep == 250 )   'Hold for 1 second
          StartCompassCalibrate

      else  
        CompassConfigStep := 0
        FlightEnableStep := 0
        
    else
      CompassConfigStep := 0
      FlightEnableStep := 0
    '------------------------------------------------------------------------
       
  else

    'Are the sticks being pushed down and away from center?

    if( Rudd < -750  AND  Aile > 750  AND  Thro < -750  AND  Elev < -750 )
      FlightEnableStep++
      LEDModeColor := LED_Yellow & LED_Half
              
      if( FlightEnableStep == 250 )   'Hold for 1 second
        DisarmFlightMode
        return                  'Prevents the motor outputs from being un-zero'd
      
    else
      FlightEnableStep := 0
    '------------------------------------------------------------------------

     
    'Rudder is integrated (accumulated)
    iRudd += Rudd


    if( FlightMode == FlightMode_Manual )

      RollDifference := Aile
      PitchDifference := -Elev


    else    
      'Angular output from the IMU is +/- 65536 units, or 32768 = 90 degrees
      'Input range from the controls is +/- 1000 units.  Scale that up to about 22.5 degrees       
       
      DesiredRoll :=  Aile << 3
      DesiredPitch := -Elev << 3

      RollDifference := (DesiredRoll - Roll) ~> 2
      PitchDifference := (DesiredPitch - Pitch) ~> 2


    gr := GyroY - GyroZY
    gp := -(GyroX - GyroZX)
    gy := -(GyroZ - GyroZZ)

    GyroRoll += ((gr - GyroRoll) * GyroRPFilter) ~> 8      
    GyroPitch += ((gp - GyroPitch) * GyroRPFilter) ~> 8      
    GyroYaw += ((gy - GyroYaw) * GyroYawFilter) ~> 8 
       

    'Yaw is different because we accumulate it - It's not specified absolutely like you can
    'with pitch and roll, so scale the stick input down a bit
    DesiredYaw := iRudd >> 3


     
    'Zero yaw target when throttle is off - makes for more stable liftoff
     
    if( Thro < -700 )
      DoIntegrate := FALSE      'Disable PID integral term until throttle is applied      
      DesiredYaw := Yaw         'Desired = measured when the throttle is off
      iRudd := Yaw << 3         'Make "measured yaw" match desired yaw until throttle is applied
    else
      DoIntegrate := TRUE
     
     
    DesiredYaw &= YawMask
     
    if( ||(DesiredYaw - Yaw) => YawCircleHalf )
      if( Yaw < YawCircleHalf )
        DesiredYaw := DesiredYaw - YawCircle
      else
        DesiredYaw := DesiredYaw + YawCircle      


    'Uncomment to allow partial PID tuning from the controller in flight
    {    
    T1 := RollPitch_P + Aux2<<1        'Left side control
    T2 := RollPitch_D + Aux3<<2        'Right side control 
                    
    RollPID.SetPGain( T1 ) 
    RollPID.SetDGain( T2 )                                           
    PitchPID.SetPGain( T1 )
    PitchPID.SetDGain( T2 )
    '}       
     
    RollOut := RollPID.Calculate_NoD2( RollDifference , GyroRoll , DoIntegrate )
    PitchOut := PitchPID.Calculate_NoD2( PitchDifference , GyroPitch , DoIntegrate )
    YawOut := YawPID.Calculate_ForceD_NoD2( DesiredYaw , Yaw , GyroYaw, DoIntegrate )
        

    ThroMix := (Thro + 1024) ~> 2                       ' Approx 0 - 512
    ThroMix <#= 64                                      ' Above 1/8 throttle, clamp it to 64
    ThroMix #>= 0
     
    'add 12000 to all Output values to make them 'servo friendly' again   (12000 is our output center)
    ThroOut := (Thro << 2) + 12000


    '-------------------------------------------
    if( FlightMode <> FlightMode_Manual )


      if( FlightMode == FlightMode_Automatic )
        if( ||Thro > 100 )
          DesiredAltitudeFractional += Thro ~> 1
          DesiredAltitude += (DesiredAltitudeFractional ~> 8)
          DesiredAltitudeFractional -= (DesiredAltitudeFractional & $FFFFFF00)   


        T1 := (Aux2 + 1024) << 1        'Left side control
        T2 := (Aux3 + 1024) << 8        'Right side control 
         
        AscentPID.SetPGain( T1 )
        AscentPID.SetDGain( T2 )

      
        AltiThrust := AscentPID.Calculate_NoD2( DesiredAltitude , AltiEst , DoIntegrate )
        ThroOut := 12000 + Thro + AltiThrust

      else

        'Accelerometer assist    
        if( ||Aile < 300 AND ||Elev < 300 AND ThroMix > 32) 'Above 1/8 throttle, add a little AccelZ into the mix if the user is trying to hover
          ThroOut -= (AccelZSmooth - Const#OneG) ~> 1

      'Tilt compensated thrust assist      
      ThrustMul := 256 #> IMU.GetThrustFactor <# 384    'Limit the effect of the thrust modifier 
      ThroOut := 8000 + (((ThroOut-8000) * ThrustMul) ~> 8)
    '-------------------------------------------

     
    ' X configuration
    Motor[OUT_FL] := ThroOut + ((+PitchOut + RollOut - YawOut) * ThroMix) ~> 7                          
    Motor[OUT_FR] := ThroOut + ((+PitchOut - RollOut + YawOut) * ThroMix) ~> 7  
    Motor[OUT_BL] := ThroOut + ((-PitchOut + RollOut + YawOut) * ThroMix) ~> 7
    Motor[OUT_BR] := ThroOut + ((-PitchOut - RollOut - YawOut) * ThroMix) ~> 7


    'The low-throttle clamp prevents combined PID output from sending the ESCs below a minimum value
      'the ESCs appear to stall (go into "stop" mode) if the throttle gets too close to zero, even for a moment
     
    Motor[0] := 8500 #> Motor[0] <# 16000
    Motor[1] := 8500 #> Motor[1] <# 16000
    Motor[2] := 8500 #> Motor[2] <# 16000
    Motor[3] := 8500 #> Motor[3] <# 16000
     
    'Copy new Ouput array into servo values
    ESC.Set( PIN#MOTOR_FL, Motor[0] )
    ESC.Set( PIN#MOTOR_FR, Motor[1] )
    ESC.Set( PIN#MOTOR_BR, Motor[2] )
    ESC.Set( PIN#MOTOR_BL, Motor[3] )


{
    if( (counter & 3) == 0 )
      dbg.txfast( 1 )
      dbg.txfast( 9 )
      dbg.dec( desiredYaw )
    elseif( (counter & 3) == 1 )
      dbg.tx( 11 )
      dbg.txfast( 9 )
      dbg.dec( Yaw )
    elseif( (counter & 3) == 2 )
      dbg.tx( 11 )
      dbg.txfast( 9 )
      dbg.dec( DesiredYaw - Yaw )
      dbg.tx( 11 )
}       

PUB UpdateFlightLEDColor | index, color

  index := (counter >> 4) & 15
  if( index == 0 )
    LEDModeColor := (long[@LEDColorTable][FlightMode] & $FE_FE_FE) >> 1 
  else
    LEDModeColor := (long[@LEDArmDisarm][FlightEnabled & 1] & $FE_FE_FE) >> 1



PUB InitTestPID

  TestPID.Init(  4550, 475*250,  8400*250 , Const#UpdateRate )               
  TestPID.SetPrecision( 12 ) 
  TestPID.SetMaxOutput( 3000 )
  TestPID.SetPIMax( 110 )
  TestPID.SetMaxIntegral( 1900 )

  'Forcing zero because the tethered quad sways when idle
  GyroZY := 23   'Note that this will be custom for each person's board



PUB FlightModeTest | ThroOut, CurrentRoll, T1, T2, T3

  'Function to test a tethered quad about the ROLL axis only.  This mode has no arm/disarm
  'sequence and the throttle is ALWAYS ON.  Be careful when using.  Added to allow quick PID tuning
   

    DoIntegrate := (Thro > -750 )
    DesiredRoll :=  Aile << 3

    RollDifference := (DesiredRoll - Roll) ~> 2
    
    GyroRoll := GyroY - GyroZY

    'Uncomment to use the knobs on a controller to play with PID parameters in real time, output them
    {
    T1 := Aux1+1024
    T2 := Aux3+1024
    T2 := (T2*T2) >> 7
    T3 := Aux2+1024

    dbg.txFast(1)
    dbg.hex( T1, 3 )
    dbg.txFast( 32 )
    dbg.hex( T2 , 4 )
    dbg.txFast( 32 )
    dbg.hex( T3 , 3 )
    dbg.txFast( 32 )

    TestPID.SetPIMax( T1 )
    TestPID.SetMaxIntegral( T2 )
    TestPID.SetMaxIntegral( T3 )
    }

    RollOut := TestPID.Calculate( RollDifference , GyroRoll , DoIntegrate )    

    PitchOut := 0
    YawOut := 0


    ThroMix := (Thro + 800) ~> 3                        ' Approx 0 - 256
    ThroMix <#= 64                                      ' Above 1/4 throttle, clamp it to 64
    ThroMix #>= 0
     
    'add 3000 to all Output values to make them 'servo friendly' again   (3000 is our output center)
    ThroOut := (Thro + 3000) << 2
     
    ' X configuration
    Motor[OUT_FL] := ThroOut + ((+PitchOut + RollOut + YawOut) * ThroMix) ~> 7                          
    Motor[OUT_FR] := ThroOut + ((+PitchOut - RollOut - YawOut) * ThroMix) ~> 7  
    Motor[OUT_BL] := ThroOut + ((-PitchOut + RollOut - YawOut) * ThroMix) ~> 7
    Motor[OUT_BR] := ThroOut + ((-PitchOut - RollOut + YawOut) * ThroMix) ~> 7

     
    Motor[0] := 8000 #> Motor[0] <# 16000
    Motor[1] := 8000 #> Motor[1] <# 16000
    Motor[2] := 8000 #> Motor[2] <# 16000
    Motor[3] := 8000 #> Motor[3] <# 16000
     
    'Copy new Ouput array into servo values
    ESC.Set( PIN#MOTOR_FL, Motor[0] )
    ESC.Set( PIN#MOTOR_FR, Motor[1] )
    ESC.Set( PIN#MOTOR_BR, Motor[2] )
    ESC.Set( PIN#MOTOR_BL, Motor[3] )




PUB ArmFlightMode

  FlightEnabled := TRUE
  FlightEnableStep := 0
  CompassConfigStep := 0
  Beep2
   
  All_LED( LED_Red & LED_Half )
  FindGyroZero
   
  All_LED( LED_Blue & LED_Half )        
  BeepTune

  'DesiredVerticalSpeed := 0
  DesiredAltitude := AltiEst
  loopTimer := cnt


PUB DisarmFlightMode

  ESC.Set( PIN#MOTOR_FL, 8000 )
  ESC.Set( PIN#MOTOR_FR, 8000 )
  ESC.Set( PIN#MOTOR_BR, 8000 )
  ESC.Set( PIN#MOTOR_BL, 8000 )

  FlightEnabled := FALSE
  FlightEnableStep := 0
  CompassConfigStep := 0
  Beep3
  
  All_LED( LED_Green & LED_Half )        
  loopTimer := cnt



PUB StartCompassCalibrate

  ESC.Set( PIN#MOTOR_FL, 8000 )
  ESC.Set( PIN#MOTOR_FR, 8000 )
  ESC.Set( PIN#MOTOR_BR, 8000 )
  ESC.Set( PIN#MOTOR_BL, 8000 )

  FlightEnabled := FALSE
  FlightMode := FlightMode_CalibrateCompass

  Beep
  waitcnt( 10_000_000 + cnt )  
  Beep2
  waitcnt( 10_000_000 + cnt )  
  Beep

  calib_Step := 0  
  calib_quadrants := 0
  calib_startQuadrant := $ff
  
  c_xmin := c_ymin := c_zmin :=  10000
  c_xmax := c_ymax := c_zmax := -10000

  Sens.ZeroMagnetometerScaleOffsets

  loopTimer := cnt



PUB DoCompassCalibrate | q, xc, xs, yc, ys, zc, zs, xr, yr, zr, mr

  if( calib_Step == 0 )
    'First cycle requires the quad to be level, and spun 360 degrees

    'Check the roll & pitch to make sure they're within some tolerance of level
    if( ||Roll > 3000  OR  ||Pitch > 3000 )
      LEDModeColor := LED_Yellow & Led_Half

    else
      LEDModeColor := LED_Violet & Led_Half
      if( ((Counter >> 5) & 3 ) == 0 )
        LEDModeColor := LED_Green & Led_Half      

      'Monitor yaw to see which quadrant we're in, keep going until we're in the same one we started, and have touched all four
      q := Calib_ComputeQuadrant( long[IMU.GetFixedMatrix][6], long[IMU.GetFixedMatrix][8] ) 

      if( calib_startQuadrant == $ff )
        calib_startQuadrant := q

      if( ( |<q | calib_quadrants) <> calib_quadrants )
        BeepHz( 5000, 10 )
        loopTimer := cnt
        calib_quadrants |= |<q   

      c_xmin := magx <# c_xmin
      c_xmax := magx #> c_xmax 
      c_ymin := magy <# c_ymin
      c_ymax := magy #> c_ymax 
      c_zmin := magz <# c_zmin
      c_zmax := magz #> c_zmax 

      if( calib_quadrants == %1111  AND  q == calib_StartQuadrant )
        calib_Step := 1
        Beep3

        'Reset these for the next phase
        calib_quadrants := 0                                
        calib_startQuadrant := $ff
        
        loopTimer := cnt        'Keep the outer counter happy - the beep has a delay, which messes it up
        return


  if( calib_Step == 1 )

    'Check to make sure the craft is vertical along the PITCH axis, nose up
    if( ||Pitch < 29_000 )
      LEDModeColor := LED_Yellow & Led_Half

    else
      LEDModeColor := LED_Violet & Led_Half
      if( ((Counter >> 5) & 3 ) == 0 )
        LEDModeColor := LED_Green & Led_Half      


      'Monitor rotation around the Z axis to see which quadrant we're in, otherwise same as above
      q := Calib_ComputeQuadrant( long[IMU.GetFixedMatrix][0], long[IMU.GetFixedMatrix][1] ) 


      if( calib_startQuadrant == $ff )
        calib_startQuadrant := q


      if( ( |<q | calib_quadrants) <> calib_quadrants )
        BeepHz( 5000, 10 )
        loopTimer := cnt
        calib_quadrants |= |<q   

      c_xmin := magx <# c_xmin
      c_xmax := magx #> c_xmax 
      c_ymin := magy <# c_ymin
      c_ymax := magy #> c_ymax 
      c_zmin := magz <# c_zmin
      c_zmax := magz #> c_zmax 

      if( calib_quadrants == %1111  AND  q == calib_StartQuadrant )
        calib_Step := 2
        loopTimer := cnt        'Keep the outer counter happy - the beep has a delay, which messes it up
        return


  if( calib_Step == 2 )
    'Yay!  We're done! 


    'Compute the necessary scales and offsets
    'xr = x_range
    'xc = x center
    'mr = maximum range of all 3 components
    'xs = x scale, IE a multiplier that such that (x * mult) / 2048 will normalize readings relative to each other

    xr := c_xmax - c_xmin
    yr := c_ymax - c_ymin
    zr := c_zmax - c_zmin

    mr := xr #> yr #> zr

    xc := (c_xmin + c_xmax) / 2
    yc := (c_ymin + c_ymax) / 2
    zc := (c_zmin + c_zmax) / 2

    xs := (mr * 2048) / xr    
    ys := (mr * 2048) / yr    
    zs := (mr * 2048) / zr    


    longmove( Settings.GetAddress(Settings#MagScaleOfsPref) , @xc , 6 )
    ApplySettings
    Settings.Save


    FlightMode := FlightMode_Assisted
     
    Beep2
    waitcnt( 10_000_000 + cnt )  
    Beep2
    waitcnt( 10_000_000 + cnt )  
    Beep2

    loopTimer := cnt        'Keep the outer counter happy - the beep has a delay, which messes it up
     



PUB Calib_ComputeQuadrant( x, y )
 
  result := 0
  if( x < 0 )
    if( y < 0 )
      result := 2
    else
      result := 3
  else
    if( y < 0 )
      result := 1
    else
      result := 0


PUB CheckDebugMode | c, gsx, gsy, gsz, gox, goy, goz, aox, aoy, aoz, i, rollOfsSin, rollOfsCos, pitchOfsSin, pitchOfsCos

  c := dbg.rxcheck
  if( c < 0 )
    return

  if( c =< MODE_VibrationTest )
    Mode := c
    return

   
  if( (c & $F8) == 8 )  'Nudge one of the motors
    if( Mode == MODE_MotorTest )
      NudgeMotor := c & 7  'Nudge the motor selected by the configuration app

  if( (c & $F8) == $10 )  'Zero, Reset, or set gyro or accelerometer calibration
    if( Mode == MODE_SensorTest )
      if( c == $10 )
        'Temporarily zero gyro drift settings
        Sens.TempZeroDriftValues

      elseif( c == $11 )
        'Reset gyro drift settings
        Sens.ResetDriftValues

      elseif( c == $12 )
        'Write new gyro drift settings - followed by 6 WORD values
        gsx := (dbg.rx << 8) | dbg.rx
        gsy := (dbg.rx << 8) | dbg.rx
        gsz := (dbg.rx << 8) | dbg.rx
        gox := (dbg.rx << 8) | dbg.rx
        goy := (dbg.rx << 8) | dbg.rx
        goz := (dbg.rx << 8) | dbg.rx

        'Sign extend all the values from 16 bit to 32 bit
        ~~gsx
        ~~gsy
        ~~gsz
        ~~gox
        ~~goy
        ~~goz

        'Copy the values into the settings object
        longmove( Settings.GetAddress(Settings#DriftScalePref), @gsx, 6 )
        
        ApplySettings                                                           'Apply the settings changes
        Settings.Save
        loopTimer := cnt                                                        'Reset the loop counter in case we took too long 

      elseif( c == $14 )
        'Temporarily zero accel offset settings
        Sens.TempZeroAccelOffsetValues

      elseif( c == $15 )
        'Reset accel offset settings
        Sens.ResetAccelOffsetValues

      elseif( c == $16 )
        'Write new accelerometer offset settings - followed by 3 WORD values
        aox := (dbg.rx << 8) | dbg.rx
        aoy := (dbg.rx << 8) | dbg.rx
        aoz := (dbg.rx << 8) | dbg.rx

        'Sign extend the values from 16 to 32 bit
        ~~aox
        ~~aoy
        ~~aoz        

        'Copy the values into the settings object
        longmove( Settings.GetAddress(Settings#AccelOffsetPref), @aox, 3 )
        
        ApplySettings                                                           'Apply the settings changes
        Settings.Save
        loopTimer := cnt                                                        'Reset the loop counter in case we took too long 


      elseif( c == $17 )
        'Write new accelerometer rotation settings - followed by 4 FLOAT values (Sin.Cos, Sin,Cos)
        repeat i from 0 to 15 
          byte[@rollOfsSin][i] := dbg.rx

        'Copy the values into the settings object
        longmove( Settings.GetAddress(Settings#RollCorrectPref), @rollOfsSin, 4 )
        
        ApplySettings                                                           'Apply the settings changes
        Settings.Save
        loopTimer := cnt                                                        'Reset the loop counter in case we took too long 




  if( (c & $f8) == $18 )        'Modify a flag setting, like PWM/SBUS, Ping, etc  
    if( c == $18 )              'Receiver type (PWM / SBUS)
      Settings.SetValue( Settings#UseSBUSPref , dbg.rx )
      Settings.Save
      loopTimer := cnt                                                          'Reset the loop counter in case we took too long 

  if( c == $ff )
    dbg.tx($E8)  'Simple ping-back to tell the application we have the right comm port



PUB DoDebugModeOutput | loop, addr, phase, i
  if( Mode == MODE_None )
    return

  phase := counter & 3


  if( Mode == MODE_RadioTest )
    if( phase == 0 )
      dbg.txFast( $77 )      
      dbg.txFast( $77 )      
      dbg.txFast( MODE_RadioTest )

      TxData[0] := Thro         'Copy the values we're interested in into a WORD array, for faster transmission                        
      TxData[1] := Aile
      TxData[2] := Elev
      TxData[3] := Rudd
      TxData[4] := Gear
      TxData[5] := Aux1
      TxData[6] := Aux2
      TxData[7] := Aux3

      dbg.txBulk( @TxData, 16 )   'Send 16 bytes of data from @TxData onward (sends 8 words worth of data)                         


  elseif( Mode == MODE_SensorTest )
    if( phase == 0 )
      dbg.txFast( $77 )      
      dbg.txFast( $77 )      
      dbg.txFast( Mode )

      TxData[0] := Temperature    'Copy the values we're interested in into a WORD array, for faster transmission                        
      TxData[1] := GyroX
      TxData[2] := GyroY
      TxData[3] := GyroZ
      TxData[4] := AccelX
      TxData[5] := AccelY
      TxData[6] := AccelZ
      TxData[7] := MagX
      TxData[8] := MagY
      TxData[9] := MagZ

      dbg.txBulk( @TxData, 20 )   'Send 20 bytes of data from @TxData onward (sends 10 words worth of data)                         
      
    elseif( phase == 2 )
    
      dbg.txBulk( @Alt, 4 )       'Send 4 bytes of data for @Alt
      dbg.txBulk( @AltTemp, 4 )   'Send 4 bytes of data for @AltTemp
      dbg.txBulk( @AltiEst, 4 )   'Send 4 bytes for altitude estimate 
       

  elseif( Mode == MODE_MotorTest )

    'Motor test code---------------------------------------
    if( NudgeMotor > -1 )

      if( NudgeMotor < 4 )     
        ESC.Set(MotorIndex[NudgeMotor], 9000)           'Motor test - 1/8 throttle
         
        'waitcnt( cnt + 10_000_000 )
        'ESC.Set(NudgeMotor, 1000)
        
      elseif( NudgeMotor == 4 )                         'Buzzer test
        BeepHz(4500, 50)
        waitcnt( cnt + 5_000_000 )
        BeepHz(3500, 50)
        
      elseif( NudgeMotor == 5 )                         'LED test
        
        ' RGB led will run a rainbow
        repeat i from 0 to 255 step 1
          All_LED( (255-i)<<16 + (i<<8) ) 
          waitcnt( cnt + 160000 )

        repeat i from 0 to 255 step 1
          All_LED( i + ((255-i) << 8) )
          waitcnt( cnt + 160000 )

        repeat i from 0 to 255 step 1
          All_LED( (255-i) + i<<16 )
          waitcnt( cnt + 160000 )


      elseif( NudgeMotor == 6 )                         'ESC Throttle calibration

        BeepHz(4500, 100)
        waitcnt( cnt + 5_000_000 )
        BeepHz(4500, 100)
        waitcnt( cnt + 5_000_000 )
        BeepHz(4500, 100)
        waitcnt( cnt + 5_000_000 )
        BeepHz(4500, 100)

        if( dbg.rx == $FF )  'Safety check - Allow the user to break out by sending anything else                  
          ESC.Set(MotorIndex[0], 16000)
          ESC.Set(MotorIndex[1], 16000)
          ESC.Set(MotorIndex[2], 16000)
          ESC.Set(MotorIndex[3], 16000)
           
          dbg.rx
           
          ESC.Set(MotorIndex[0], 8000)
          ESC.Set(MotorIndex[1], 8000)
          ESC.Set(MotorIndex[2], 8000)
          ESC.Set(MotorIndex[3], 8000)

      elseif( NudgeMotor == 7 )                         'Motor off (after motor test)
        ESC.Set(MotorIndex[0], 8000)
        ESC.Set(MotorIndex[1], 8000)
        ESC.Set(MotorIndex[2], 8000)
        ESC.Set(MotorIndex[3], 8000)
        

      NudgeMotor := -1
      loopTimer := cnt
    'End Motor test code-----------------------------------
      
  elseif( Mode == MODE_IMUTest ) 

    if( phase == 0 )

      'Quaternions are sensitive.  Copy it locally so we can send it in pieces without messing it up     
      bytemove( @Quat, IMU.GetQuaternion, 16)

      dbg.txFast( $77 )      
      dbg.txFast( $77 )      
      dbg.txFast( Mode )

      dbg.txBulk( @Quat , 16 )

    elseif( phase == 3 )

      Dbg.txFast( Pitch )
      Dbg.txFast( Pitch >> 8 )
      Dbg.txFast( Roll )
      Dbg.txFast( Roll >> 8 )
      Dbg.txFast( Yaw )
      Dbg.txFast( Yaw >> 8 )

  elseif( Mode == MODE_IMUComp )

    if( (phase&1) == 0 )
      dbg.txFast( $77 )      
      dbg.txFast( $77 )      
      dbg.txFast( Mode )

      TxData[0] := GyroX          'Copy the values we're interested in into a WORD array, for faster transmission                        
      TxData[1] := GyroY
      TxData[2] := GyroZ
      TxData[3] := AccelX
      TxData[4] := AccelY
      TxData[5] := AccelZ

      dbg.txBulk( @TxData, 12 )   'Send 12 bytes of data from @TxData onward (sends 6 words worth of data)                         
      dbg.txBulk( @Alt, 4 )                         

  elseif( Mode == MODE_VibrationTest )
    dbg.txFast( $77 )      
    dbg.txFast( $77 )      
    dbg.txFast( Mode )
     
    TxData[0] := GyroX
    TxData[1] := GyroY
    TxData[2] := GyroZ
     
    dbg.txBulk( @TxData, 6 )   'Send 20 bytes of data from @TxData onward (sends 10 words worth of data)                         

      


PUB All_LED( Color )
  LongFill( @LEDValue[0], Color, LED_COUNT) 


PUB BeepHz( Hz , Delay ) | i, loop, d, ctr

  'Note that each loop does a high and low cycle, so we divide clkfreq by 2 and 2000 instead of 1 and 1000

  d := constant(_clkfreq/2) / Hz                        'Compute the amount of time to delay between pulses to get the right frequency
  loop := (Delay * constant(_clkfreq/2000)) / d         'How many iterations of the loop to make "Delay" milliseconds?

   
  if( PIN#BUZZER_1 == PIN#BUZZER_2 )

    'Revision 3 firmware has one buzzer pin  

    ctr := cnt
    repeat i from 0 to loop
      OUTA[PIN#BUZZER_1] := 1    
     
      ctr += d
      waitcnt( ctr )    
     
      OUTA[PIN#BUZZER_1] := 0    
     
      ctr += d
      waitcnt( ctr )    

  else

    'Revision 2 firmware has two buzzer pins  

    ctr := cnt
    repeat i from 0 to loop
      OUTA[PIN#BUZZER_1] := 1    
      OUTA[PIN#BUZZER_2] := 0
     
      ctr += d
      waitcnt( ctr )    
     
      OUTA[PIN#BUZZER_1] := 0    
      OUTA[PIN#BUZZER_2] := 1
     
      ctr += d
      waitcnt( ctr )    
     
    OUTA[PIN#BUZZER_1] := 0    
    OUTA[PIN#BUZZER_2] := 0
     


PUB BeepTune

  BeepHz( 1174, 150 )           'D5
  BeepHz( 1318, 150 )           'E5
  BeepHz( 1046, 150 )           'C5
  BeepHz(  522, 150 )           'C4
  BeepHz(  784, 300 )           'G4



PUB Beep
  BeepHz( 5000 , 80 )


PUB Beep2
  Beep
  waitcnt( 5_000_000 + cnt ) 
  Beep


PUB Beep3
  Beep
  waitcnt( 5_000_000 + cnt ) 
  Beep
  waitcnt( 5_000_000 + cnt ) 
  Beep
  

DAT
LEDColorTable
        LED_Assisted    long    LED_DimCyan
        LED_Automatic   long    LED_Green
        LED_Manual      long    LED_Yellow
        LED_CompCalib   long    LED_Violet

LEDArmDisarm
        LED_Disarmed    long    LED_Green
        LED_Armed       long    LED_Red
                                       