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

  'Output pins to corresponding motors
  MOTOR_FL = 15
  MOTOR_FR = 16
  MOTOR_BR = 17
  MOTOR_BL = 18

  'ESC output array indices for corresponding motors
  OUT_FL = 0
  OUT_FR = 1
  OUT_BR = 2
  OUT_BL = 3

  '        V2    V1
  CS_ALT = 9    '8
  CS_AG  = 11   '11
  SDO    = 13   '12
  SDI    = 14   '13
  SCL    = 12   '14
  CS_M   = 10   '15
  LED_PIN = 8   '19

  LED_COUNT = 2
  LED_TESTDELAY = 6_000_000

  BUZZER_1 = 6
  BUZZER_2 = 7

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

  

OBJ
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

  TestPID   : "IntPID.spin"

  eeprom : "Propeller Eeprom.spin"



VAR
  'Receiver inputs
  long  Thro, Aile, Elev, Rudd, Gear, Aux1, Aux2, Aux3
  long  iRudd         'Integrated rudder input value

  'Sensor inputs, in order of outputs from the Sensors cog, so they can be bulk copied for speed
  long  Temperature, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, MagX, MagY, MagZ, Alt, AltTemp, Pressure
  long  SensorTime    'How long sensors took to read (debug / optimization test value)
  long GyroZX, GyroZY, GyroZZ

  'Debug output mode, working variables  
  long  Mode, counter, NudgeMotor
  word  TxData[10]     'Word-sized copies of Temp, Gyro, Accel, Mag, for debug transfer speed
  byte  Quat[16]      'Current quaternion from the IMU functions

  'Current IMU values for orientation estimate
  long  Pitch, Roll, Yaw

  'Working variables - used to convert receiver inputs to desired ranges for the PIDs  
  long  DesiredRoll, DesiredPitch, DesiredYaw, DesiredAltitude

  long  RollDifference, PitchDifference                 'Delta between current measured roll/pitch and desired roll/pitch                         
  long  GyroRoll, GyroPitch                             'Raw gyro values altered by desired pitch & roll targets   


  long  PitchOut, RollOut, YawOut                       'Output values from the PIDs
  long  ThroMix                           

  long  Motor[4]                                        'Motor output values  
  long  LEDValue[LED_COUNT]                             'LED outputs (copied to the LEDs by the Sensors cog)
  
  long loopTimer                'Master flight loop counter - used to keep a steady update rate


  word EnableStep               'Flight arm/disarm counter  
  byte FlightEnabled            'Flight arm/disarm flag
  byte FlightMode

  byte DoIntegrate              'Integration enabled in the flight PIDs              
  byte MotorIndex[4]            'Motor index to pin index table


  long RollPitch_P, RollPitch_D 
  long UseSBUS, SBUSCenter
  


PUB Main | Cycles

  'Initialize everything - First reset all variables to known states

  MotorIndex[0] := MOTOR_FL
  MotorIndex[1] := MOTOR_FR
  MotorIndex[2] := MOTOR_BR
  MotorIndex[3] := MOTOR_BL


  Mode := MODE_None
  NudgeMotor := -1                                      'No motor to nudge
  FlightEnabled := FALSE 

  DesiredRoll := DesiredPitch := DesiredYaw := 0
  EnableStep := 0               'Counter to know which section of enable/disable sequence we're in

  FlightMode := FlightMode_Assisted

  'eeprom.ToRam(@UseSBUS, @UseSBUS+8, Const#UseSBUSPref )   ' Copy from EEPROM to VAR
  UseSBUS := 0
  SBUSCenter := 1000


  'Configure and start all the objects
  Dbg.Start( 31, 30, 0, 115200 )

  if( UseSBUS )
    SBUS.Start( 26 , SBUSCenter )                       'SBUS input is on PIN 26 (Throttle channel)
  else  
    RC.Start

  All_LED( LED_Red & LED_Half )                         'LED red on startup
    
  Sens.Start(SDI, SDO, SCL, CS_AG, CS_M, CS_ALT, LED_PIN, @LEDValue, LED_COUNT)  
  IMU.Start

  DIRA[BUZZER_1] := 1           'Enable buzzer pins    
  DIRA[BUZZER_2] := 1


  'Debug code - Display the length of the FPU programs
  'Dbg.rx
  'Dbg.dec( IMU.GetQuatUpdateLen )
  'Dbg.tx(13)
  'Dbg.dec( IMU.GetCalcErrorLen )
  'dbg.tx( 13 )
  'dbg.rx


  'Initialize the ESC driver, specify 400Hz outputs for 4 motor pins
  ESC.Init( 400 )
  ESC.AddFastPin(MOTOR_FL)
  ESC.AddFastPin(MOTOR_FR)
  ESC.AddFastPin(MOTOR_BR)
  ESC.AddFastPin(MOTOR_BL)

  ESC.Set(MOTOR_FL, 8000) 'Throttle ranges from 8000 to 16000 - 8000 is "off"
  ESC.Set(MOTOR_FR, 8000)
  ESC.Set(MOTOR_BR, 8000)
  ESC.Set(MOTOR_BL, 8000)
  ESC.Start


  RollPitch_P := 10000           'Set here to allow an in-flight tuning baseline
  RollPitch_D := 30000 


  RollPID.Init( RollPitch_P, 0,  RollPitch_D )               
  RollPID.SetPrecision( 12 ) 
  RollPID.SetMaxOutput( 3000 )
  RollPID.SetPIMax( 100 )
  RollPID.SetMaxIntegral( 1900 )


  PitchPID.Init( RollPitch_P, 0,  RollPitch_D )               
  PitchPID.SetPrecision( 12 ) 
  PitchPID.SetMaxOutput( 3000 )
  PitchPID.SetPIMax( 100 )
  PitchPID.SetMaxIntegral( 1900 )


  YawPID.Init(  5000,   0,  -15000 ) 
  YawPID.SetPrecision( 12 ) 
  YawPID.SetMaxOutput( 5000 )



  FindGyroZero     'Get a gyro baseline - We'll re-do this on flight arming, but this helps settle the IMU

  InitTestPID



  counter := 0
  loopTimer := cnt                                      'Timekeeping value - tracks the next 400th/sec interval for IMU accuracy

  repeat
    Cycles := cnt

    'Read ALL inputs from the sensors into local memory, starting at Temperature
    longmove( @Temperature, Sens.Address, constant(Sens#ParamsSize) ) 

    IMU.Update_Part1( @GyroX )        'Entire IMU takes ~92000 cycles


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
       

    UpdateFlightLoop            '~82000 cycles when in flight mode

    'FlightModeTest


    IMU.WaitForCompletion

    Pitch := IMU.GetPitch
    Roll  := IMU.GetRoll        'Yes, this puts these 1 cycle behind, but the flight loop gets to use current gyro values
    Yaw   := IMU.GetYaw


    CheckDebugMode
    DoDebugModeOutput

    Cycles := cnt - Cycles
    'dbg.tx(1)
    'dbg.dec( Cycles )
    'dbg.tx(13)


    ++counter
    loopTimer += Const#UpdateCycles
    waitcnt( loopTimer )



PUB FindGyroZero | i

  GyroZX := 0
  GyroZY := 0
  GyroZZ := 0

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



PUB UpdateFlightLoop | ThroOut, T1, T2


  'Test for flight mode change-----------------------------------------------
    
  if( FlightEnabled == FALSE )

    'Are the sticks being pushed down and toward the center?

    if( Rudd > 750  AND  Aile < -750  AND  Thro < -750  AND  Elev < -750 )
      EnableStep++
      All_LED( LED_Yellow & LED_Half )
              
      if( EnableStep == 250 )   'Hold for 1 second
        ArmFlightMode
      
    else
      EnableStep := 0
      All_LED( LED_Green & LED_Half )
    '------------------------------------------------------------------------

  else

    'Are the sticks being pushed down and away from center?

    if( Rudd < -750  AND  Aile > 750  AND  Thro < -750  AND  Elev < -750 )
      EnableStep++
      All_LED( LED_Yellow & LED_Half )
              
      if( EnableStep == 250 )   'Hold for 1 second
        DisarmFlightMode
        return                  'Prevents the motor outputs from being un-zero'd
      
    else
      EnableStep := 0
      All_LED( LED_Red & LED_Half )

    '------------------------------------------------------------------------

    if( Gear < -512 )
      FlightMode := FlightMode_Assisted
    elseif( Gear > 512 )
      FlightMode := FlightMode_Manual
    else
      FlightMode := FlightMode_Automatic


     
    'Rudder is integrated (accumulated)
    iRudd += Rudd


    if( FlightMode == FlightMode_Manual )
      'DesiredRoll +=  Aile / 8
      'DesiredPitch += Elev / 8

      RollDifference := Aile
      PitchDifference := Elev


    else    
      'Angular output from the IMU is +/- 65536 units, or 32768 = 90 degrees
      'Input range from the controls is +/- 1000 units.  Scale that up to about 22.5 degrees       
       
      DesiredRoll :=  Aile << 3
      DesiredPitch := Elev << 3

      RollDifference := (DesiredRoll - Roll) ~> 2
      PitchDifference := (DesiredPitch - Pitch) ~> 2

       
    GyroRoll := GyroY - GyroZY
    GyroPitch := GyroX - GyroZX
      
       

    'Yaw is different because we accumulate it - It's not specified absolutely like you can
    'with pitch and roll, so scale the stick input down a bit
    DesiredYaw :=   iRudd >> 3


     
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
    YawOut := YawPID.Calculate_ForceD_NoD2( DesiredYaw , Yaw , -GyroZ, DoIntegrate )
        

    ThroMix := (Thro + 800) ~> 3                        ' Approx 0 - 256
    ThroMix <#= 64                                      ' Above 1/4 throttle, clamp it to 64
    ThroMix #>= 0
     
    'add 3000 to all Output values to make them 'servo friendly' again   (3000 is our output center)
    ThroOut := (Thro + 3000) << 2

    'if( ||Aile < 300 AND ||Elev < 300 AND ThroMix > 32) 'Above 1/8 throttle, add a little AccelZ into the mix if the user is trying to hover
    '  ThroOut -= (AccelZ - 8192) ~> 2
     
    ' X configuration
    Motor[OUT_FL] := ThroOut + ((-PitchOut + RollOut - YawOut) * ThroMix) ~> 7                          
    Motor[OUT_FR] := ThroOut + ((-PitchOut - RollOut + YawOut) * ThroMix) ~> 7  
    Motor[OUT_BL] := ThroOut + ((+PitchOut + RollOut + YawOut) * ThroMix) ~> 7
    Motor[OUT_BR] := ThroOut + ((+PitchOut - RollOut - YawOut) * ThroMix) ~> 7


    'The low-throttle clamp prevents combined PID output from sending the ESCs below a minimum value
      'the ESCs appear to stall (go into "stop" mode) if the throttle gets too close to zero, even for a moment

     
    Motor[0] := 8500 #> Motor[0] <# 16000
    Motor[1] := 8500 #> Motor[1] <# 16000
    Motor[2] := 8500 #> Motor[2] <# 16000
    Motor[3] := 8500 #> Motor[3] <# 16000
     
    'Copy new Ouput array into servo values
    ESC.Set( MOTOR_FL, Motor[0] )
    ESC.Set( MOTOR_FR, Motor[1] )
    ESC.Set( MOTOR_BR, Motor[2] )
    ESC.Set( MOTOR_BL, Motor[3] )


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


PUB InitTestPID

  TestPID.Init(  4550, 475,  8400 )               
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
    Motor[OUT_FL] := ThroOut + ((-PitchOut + RollOut + YawOut) * ThroMix) ~> 7                          
    Motor[OUT_FR] := ThroOut + ((-PitchOut - RollOut - YawOut) * ThroMix) ~> 7  
    Motor[OUT_BL] := ThroOut + ((+PitchOut + RollOut - YawOut) * ThroMix) ~> 7
    Motor[OUT_BR] := ThroOut + ((+PitchOut - RollOut + YawOut) * ThroMix) ~> 7

     
    Motor[0] := 8000 #> Motor[0] <# 16000
    Motor[1] := 8000 #> Motor[1] <# 16000
    Motor[2] := 8000 #> Motor[2] <# 16000
    Motor[3] := 8000 #> Motor[3] <# 16000
     
    'Copy new Ouput array into servo values
    ESC.Set( MOTOR_FL, Motor[0] )
    ESC.Set( MOTOR_FR, Motor[1] )
    ESC.Set( MOTOR_BR, Motor[2] )
    ESC.Set( MOTOR_BL, Motor[3] )




PUB ArmFlightMode

  FlightEnabled := TRUE
  EnableStep := 0
  Beep2
   
  All_LED( LED_Red & LED_Half )
  FindGyroZero
   
  All_LED( LED_Blue & LED_Half )        
  BeepTune

  DesiredAltitude := Alt
  
  loopTimer := cnt


PUB DisarmFlightMode

  ESC.Set( MOTOR_FL, 8000 )
  ESC.Set( MOTOR_FR, 8000 )
  ESC.Set( MOTOR_BR, 8000 )
  ESC.Set( MOTOR_BL, 8000 )

  FlightEnabled := FALSE
  EnableStep := 0
  Beep3
  All_LED( LED_Green & LED_Half )        
  loopTimer := cnt
   


PUB CheckDebugMode | c, gsx, gsy, gsz, gox, goy, goz, aox, aoy, aoz

  c := dbg.rxcheck
  if( c < 0 )
    return

  if( c =< MODE_VibrationTest )
    Mode := c
    return

   
  if( (c & $F8) == 8 )  'Nudge one of the motors
    if( Mode == MODE_MotorTest )
      NudgeMotor := c & 7  'Nudge the motor selected by the configuration app

  if( (c & $F8) == $10 )  'Zero, Reset, or set gyro calibration
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
        Sens.SetDriftValues( ~~gsx, ~~gsy, ~~gsz, ~~gox, ~~goy, ~~goz )         'Sign extend all the values from 16 bit to 32 bit
        loopTimer := cnt                                                        'Reset the loop counter or we'll be waiting forever 

      elseif( c == $14 )
        'Temporarily zero accel offset settings
        Sens.TempZeroAccelOffsetValues

      elseif( c == $15 )
        'Reset accel offset settings
        Sens.ResetAccelOffsetValues

      elseif( c == $16 )
        'Write new gyro drift settings - followed by 6 WORD values
        aox := (dbg.rx << 8) | dbg.rx
        aoy := (dbg.rx << 8) | dbg.rx
        aoz := (dbg.rx << 8) | dbg.rx
        Sens.SetAccelOffsetValues( ~~aox, ~~aoy, ~~aoz)                         'Sign extend all the values from 16 bit to 32 bit
        loopTimer := cnt                                                        'Reset the loop counter or we'll be waiting forever 


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
    
      dbg.txBulk( @Alt, 8 )       'Send 8 bytes of data from @Alt onward (2 longs worth of data, Alt and AltTemp)
       

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
   
  ctr := cnt
    
  repeat i from 0 to loop
    OUTA[BUZZER_1] := 1    
    OUTA[BUZZER_2] := 0

    ctr += d
    waitcnt( ctr )    

    OUTA[BUZZER_1] := 0    
    OUTA[BUZZER_2] := 1

    ctr += d
    waitcnt( ctr )    

  OUTA[BUZZER_1] := 0    
  OUTA[BUZZER_2] := 0
   


PUB BeepTune

  BeepHz( 1174, 150 )           'D5
  BeepHz( 1318, 150 )           'E5
  BeepHz( 1046, 150 )           'C5
  BeepHz(  522, 150 )           'C4
  BeepHz(  784, 300 )           'G4



PUB Beep
  BeepHz( 4500 , 80 )


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
  
          