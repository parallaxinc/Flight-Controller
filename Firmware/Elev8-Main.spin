''
''  Elev8 Flight Controller
''  Main flight core
''

CON
  _clkmode = xtal1 + pll16x
  '_xinfreq = 5_000_000
  _clkfreq = 80_000_000

  RC_THRO = 2
  RC_AILE = 0
  RC_ELEV = 4
  RC_RUDD = 5
  RC_AUX1 = 1
  RC_AUX2 = 3
  RC_AUX3 = 26
  RC_AUX4 = 27

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


  CS_ALT = 9    '8
  CS_AG  = 11   '11
  SDO    = 13   '12
  SDI    = 14   '13
  SCL    = 12   '14
  CS_M   = 10   '15

  LED_PIN = 8
  LED_COUNT = 2
  LED_TESTDELAY = 6_000_000

  BUZZER_1 = 6
  BUZZER_2 = 7

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


  YawCircle = $2_0000
  YawMask = YawCircle - 1
  YawCircleHalf = YawCircle >> 1
  

VAR
  long  Thro, Aile, Elev, Rudd, Aux1, Aux2, Aux3, Aux4, iRudd
  long  Temperature, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, MagX, MagY, MagZ, Alt, AltTemp
  long  Mode, counter, NudgeMotor

  long  Pitch, Roll, Yaw
  long  DesiredRoll, DesiredPitch, DesiredYaw
  long  PitchOut, RollOut, YawOut                       'Outputs from the PID loops
  long  ThroMix                           

  long  Motor[4]                                        'Motor output values  

  long  LEDValue[LED_COUNT]
  byte  Quat[16]
  
  long loopTimer
  long SensorTime

  WORD TxData[10]
  
  byte FlightEnabled, EnableStep
  byte DoIntegrate
  byte MotorIndex[4]
  

OBJ
  RC :   "RC_Receiver.spin"                             '1 cog
  Sens : "Sensors.spin"                                 '1 cog
  IMU :  "QuatIMU.spin"                                 '1 cogs (float command stream processor) 
  ESC :  "Servo32-HighRes.spin"                         '1 cog
  
  Dbg:   "FullDuplexSerial-32.spin"                     '1 cog (32-byte buffers, instead of 16)

  RollPID   : "IntPID.spin"
  PitchPID  : "IntPID.spin"
  YawPID    : "IntPID.spin"

  Freq      : "Synth.spin"


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


  'Configure and start all the objects
  Dbg.Start( 31, 30, 0, 115200 )
  RC.Start

  All_LED( LED_Red & LED_Eighth )             'LED red on startup
    
  Sens.Start(SDI, SDO, SCL, CS_AG, CS_M, CS_ALT, LED_PIN, @LEDValue, LED_COUNT)  
  IMU.Start

  'Dbg.rx
  'Dbg.dec( IMU.GetQuatUpdateLen )
  'Dbg.tx(13)
  'Dbg.dec( IMU.GetCalcErrorLen )
  'dbg.tx( 13 )
  'dbg.rx
  


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


  RollPID.Init (    1500,   0,  -12000 )               
  PitchPID.Init(    1500,   0,  -12000 )     
  YawPID.Init  (    3000,   0,   -8000 ) 

  RollPID.SetPrecision( 13 ) 
  PitchPID.SetPrecision( 13 ) 
  YawPID.SetPrecision( 13 ) 

  RollPID.SetMaxOutput( 300*8 )
  PitchPID.SetMaxOutput( 300*8 )

  RollPID.SetPIMax( 20*8 )
  PitchPID.SetPIMax( 20*8 )
  
  RollPID.SetMaxIntegral( 40000*8 )
  PitchPID.SetMaxIntegral( 40000*8 )

  YawPID.SetMaxOutput( 300 )


  FindGyroZero     'Get a gyro baseline - We'll re-do this on flight arming, but this helps settle the IMU
  

  All_LED( LED_Green & LED_Eighth )


  counter := 0
  loopTimer := cnt                                      'Timekeeping value - tracks the next 400th/sec interval for IMU accuracy


  repeat
    Cycles := cnt

    'Read ALL inputs from the sensors into local memory, starting at Temperature
    longmove( @Temperature, Sens.Address, 12 ) 

    SensorTime := Sens.In(12) 

    IMU.Update_Part1( @GyroX )        'Entire IMU takes ~92000 cycles


    Thro :=  RC.GetRC( RC_THRO )
    Aile := -RC.GetRC( RC_AILE )
    Elev :=  RC.GetRC( RC_ELEV )
    Rudd := -RC.GetRC( RC_RUDD )
    Aux1 :=  RC.GetRC( RC_AUX1 )
    Aux2 := -RC.GetRC( RC_AUX2 )
    Aux3 :=  RC.GetRC( RC_AUX3 )
    Aux4 := -RC.GetRC( RC_AUX4 )

    'Small dead-band around zero
    'if( ||Aile < 4 )
    '  Aile := 0

    'if( ||Elev < 4 )
    '  Elev := 0



    Pitch := IMU.GetPitch
    Roll  := IMU.GetRoll
    Yaw   := IMU.GetYaw


    IMU.Update_Part2

    UpdateFlightMode            '~82000 cycles when in flight mode
    
    IMU.WaitForCompletion

    CheckDebugMode
    DoDebugModeOutput

    Cycles := cnt - Cycles
    'dbg.tx(1)
    'dbg.dec( Cycles )
    'dbg.tx(13)



    ++counter
    loopTimer += constant(_clkfreq / 320)
    waitcnt( loopTimer )



PUB FindGyroZero | i

  waitcnt( cnt + constant(_clkfreq / 4) )
  repeat i from 0 to 255
    GyroX += Sens.In(1)
    GyroY += Sens.In(2)
    GyroZ += Sens.In(3)
    AccelX += Sens.In(4)
    AccelY += Sens.In(5)
    AccelZ += Sens.In(6)

    waitcnt( cnt + constant(_clkfreq / 2000) )      

  GyroX /= 256  
  GyroY /= 256  
  GyroZ /= 256  
  AccelX /= 256  
  AccelY /= 256  
  AccelZ /= 256
  IMU.SetGyroZero( GyroX, GyroY, GyroZ )   



PUB UpdateFlightMode | ThroOut


  if( FlightEnabled == FALSE )

      'Test for power up sequence----------------------------------------------
      'Left stick positioned in lower left?
    if( EnableStep == 0)
      if( Rudd < -760  AND  Thro < -760 )
        EnableStep := 1
        Beep
        All_LED( LED_Cyan & LED_Eighth )        
        loopTimer := cnt
         

    'Left stick positioned in lower right?
    elseif( EnableStep == 1)
      if( Rudd >  760  AND  Thro < -760 )
        FlightEnabled := TRUE
        EnableStep := 0
        Beep2
       
        All_LED( LED_Red & LED_Eighth )
        FindGyroZero
       
        All_LED( LED_Blue & LED_Eighth )        
        BeepTune
        loopTimer := cnt
      '------------------------------------------------------------------------
     
      return

  else  'FlightEnabled == TRUE

     
    'Test for power down sequence--------------------------------------------
    'Left stick positioned in lower left?
    if( EnableStep == 0)
      if( Rudd < -760  AND  Thro < -760 )
        EnableStep := 1
        Beep
        All_LED( LED_Yellow & LED_Eighth )        
        loopTimer := cnt
         
     
    'Left stick positioned in lower left?
    elseif( EnableStep == 1 )
      if( Rudd >  760  AND  Thro < -760 )
        FlightEnabled := FALSE
        EnableStep := 0
        Beep3
        All_LED( LED_Green & LED_Eighth )        
        loopTimer := cnt
    '------------------------------------------------------------------------
     
     
    'Rudder is integrated (accumulated)
    iRudd += Rudd


    'Angular output from the IMU is +/- 65536 units, or 32768 = 90 degrees
    'Input range from the controls is +/- 1000 units.  Scale that up to about 45 degrees       

    DesiredRoll :=   Aile << 4
    DesiredPitch :=  Elev << 4


    'Yaw is different because we accumulate it - It's not specified absolutely like you can
    'with pitch and roll, so scale the stick input down a bit
    DesiredYaw :=   iRudd >> 3
     
     
    'Zero yaw target when throttle is off - makes for more stable liftoff
     
    if( Thro < -760 )
      DoIntegrate := FALSE
      DesiredYaw := Yaw      'Desired = measured when the throttle is off
      iRudd := Yaw << 3
    else
      DoIntegrate := TRUE
     
     
    DesiredYaw &= YawMask
     
    if( ||(DesiredYaw - Yaw) => YawCircleHalf )
      if( Yaw < YawCircleHalf )
        DesiredYaw := DesiredYaw - YawCircle
      else
        DesiredYaw := DesiredYaw + YawCircle      

     
    RollOut := RollPID.Calculate_ForceD_NoD2( DesiredRoll , Roll , GyroY , DoIntegrate )    
    PitchOut := PitchPID.Calculate_ForceD_NoD2( DesiredPitch , Pitch , GyroX , DoIntegrate )    
    YawOut := YawPID.Calculate_ForceD_NoD2( DesiredYaw , Yaw , -GyroZ, DoIntegrate )    

     
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



PUB CheckDebugMode | c, gsx, gsy, gsz, gox, goy, goz

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

  if( c == $ff )
    dbg.tx($E8)  'Simple ping-back to tell the application we have the right comm port



PUB DoDebugModeOutput | loop, addr, phase, ledcoloridx, ledbrightidx
  if( Mode == MODE_None )
    return

  phase := counter & 3


  if( Mode == MODE_RadioTest )
    if( phase == 0 )
      dbg.txFast( $77 )      
      dbg.txFast( $77 )      
      dbg.txFast( MODE_RadioTest )

      dbg.txFast( Thro )            'Normally I'd use ( Thro & 255, but the serial object only sends the low 8 bits, so not necessary)      
      dbg.txFast( Thro >> 8 )      
      dbg.txFast( Aile )      
      dbg.txFast( Aile >> 8 )      
      dbg.txFast( Elev )      
      dbg.txFast( Elev >> 8 )      
      dbg.txFast( Rudd )      
      dbg.txFast( Rudd >> 8 )      

    elseif( phase == 2 )
      dbg.txFast( Aux1 )      
      dbg.txFast( Aux1 >> 8 )      
      dbg.txFast( Aux2 )      
      dbg.txFast( Aux2 >> 8 )      
      dbg.txFast( Aux3 )      
      dbg.txFast( Aux3 >> 8 )      
      dbg.txFast( Aux4 )      
      dbg.txFast( Aux4 >> 8 )      


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
        ESC.Set(MotorIndex[NudgeMotor], 1200*8)
         
        'waitcnt( cnt + 10_000_000 )
        'ESC.Set(NudgeMotor, 1000)
        
      elseif( NudgeMotor == 4 )
        BeepHz(2000, 50)
        waitcnt( cnt + 5_000_000 )
        BeepHz(3000, 50)
        
      elseif( NudgeMotor == 5 )
        'repeat loop from 0 to $f0_f0_f0 step $04_04_04 
          'All_LED( loop )
           'waitcnt( cnt + 500_000 )
        'repeat loop from $f0_f0_f0 to 0 step $04_04_04 
          'All_LED( loop )
          'waitcnt( cnt + 500_000 )
        
        ' RGB led will fade-up/down through 3 colors: red, green, blue  
        repeat ledcoloridx from 0 to 2
            repeat ledbrightidx from 0 to 8

                All_LED( LEDTestSeqColor[ledcoloridx] & LEDTestSeqBright[ledbrightidx] )
                waitcnt( cnt + LED_TESTDELAY ) 

        All_LED( 0 )

      elseif( NudgeMotor == 6 )

        BeepHz(3000, 100)
        waitcnt( cnt + 5_000_000 )
        BeepHz(3000, 100)
        waitcnt( cnt + 5_000_000 )
        BeepHz(3000, 100)
        waitcnt( cnt + 5_000_000 )
        BeepHz(3000, 100)

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

      elseif( NudgeMotor == 7 )
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

  elseif( Mode == MODE_VibrationTest )
    dbg.txFast( $77 )      
    dbg.txFast( $77 )      
    dbg.txFast( Mode )
     
    TxData[0] := GyroX
    TxData[1] := GyroY
    TxData[2] := GyroZ
     
    dbg.txBulk( @TxData, 6 )   'Send 20 bytes of data from @TxData onward (sends 10 words worth of data)                         
      

{
PUB Beep(delay, count) | i
  repeat i from 0 to count
    OUTA[16]~~
    waitcnt( CNT + delay )
    OUTA[16]~
    waitcnt( CNT + delay )
}


PUB All_LED( Color )
  LongFill( @LEDValue[0], Color, LED_COUNT) 


PUB BeepHz( Hz , Delay ) | i, loop, d, ctr

  'Note that each loop does a high and low cycle, so we use a baseline of 40_000_000 cycles instead of 80_000_000 (1/2)

  d := 40_000_000 / Hz          'Compute the amount of time to delay between pulses to get the right frequency
  loop := (Delay * 40_000) / d  'How many iterations of the loop to make "Delay" milliseconds?

  DIRA[BUZZER_1] := 1    
  DIRA[BUZZER_2] := 1
   
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
   


PUB BeepHz2( Hz , Delay ) | i

  DIRA[BUZZER_1] := 1                                   'Enable the buzzer pin output
  Freq.Synth("A", BUZZER_1, Hz )
  waitcnt( Delay*constant(_clkfreq/1000) + cnt )
  Freq.Synth( "A", BUZZER_1, 0 )
  OUTA[BUZZER_1] := 0        
  DIRA[BUZZER_1] := 0                                   'Disable the buzzer pin output


PUB BeepTune

  BeepHz( 1174, 150 )           'D5
  BeepHz( 1318, 150 )           'E5
  BeepHz( 1046, 150 )           'C5
  BeepHz(  522, 150 )           'C4
  BeepHz(  784, 300 )           'G4



PUB Beep
  BeepHz( 3000 , 80 )


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

  LEDTestSeqColor   long    LED_Red
                    long    LED_Green
                    long    LED_Blue
                    
  LEDTestSeqBright  long    LED_Dim
                    long    LED_Eighth
                    long    LED_Quarter
                    long    LED_Half
                    long    LED_Full
                    long    LED_Half
                    long    LED_Quarter
                    long    LED_Eighth
                    long    LED_Dim
