
CON
  _clkmode = xtal1 + pll16x
  '_xinfreq = 5_000_000
  _clkfreq = 80_000_000

  CS_ALT = 9
  CS_AG  = 11
  SDO    = 13
  SDI    = 14
  SCL    = 12
  CS_M   = 10
  LED_PIN = 8

  LED_COUNT = 1


OBJ
  RC : "RC_Receiver_driver"
  Servo32 : "Servo32_HighRes_driver"
  Sensors : "Sensors_driver"

  pid : "IntPID"
  Dbg:   "FullDuplexSerial"


VAR
  long LEDValue[1]
  long start, end


PUB main

  Dbg.Start( 31, 30, 0, 115200 )

  'Add startup code here.
  RC.Start

  Servo32.Init( 400 )
  Servo32.AddFastPin( 15 )
  Servo32.AddFastPin( 16 )
  Servo32.Set( 15, 8000 )
  Servo32.Set( 16, 8000 )
  
  Servo32.Start

  LEDValue[0] := $0F000F
  
  Sensors.Start( SDI, SDO, SCL, CS_AG, CS_M, CS_ALT, LED_PIN, @LEDValue[0], LED_COUNT )

  dbg.rx
  start := cnt
  
  pid.Init( 1000, 100000, 100000, 250 )
  pid.Calculate( 0, 10, true )
  
  end := cnt

  dbg.dec( end - start )
  dbg.rx
  


  repeat

    dbg.dec( Sensors.In(10) )
    dbg.tx(9) 
    dbg.dec( Sensors.In(12) )
    dbg.tx(9) 
    dbg.dec( Sensors.In(13) )
    dbg.tx(13)


    waitcnt( CNT + constant(80000000/100) )