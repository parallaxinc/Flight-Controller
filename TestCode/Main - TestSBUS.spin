''
''  Elev8 Flight Controller
''  Main flight core
''

CON
'HoverFlyGimbal, 10MHz
'  _CLKMODE      = XTAL1 + PLL8X                        
'  _XINFREQ      = 10_000_000     '10MHz crystal is used

'Parallax Protoboard, etc
  _CLKMODE      = XTAL1 + PLL16X                        
  _XINFREQ      = 5_000_000       '5MHz crystal is used

  MOTOR_FL = 23
  MOTOR_FR = 20
  MOTOR_BR = 22
  MOTOR_BL = 21


VAR
  long  counter, loopTimer
  

OBJ
  Dbg:   "FullDuplexSerial.spin"                     '1 cog (32-byte buffers, instead of 16)
  'SBUS:  "SBUS-Receiver.spin"                       '1 cog (32-byte buffers, instead of 16)

  RC :   "RC_Receiver-2.spin"
  ESC:   "Servo8Fast-Assignable.spin"

  
PUB Main | Cycles, i, pinsAddr, delaysAddr, pulse, Pins

  ESC.Init
  ESC.AddFastPin(MOTOR_FL)
  ESC.AddFastPin(MOTOR_FR)
  ESC.AddFastPin(MOTOR_BR)
  ESC.AddFastPin(MOTOR_BL)


  ESC.Set(0, 1000) 'Throttle ranges from 1000 to 2000 - 1000 is "off"
  ESC.Set(1, 1000)
  ESC.Set(2, 1000)
  ESC.Set(3, 1000)
  
  ESC.Start

  RC.Start

  Dbg.Start( 31, 30, 0, 115200 )
  'SBUS.Start( 17 )
  
  repeat

    dbg.tx(1)
    {
    repeat i from 0 to 17
      Dbg.dec( SBUS.Channel(i) )
      Dbg.tx( 11 )
      Dbg.tx( 13 )
    }

    repeat i from 0 to 7
        Dbg.dec( rc.get(i) )
        Dbg.tx( 11 )
        Dbg.tx( 13 )

    waitcnt( cnt + 1_000_000 )      