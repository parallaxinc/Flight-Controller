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


VAR
  long  counter, loopTimer

  

OBJ
  ESC :  "Servo32-HighRes.spin"                      '1 cog
  Dbg:   "FullDuplexSerial.spin"                     '1 cog (32-byte buffers, instead of 16)

  
PUB Main | Cycles, i, pinsAddr, delaysAddr, pulse

  Dbg.Start( 31, 30, 0, 115200 )

  ESC.Init( 400 )               'ESC update rate for fast pins (slow pins are always 50hz)

  'First, set up which pins are fast, and which are slow

  repeat i from 0 to 29
    if( i == 16 OR i == 17 )
      ESC.AddSlowPin(i)
    else
      ESC.AddFastPin(i)

  'Now set the values you want intially - I recommend zeroing, but for the demo this is just random numbers

  ESC.Set( 0, 16010)
  ESC.Set( 1, 16015)
  ESC.Set( 2, 16020)
  ESC.Set( 3, 16025)
  ESC.Set( 4, 16030)
  ESC.Set( 5, 16035)
  ESC.Set( 6, 16040)
  ESC.Set( 7, 16045)

  ESC.Set( 8, 11000)
  ESC.Set( 9, 15000)
  ESC.Set(10, 15500)
  ESC.Set(11, 14520)
  ESC.Set(12, 13285)
  ESC.Set(13, 12364)
  ESC.Set(14, 15235)
  ESC.Set(15, 15347)

  ESC.Set(16,  4000)
  ESC.Set(17,  8000)
  ESC.Set(18, 12000)
  ESC.Set(19, 12001)
  ESC.Set(20, 12002)
  ESC.Set(21, 12003)
  ESC.Set(22, 12004)
  ESC.Set(23, 12005)

  ESC.Set(24, 12000)
  ESC.Set(25, 12120)
  ESC.Set(26, 12125)
  ESC.Set(27, 12130)
  ESC.Set(28, 12135)
  ESC.Set(29, 12140)
  'ESC.Set(30, 12145)
  'ESC.Set(31, 12150)

  
  ESC.Start

  Pulse := 4000

  repeat

    ESC.Set( 16, Pulse )
    ESC.Set( 23, Pulse )

    Pulse := Pulse + 200        'Dynamically change one of the servo outputs
    if( Pulse => 16_000 )
      Pulse := 4000

    DBG.tx( 1 )
    dbg.dec( ESC.GetCycles )
    dbg.tx( 11 )
          
    waitcnt( cnt + 1_000_000 )