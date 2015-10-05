''
'' Settings - settings and user prefs storage for Elev8-FC
''


CON
  DriftScalePref  =  0          '3 longs
  DriftOffsetPref =  3          '3 longs 

  AccelOffsetPref =  6          '3 longs
  MagScaleOfsPref =  9          '6 longs

  UseSBUSPref     = 15          '1 long                       
  SBUSCenterPref  = 16          '1 long
  UsePingPref     = 17          '1 long 

  RollCorrectPref = 18          '2 longs (Sin,Cos)
  PitchCorrectPref= 20          '2 longs (Sin.Cos)

  PrefLen         = 22



OBJ
  eeprom : "Propeller Eeprom.spin"
  'Dbg:   "FullDuplexSerial-32.spin"                     '1 cog (32-byte buffers, instead of 16)


PUB Main | testCheck
  ''This function exists only to test and validate the Load / Save / Checksum code
   
  {
  Dbg.Start( 31, 30, 0, 115200 )
  Dbg.rx

  eeprom.ToRam(@PrefStorage, @PrefStorage + constant(PrefLen*4 + 3), 32768 )    'Copy from EEPROM to DAT, address 32768

  testCheck := CalculateChecksum
  dbg.tx(0)
  dbg.hex( Checksum, 8 )
  dbg.tx(32)
  dbg.hex( testCheck, 8 )
  dbg.tx(13)


  SetDefaults   
  testCheck := CalculateChecksum
  dbg.hex( testCheck, 8 )
  dbg.tx(13)

  Save

  eeprom.ToRam(@PrefStorage, @PrefStorage + constant(PrefLen*4 + 3), 32768 )    'Copy from EEPROM to DAT, address 32768


  testCheck := CalculateChecksum
  dbg.tx(0)
  dbg.hex( Checksum, 8 )
  dbg.tx(32)
  dbg.hex( testCheck, 8 )
  dbg.tx(13)
  }


PUB Load | testChecksum 
  eeprom.ToRam(@PrefStorage, @PrefStorage + constant(PrefLen*4 + 3), 32768 )    'Copy from EEPROM to DAT, address 32768

  testChecksum := CalculateChecksum
  if( testChecksum <> Checksum )
    SetDefaults
    Save  


PUB Save

  Checksum := CalculateChecksum
  eeprom.FromRam(@PrefStorage, @PrefStorage + constant(PrefLen*4 + 3), 32768 )  'Copy from DAT to EEPROM, address 32768



PUB SetDefaults
  longfill( @PrefStorage, 0, PrefLen )
  SBUSCenter := 1000 
  long[@RollCorrect][0] := 0.0                          'Sin of roll correction angle
  long[@RollCorrect][1] := 1.0                          'Cos of roll correction angle

  long[@PitchCorrect][0] := 0.0                         'Sin of pitch correction angle 
  long[@PitchCorrect][1] := 1.0                         'Cos of pitch correction angle 



PUB GetValue( index )
  return long[@PrefStorage][index]


PUB SetValue( index , val )
  long[@PrefStorage][index] := val


PUB GetAddress( index )
  return @PrefStorage + index*4



PUB CalculateChecksum | i, r

  r := $55555555           'Start with a strange, known value
  repeat i from 0 to constant(PrefLen-1)
    r := (r <-= 7)
    r := r ^ long[@PrefStorage][i]     'Jumble the bits, XOR in the prefs value

  return r


DAT
PrefStorage        
DriftScale              long    0, 0, 0
DriftOffset             long    0, 0, 0 
AccelOffset             long    0, 0, 0
MagScaleOfs             long    0, 0, 0, 0, 0, 0
 
UseSBUS                 long    0                       
SBUSCenter              long    1000
UsePing                 long    0

RollCorrect             long    0, 1.0                  'Sin,Cos of roll correction angle
PitchCorrect            long    0, 1.0                  'Sin,Cos of pitch correction angle 

Checksum                long    0
 