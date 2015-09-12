
''Constants
''
''Object used to specify cross-module constants, such as pin assigments, update rates, etc


CON
  _clkmode = xtal1 + pll16x
  '_xinfreq = 5_000_000
  _clkfreq = 80_000_000

  UpdateRate = 250
  UpdateCycles = _clkfreq / UpdateRate


  DriftScalePref =  32768 + 0
  DriftOffsetPref = 32768 + 3*4
  AccelOffsetPref = 32768 + 6*4
  UseSBUSPref     = 32768 + 9*4 
  SBUSCenterPref  = 32768 + 10*4 

  NextPref        = 32768 + 11*4
 
PUB Placeholder
  'This is simply to allow this object to compile


{
Other notes:

Transmit rates @ 115,200 baud:

  46 bytes per update @ 250 Hz
  36 bytes per update @ 320 Hz  - These are maximum, optimal rates, probably not possible
  28 bytes per update @ 400 Hz
}