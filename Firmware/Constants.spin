
''Constants
''
''Object used to specify cross-module constants, such as pin assigments, update rates, etc


CON
  _clkmode = xtal1 + pll16x
  '_xinfreq = 5_000_000
  _clkfreq = 80_000_000

  UpdateRate = 250
  UpdateCycles = _clkfreq / UpdateRate

 
PUB Placeholder
  'This is simply to allow this object to compile


{
Other notes:

Transmit rates @ 115,200 baud:

  46 bytes per update @ 250 Hz
  36 bytes per update @ 320 Hz  - These are maximum, optimal rates, probably not possible
  28 bytes per update @ 400 Hz
}
