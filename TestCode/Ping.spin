CON

  TO_IN = 73_746                                                                ' Inches
  TO_CM = 29_034                                                                ' Centimeters
                                                                                 

PUB Fire(Pin)
''Return Ping)))'s one-way ultrasonic travel time in microseconds

  FRQA := 1
  PHSA := 0
                                                                                 
  outa[Pin]~                                                                    ' Clear I/O Pin
  dira[Pin]~~                                                                   ' Make Pin Output
  outa[Pin]~~                                                                   ' Set I/O Pin
  outa[Pin]~                                                                    ' Clear I/O Pin (> 2 µs pulse)
  dira[Pin]~                                                                    ' Make I/O Pin Input

  'Measure the length of the high signal on the PING pin
  CTRA := (%01000 << 26) | (%111 << 23) | Pin


PUB Ticks(pin) : Microseconds | len
  'waitpne(0, |< Pin, 0)                                                         ' Wait For Pin To Go HIGH
  'cnt1 := cnt                                                                   ' Store Current Counter Value
  'waitpeq(0, |< Pin, 0)                                                         ' Wait For Pin To Go LOW 
  'cnt2 := cnt                                                                   ' Store New Counter Value
  len := PHSA
  Microseconds := (len / (clkfreq / 1_000_000)) >> 1                             ' Return Time in µs
  

PUB Inches(Pin) : Distance
''Measure object distance in inches

  Distance := Ticks(Pin) * 1_000 / TO_IN                                        ' Distance In Inches
                                                                                 
                                                                                 
PUB Centimeters(Pin) : Distance                                                  
''Measure object distance in centimeters
                                              
  Distance := Millimeters(Pin) / 10                                             ' Distance In Centimeters
                                                                                 
                                                                                 
PUB Millimeters(Pin) : Distance                                                  
''Measure object distance in millimeters
                                              
  Distance := Ticks(Pin) * 10_000 / TO_CM                                       ' Distance In Millimeters
