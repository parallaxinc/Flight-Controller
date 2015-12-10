{{
* RC_driver_ppm.spin *

  This file is part of the ELEV-8 Flight Controller Firmware
  for Parallax part #80204, Revision A
  
  Copyright 2015 Parallax Incorporated

  ELEV-8 Flight Controller Firmware is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  ELEV-8 Flight Controller Firmware is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the ELEV-8 Flight Controller Firmware.  If not, see <http://www.gnu.org/licenses/>.
  
  Written by Jason Dorie
  
-----------------------------------------------------------------------------------------------
Read servo control pulses from a generic R/C receiver
Use 4.7K resistors (or 4050 buffers) between each Propeller input and receiver signal output.

 Note: +5 and GND on all the receiver channels are usally interconnected,
 so to power the receiver only one channel need to be connected to +5V and GND.
-----------------------------------------------------------------------------------------------
}}

Con
  Scale = 80/2 ' System clock frequency in Mhz, halved - we're converting outputs to 1/2 microsecond resolution
   
VAR
  long  Cog
  long  Pins[8]
  long  PinMask                                          


PUB start : status

  'Input pin is P25  (receiver input 0)
  PinMask := %00000010_00000000_00000000_00000000       'Elev8-FC pins are non-contiguous 

'' Start driver (1 Cog)  
'' - Note: Call setpins() before start
  if not Cog
    repeat status from 0 to 7
      Pins[status] := Scale * 3000                      ' Center Pins[0..7]
    Pins[0] := Scale * 2000                             ' Zero throttle pin
    status := Cog := cognew(@INIT, @Pins) + 1

PUB stop
'' Stop driver and release cog
  if Cog
    cogstop(Cog~ - 1)


PUB get(_pin) : value
'' Get receiver servo pulse width in us. 
  value := Pins[_pin] / Scale                           ' Get pulse width from Pins[..] , convert to uSec


PUB getrc(_pin) : value
'' Get receiver servo pulse width as normal r/c values (+/- 1000) 
  value := Pins[_pin] / Scale - 3000                   ' Get puls width from Pins[..], convert to uSec, make 0 center


PUB Channel( _pin )
  return Pins[_pin] / Scale   


DAT
        org   0

INIT    mov   p1, par                           ' Get data pointer
        add   p1, #4*8                          ' Point to PinMask
        rdlong pin_mask, p1                     ' Read PinMask
        andn  dira, pin_mask                    ' Set input pins
        mov   pin_index, #0

'=================================================================================
        'Record an initial time value
        mov   start_time, cnt

:loop
        waitpeq pin_mask, pin_mask              ' Wait for the pin to go high                                     
        waitpne pin_mask, pin_mask              ' Wait for the pin to go low
        
        mov   end_time, cnt                     ' Record the current time

        mov   elapsed, end_time                 ' Compute elapsed time
        sub   elapsed, start_time

        cmp   elapsed, sync_thresh  wc          ' Is the elapsed time less than a frame pause? 

  if_nc jmp   #:resync          'elapsed >= frame threshold ? - jump to resync because we're back to pin 0

  
        cmp   pin_index, #32 wc 'Make sure that we're writing a valid index (receiver might support > 8 channels)
  if_nc jmp   #:endLoop

        mov   p1, par           'load the address of the Pins[] array
        add   p1, pin_index     '...offset by the current pin index (in longs) 
        wrlong elapsed, p1      'write the elapsed time for this pin into the array

        add   pin_index, #4                     ' increment pin_index (destination offset) by one long address
        jmp   #:endLoop                         ' move on to the next pin        

:resync 'elapsed >= threshold
        mov   pin_index, #0     ' We hit a gap in pins - reset the pin index                

:endLoop
        mov   start_time, end_time              ' copy current time to previous time        
        jmp   #:loop                            ' do it all over again        




'=================================================================================

pin_mask      long      %00000010_00000000_00000000_00000000         'Pin 25
pin_index     long      0     
sync_thresh   long      (80_000_000 / 1000) * 3         '3ms sync pulse threshold                                   

start_time    res       1
end_time      res       1
elapsed       res       1
p1            res       1


        FIT   496
{{
********************************************************************************************************************************
*                                                   TERMS OF USE: MIT License                                                  *                                                            
********************************************************************************************************************************
*Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    * 
*files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    *
*modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software*
*is furnished to do so, subject to the following conditions:                                                                   *
*                                                                                                                              *
*The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.*
*                                                                                                                              *
*THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          *
*WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         *
*COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   *
*ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         *
********************************************************************************************************************************
}}