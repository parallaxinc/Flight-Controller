{{RC_Receiver.spin
-----------------------------------------------------------------------------------------------
Read servo control pulses from a generic R/C receiver
Use 4.7K resistors (or 4050 buffers) between each Propeller input and receiver signal output.

 Note: +5 and GND on all the receiver channels are usally interconnected,
 so to power the receiver only one channel need to be connected to +5V and GND.
-----------------------------------------------------------------------------------------------

The getrc function was modified to return values centered at zero instead of 3000

NOTE that the pins are accessed by index, not by pin number.  If you set the code to
read the four pins %011001100, the lowest pin index (P2) will be entry 0 in the array,
and will be returned when you call Get( 0 ) or GetRC( 0 ).  Pins are returned from lowest
to highest.

}}
Con
  Scale = 80/2 ' System clock frequency in Mhz, halved - we're converting outputs to 1/2 microsecond resolution
   
VAR
  long  Cog
  long  Pins[8]
  long  PinMask                                          


PUB start : status

  'Input pins are P0,1,2,3,4,5,26,27                       
  PinMask := %00001100_00000000_00000000_00111111       'Elev8-FC pins are non-contiguous 

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
'' Get receiver servo pulse width in µs. 
  value := Pins[_pin] / Scale                           ' Get pulse width from Pins[..] , convert to uSec


PUB getrc(_pin) : value
'' Get receiver servo pulse width as normal r/c values (±1000) 
  value := Pins[_pin] / Scale - 3000                   ' Get puls width from Pins[..], convert to uSec, make 0 center


PUB Channel( _pin )
  return Pins[_pin] / Scale   


DAT
        org   0

INIT    mov   p1, par                           ' Get data pointer
        add   p1, #4*8                          ' Point to PinMask
        rdlong pin_mask, p1                     ' Read PinMask
        andn  dira, pin_mask                    ' Set input pins

'=================================================================================

:loop   mov   d2, d1                            ' Store previous pin status
        waitpne d1, pin_mask                    ' Wait for change on relevant pins
        mov   d1, ina                           ' Get new pin status 
        mov   c1, cnt                           ' Store change cnt                           
        and   d1, pin_mask                      ' Remove non-relevant pin changes
{
d2      1100
d1      1010
-------------
!d2     0011
&d1     1010
=       0010 POS edge

d2      1100
&!d1    0101
=       0100 NEG edge     
}
        ' Mask for POS edge changes
        mov   d3, d1
        andn  d3, d2

        ' Mask for NEG edge changes
        andn  d2, d1

'=================================================================================

:POS    tjz  d3, #:NEG                          ' Skip if no POS edge changes
'Pin 0
        test  d3, pin_mask_0    wz              ' Change on pin?
if_nz   neg   pe0, c1                           ' Store riding edge change cnt, but as a negative number, so we can "subtract it" from the falling edge time later
'Pin 1
        test  d3, pin_mask_1    wz              ' ...
if_nz   neg   pe1, c1
'Pin 2
        test  d3, pin_mask_2    wz
if_nz   neg   pe2, c1
'Pin 3
        test  d3, pin_mask_3    wz
if_nz   neg   pe3, c1
'Pin 4
        test  d3, pin_mask_4    wz
if_nz   neg   pe4, c1
'Pin 5
        test  d3, pin_mask_5    wz
if_nz   neg   pe5, c1
'Pin 6
        test  d3, pin_mask_6    wz
if_nz   neg   pe6, c1
'Pin 7
        test  d3, pin_mask_7    wz
if_nz   neg   pe7, c1


'=================================================================================

:NEG    tjz   d2, #:loop                        ' Skip if no NEG edge changes
'Pin 0
        mov   p1, par                           ' Get data pointer
        test  d2, pin_mask_0    wz              ' Change on pin 0?
if_nz   add   pe0, c1                           ' Get pulse width : NewTime - OldTime  is the same as  NewTime + (-OldTime)
if_nz   wrlong pe0, p1                          ' Store pulse width to HUB location
'Pin 1
        add   p1, #4                            ' Get next data pointer
        test  d2, pin_mask_1    wz              ' ...
if_nz   add   pe1, c1
if_nz   wrlong pe1, p1
'Pin 2
        add   p1, #4
        test  d2, pin_mask_2    wz
if_nz   add   pe2, c1
if_nz   wrlong pe2, p1             
'Pin 3
        add   p1, #4
        test  d2, pin_mask_3    wz
if_nz   add   pe3, c1
if_nz   wrlong pe3, p1             
'Pin 4
        add   p1, #4
        test  d2, pin_mask_4    wz
if_nz   add   pe4, c1
if_nz   wrlong pe4, p1             
'Pin 5
        add   p1, #4
        test  d2, pin_mask_5    wz
if_nz   add   pe5, c1
if_nz   wrlong pe5, p1             
'Pin 6
        add   p1, #4
        test  d2, pin_mask_6    wz
if_nz   add   pe6, c1
if_nz   wrlong pe6, p1             
'Pin 7
        add   p1, #4
        test  d2, pin_mask_7    wz
if_nz   add   pe7, c1
if_nz   wrlong pe7, p1             

        jmp   #:loop



'=================================================================================

pin_mask long 0

c1      long  0
               
d1      long  0
d2      long  0
d3      long  0
d4      long  0

p1      long  0

pe0     long  0
pe1     long  0
pe2     long  0
pe3     long  0
pe4     long  0
pe5     long  0
pe6     long  0
pe7     long  0


pin_mask_0    long      %00000100_00000000_00000000_00000000         'Pin 26
pin_mask_1    long      %00001000_00000000_00000000_00000000         'Pin 27 
pin_mask_2    long      %00000000_00000000_00000000_00000001         'Pin 0
pin_mask_3    long      %00000000_00000000_00000000_00000010         'Pin 1
pin_mask_4    long      %00000000_00000000_00000000_00000100         'Pin 2
pin_mask_5    long      %00000000_00000000_00000000_00001000         'Pin 3
pin_mask_6    long      %00000000_00000000_00000000_00010000         'Pin 4
pin_mask_7    long      %00000000_00000000_00000000_00100000         'Pin 5


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
