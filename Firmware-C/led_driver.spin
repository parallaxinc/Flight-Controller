{{
* Bare WS2812 driver *

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
}}
             
'' Note that this code assumes an 80 MHz clock

' 1 / x = 

' 1_000 = ms
' 1_000_000 = us
' 1_000_000_000 = ns

'    20_000_000 = 50 ns        (one instruction, 4 cycles @ 80MHz)
'    80_000_000 = 80MHz cycle


CON
  _clkmode = xtal1 + pll16x
  _clkfreq = 80_000_000


  HUNDRED_nS  = _clkfreq / 10_000_000  'Number of clock cycles per 100 nanoseconds (8 @ 80MHz)                        
  ONE_uS      = HUNDRED_nS * 10 'Number of clock cycles per 1 microsecond (1000 nanoseconds)

' LED_RESET   = 50 * ONE_uS     'Too big to be a constant, so it's in a variable in the DAT section

'WS2812B Timings
  LED_0_HI    = (ONE_uS * 35)/100       
  LED_0_LO    = (ONE_uS * 90)/100       
  LED_1_HI    = (ONE_uS * 90)/100       
  LED_1_LO    = (ONE_uS * 35)/100       


'WS2812 Timings
'  LED_0_HI    = (ONE_uS * 35)/100       
'  LED_0_LO    = (ONE_uS * 80)/100       
'  LED_1_HI    = (ONE_uS * 70)/100       
'  LED_1_LO    = (ONE_uS * 60)/100       

VAR

  long  ins[3]         'LEDPin, LEDAddr, LEDCount
  long  cog

OBJ
  'const  : "Constants.spin"
  'settings : "Settings.spin"


PUB start(_LEDPin, _LEDAddr, _LEDCount) : okay

'' Start driver - starts a cog
'' returns false if no cog available
'' may be called again to change settings
''
''   LEDPin  = pin connected to WS2812B LED array
''   LEDAddr = HUB address of RGB values for LED array (updated constantly)
''   LEDCount= Number of LED values to update  

  return startx(@_LEDPin)



PRI startx(ptr) : okay

  stop
  longmove(@ins, ptr, 3)        'Copy the 3 parameters from the stack into the ins array

  return cog := cognew(@entry, @ins) + 1


PUB stop

'' Stop driver - frees a cog

  if cog
    cogstop(cog~ - 1)




DAT

'*********************************************
'* Assembly language LSM9DS1 + LPS25H driver *
'*********************************************

                        org

entry                   mov     t1,par                  'read parameters

                        'First param is LED pin mask
                        call    #param
                        mov     ledMask, t2
                        or      dira, t2

                        call    #param                  'setup LED Address
                        mov     ledAddress, t3

                        call    #param                  'setup LED count
                        mov     ledCount, t3

                        or      outa,ledMask            'bring LED pin high


'Main LED loop

main_loop

                        call    #WriteLEDs

                        mov     LoopTime, cnt
                        add     LoopTime, LoopDelay
                        waitcnt LoopTime, #0

                        jmp     #main_loop              'Repeat forever



''------------------------------------------------------------------------------
'' Get parameter, advance parameter pointer, result MASK in t2, VALUE in t3
''------------------------------------------------------------------------------
param                   rdlong  t3,t1                   'get parameter into t3
                        add     t1,#4                   'point to next parameter
                        mov     t2,#1                   'make pin mask in t2
                        shl     t2,t3
param_ret               ret
'------------------------------------------------------------------------------




''------------------------------------------------------------------------------
'' Write RGB values out to the WS2812b LED array
''------------------------------------------------------------------------------
WriteLEDs
                        andn    outa, ledMask           'Drive the LED line low to reset

                        mov     t3, ledCount
                        mov     t1, ledAddress               
                        
                        mov     ledDelay, cnt
                        add     ledDelay, LED_RESET     'wait for the reset time

                        waitcnt ledDelay, #0

:ledLoop

                        rdlong  spi_bits, t1            'Read the RGB triple from hub memory
                        add     t1, #4                  'Increment to the next address
                        
                        shl     spi_bits, #8            'high bit is the first one out, so shift it into position
                        mov     spi_bitcount, #24       '24 bits to send
:bitLoop
                        rcl     spi_bits, #1    wc

        if_nc           mov     ledDelay, #LED_0_HI  
        if_c            mov     ledDelay, #LED_1_HI                

                        or      outa, ledMask
                        add     ledDelay, cnt           'sync the timer to the bit-delay time
                          
        if_nc           waitcnt ledDelay, #LED_0_LO
        if_c            waitcnt ledDelay, #LED_1_LO                
         
                        andn    outa, ledMask           'pull the LED pin low             
                        waitcnt ledDelay, #0            'hold for the bit duration

                        djnz    spi_bitcount, #:bitLoop

                        djnz    t3, #:ledLoop
                        
WriteLEDs_ret           ret



'------------------------------------------------------------------------------


'
' Initialized data
'
LoopDelay               long    _clkfreq / 100          'Main loop at 100 hz

LED_RESET               long    5000                    'minimum of 50 * ONE_uS = 4000 @ 80MHz

'
' Uninitialized data

spi_bits                res     1                       'SPI bits to output in SPI_SendBits function
spi_bitcount            res     1                       'Number of bits to send / receive

t1                      res     1                       '
t2                      res     1                       'internal temporary registers
t3                      res     1                       '

ledmask                 res     1                       'LED pin mask
ledAddress              res     1                       'HUB Address of LED values
ledCount                res     1
ledDelay                res     1                       'Next counter value to wait for when sending / receiving


LoopTime                res     1                       'Register used to measure how much time a single loop actually takes


FIT 496       'Make sure all of the above fits into the cog (from the org statement to here)


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