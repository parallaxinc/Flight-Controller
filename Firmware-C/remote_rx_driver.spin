{{
*************************************
* Spektrum RemoteRX reciever driver *
*************************************

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

VAR
  long  Cog

  long  InputMask
  long  BaudDelay
  word  Channels[16]
  word  CenterOffset


PUB Start( InputPin , Center )
  CenterOffset := Center
  InputMask := 1 << InputPin
  BaudDelay := ClkFreq / 120_000                        'RemoteRX is 125_000 bps, but 115200 is allowed, so split the difference
  Cog := cognew(@RMRXStart, @InputMask) + 1                                             


PUB stop
'' Stop driver and release cog
  if Cog
    cogstop(Cog~ - 1)


PUB Get( i )
  return Channels[i] 

PUB GetRC( i )
  return Channels[i] - CenterOffset 


    
DAT

'*********************
'* Assembly language *
'*********************
org
                        
'------------------------------------------------------------------------------------------------------------------------------------------------
RMRXStart
                        mov     Index,                  par                     'Set Index Pointer
                        rdlong  _InputPin,              Index                   'Get I/O pin directions
                        
                        add     Index,                  #4                      'Increment Index to next Pointer
                        rdlong  _BaudDelay,             Index                   'Get I/O pin directions
                        
                        add     Index,                  #4                      'Increment Index to next Pointer
                        mov     _HubChannels,           Index                   'Get HUB address to write channel data


                        call    #FindPacketEnd                                  'Initial power-up wait
                        
ReceiveLoop
                        call    #ReadInputBytes
                        call    #ConvertToChannels
                        call    #OutputToHub
                        call    #DelayHalfMillisecond   'some receivers send a checksum (like SRXL) wait past it

                        jmp     #ReceiveLoop

                                                   
'------------------------------------------------------------------------------------------------------------------------------------------------

ReadInputBytes
                        mov     LoopCounter, #16                                'Number of bytes to receive  (SRXL is 18 bytes, header = 0xA5)
                        movd    :writeWord, #inputWords                         'Write the destination address into the output instruction                        
                        mov     inWord, #0
:byteLoop
                        mov     timer, _BaudDelay
                        shr     timer, #1                                       'The first timer is to advance to the middle of the bit (1/2 a bit delay)
                        add     timer, _BaudDelay                               '...plus a whole bit for the start bit itself                                                       

                        waitpeq _InputPin, _InputPin                            'Wait for the input pin to go high
                        waitpne _InputPin, _InputPin                            'Wait for the input pin to be low (start bit)
                        
                        add     timer, cnt                                      'Add the current counter value                        

                        mov     bitCount, #8                                    '8 data bits, (we already waited for the start bit)
:bitLoop
                        waitcnt timer, _BaudDelay                               'Wait for the middle of the bit
                        test    _InputPin, ina  wc                              'sample it into the carry flag
                        rcl     inWord, #1                                      'Shift up for the next bit
                        djnz    bitCount, #:bitLoop                             'Do that for all the bits

                        test    LoopCounter, #1         wc
        if_nc           jmp     #:skipWrite                                     'only write whole words (every other byte read)

                        rev     inWord, #(32-16)                                'reverse the order of the low 16 bits

                        'Endian swap the high & low bytes 
                        mov     bitCount, inWord
                        shl     bitCount, #8
                        shr     inWord, #8
                        or      inWord, bitCount
                        and     inWord, wordMask 
                         
:writeWord              mov     inputWords, inWord                              'Write the output into the byte array
                        mov     inWord, #0                                      'Clear the value for the next read
                        add     :writeWord, d_field                             'Increment the array offset to write to                                

:skipWrite
                        djnz    LoopCounter, #:byteLoop    

ReadInputBytes_ret      ret


'------------------------------------------------------------------------------------------------------------------------------------------------
'When this is called we'll have a header word (16 bits) followed by 7 channel words
' each of which will have a channel index and a value stuck together
'------------------------------------------------------------------------------------------------------------------------------------------------
ConvertToChannels


                        mov     inWord, inputWords
                        and     inWord, #$ff
                        cmp     inWord, #$12    wz                              '11ms 2048 DSM2 master
              if_e      jmp     #:DoConvert

                        cmp     inWord, #$02    wz                              '11ms 2048 SRXL master packet 1
              if_e      jmp     #:DoConvert

                        cmp     inWord, #$03    wz                              '11ms 2048 SRXL master packet 2
              if_e      jmp     #:DoConvert

                        cmp     inWord, #0      wz                              '11ms 2048 DSM2 remote
              if_z      jmp     #:DoConvert

                        call    #FindPacketEnd
                        jmp     #ConvertToChannels_ret              

:DoConvert              movs    :readWord, #inputWords+1
                        mov     LoopCounter, #7                                 'Number of channels to read                                                

                        'extract the channel ID (AND with channel ID mask, shift down)

  :readWord             mov     inWord, inputWords                              'Read an input word
                        add     :readWord, #1                                   'Increment the read address

                        'extract the channel value (AND with value mask, possible shift up, if 10 bit)
        
                        mov     outChannel, inWord                              'extract the channel index from the value                        
                        shr     outChannel, channelShift
                        and     outChannel, #15

                        'add the channel output target to the channel ID
                        add     outChannel, #channelData                        'add the start of the channel array
                        movd    :writeChannel, outChannel                       'set the output address of the write operation

                        and     inWord, valueMask
                        shl     inWord, valueShift                              'only necessary to support 10-bit data     
                                               
   :writeChannel        mov     channelData, inWord                             'Write the output to the channel array

                        djnz    LoopCounter, #:readWord                         'Keep going until we're finished                        


ConvertToChannels_ret   ret




'------------------------------------------------------------------------------------------------------------------------------------------------
OutputToHub
                        movd    :hubWrite, #channelData                         'Starting location in COG to copy the channel data from

                        mov     HubAddress, _HubChannels                        'Address of the channel data in HUB ram
                        mov     LoopCounter, #16                                'Number of channel entries to transfer

:Loop
   :hubWrite            wrword  channelData, HubAddress                         'Write the COG value to HUB memory
   
                        add     HubAddress, #2                                  'Increment the HUB address to write to
                        add     :hubWrite, d_field                              'Increment the COG address to read from
                        
                        djnz    LoopCounter, #:Loop                             'Loop until all 64 values are written                                    

OutputToHub_ret         ret




'Called on startup to locate the end of a packet so we don't try to parse from the middle of one
'------------------------------------------------------------------------------------------------------------------------------------------------
FindPacketEnd

                        'Wait for the input pin to be high for 1ms
:StartLoop
                        waitpne _InputPin, _InputPin                            'wait for the input pin to be low
                        waitpeq _InputPin, _InputPin                            'wait for the input pin to transition high
                        mov     StartTime, cnt                                  'record the start time                        

:WaitLoop
                        'If the pin is low, branch back to waitLoop
                        test    INA, _InputPin  wc
              if_c      jmp     #:StartLoop
                        
                        'check to see if a millisecond has elapsed
                        mov     timer, cnt
                        sub     timer, StartTime
                        cmp     timer, millisecond      wc
                        
                        'if not, keep waiting
              if_c      jmp     #:WaitLoop     

FindPacketEnd_ret       ret



'------------------------------------------------------------------------------------------------------------------------------------------------
DelayHalfMillisecond
                        mov     timer, millisecond
                        shr     timer, #1
                        add     timer, cnt

                        waitcnt timer, #0
DelayHalfMillisecond_ret
                        ret


'------------------------------------------------------------------------------------------------------------------------------------------------
d_field                 long    $0000_0200
d_and_s_field           long    $0000_0201
millisecond             long    80_000_000 / 1000
channelShift            long    11
valueShift              long    0
valueMask               long    2047
wordMask                long    $FFFF            


_InputPin               res     1
_BaudDelay              res     1
_HubChannels            res     1

timer                   res     1
StartTime               res     1

Index                   res     1
temp                    res     1
inWord                  res     1
outChannel              res     1
outShift                res     1        
bitCount                res     1

HubAddress              res     1
LoopCounter             res     1

InputIndex              res     1
OutputIndex             res     1

inputWords              res     8
channelData             res     16



fit 496

DAT
{{
}}