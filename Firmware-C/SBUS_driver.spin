{{
************************************
* Futaba S-BUS reciever driver     *
************************************
  Elev8 Flight Controller - V1.0

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

  Written by Jason Dorie
}}



''*****************************************************************

''*****************************************************************
'' Coded by Jason Dorie
''
''
''*****************************************************************
''
''
''---------------------------------------------------------------------------------
''
''---------------------------------------------------------------------------------


VAR
  long  Cog

  long  InputMask
  long  BaudDelay
  word  Channels[18]  'Last two channels are digital
  word  CenterOffset


PUB Start( InputPin , Center )
  CenterOffset := Center
  InputMask := 1 << InputPin
  BaudDelay := ClkFreq / 100_000                        'SBUS is 100_000 bps 
  Cog := cognew(@SBUSStart, @InputMask) + 1                                             


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
SBUSStart
                        mov     Index,                  par                     'Set Index Pointer
                        rdlong  _InputPin,              Index                   'Get I/O pin directions
                        
                        add     Index,                  #4                      'Increment Index to next Pointer
                        rdlong  _BaudDelay,             Index                   'Get I/O pin directions
                        
                        add     Index,                  #4                      'Increment Index to next Pointer
                        mov     _HubChannels,           Index                   'Get HUB address to write SBUS data


                        call    #FindPacketEnd
                        
ReceiveLoop
                        call    #ReadInputBytes
                        call    #ConvertToChannels
                        call    #OutputToHub

                        jmp     #ReceiveLoop


'------------------------------------------------------------------------------------------------------------------------------------------------

ReadInputBytes
                        mov     LoopCounter, #25                                'Number of bytes to receive
                        movd    :writeByte, #inputBytes                         'Write the destination address into the output instruction                        

:byteLoop
                        mov     timer, _BaudDelay
                        shr     timer, #1                                       'The first timer is to advance to the middle of the bit (1/2 a bit delay)
                        add     timer, _BaudDelay                               '...plus a whole bit for the start bit itself                                                       

                        waitpne _InputPin, _InputPin                            'Wait for the input pin to be low
                        waitpeq _InputPin, _InputPin                            'Wait for the input pin to go high (start bit)
                        
                        add     timer, cnt                                      'Add the current counter value                        

                        mov     bitCount, #9                                    '8 data bits, 1 parity (we already waited for the start bit)
                        mov     inByte, #0
:bitLoop
                        waitcnt timer, _BaudDelay                               'Wait for the middle of the bit
                        test    _InputPin, ina  wc                              'sample it into the carry flag
                        rcl     inByte, #1                                      'Shift up for the next bit
                        djnz    bitCount, #:bitLoop                             'Do that for all the bits

                        shr     inByte, #1                                      'Discard the bottom bits
                        xor     inByte, #255                                    'invert the byte                        

:writeByte              mov     inputBytes, inByte                              'Write the output into the byte array
                        add     :writeByte, d_field                             'Increment the byte array offset to write to                                

                        djnz    LoopCounter, #:byteLoop    

ReadInputBytes_ret      ret


'------------------------------------------------------------------------------------------------------------------------------------------------
ConvertToChannels
                        cmp     inputBytes, #$F0        wz
              if_e      jmp     #:DoConvert

                        call    #FindPacketEnd
                        jmp     #ConvertToChannels_ret              

:DoConvert              movs    :readByte, #inputBytes+1
                        movd    :writeChannel, #channelData

                        mov     LoopCounter, #16                                'Number of channels to read                                                
                        mov     temp, #0
                        mov     bitCount, #0

  :readByte             mov     inByte, inputBytes                              'Read an input byte
                        add     :readByte, #1                                   'Increment the read address
                        
                        shl     temp, #8                                        'Shift up the bit rack
                        or      temp, inByte                                    'OR in the new bits
                        add     bitCount, #8                                    'Update the count of available bits

                        cmp     bitCount, #11   wc                              'Do we have enough for an output?
              if_c      jmp     #:readByte

                        mov     outShift, bitCount
                        sub     outShift, #11

                        mov     outChannel, temp
                        shr     outChannel, outShift                            'Shift down to drop the low bits
                        shl     outChannel, outShift                            'Shift the output back up
                        xor     temp, outChannel                                'remove these bits from the bit rack
                        sub     bitCount, #11                                   'Subtract off the count of bits we've consumed                                                           

                        shr     outChannel, outShift                            'Shift back down again (this time we're keeping it)
                        rev     outChannel, #(32-11)                            'Reverse the lower 11 bits - SBUS transmits them little-endian

   :writeChannel        mov     channelData, outChannel                         'Write the output to the channel array
                        add     :writeChannel, d_field                          'Increment the write address

                        djnz    LoopCounter, #:readByte                         'Keep going until we're finished                        

                        mov     inByte, inputBytes+23
                        test    inByte, #$80    wc
                        muxc    channelData+16, #255                         

                        test    inByte, #$40    wc
                        muxc    channelData+17, #255                         

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

                        'Wait for the input pin to be low for 1ms
:StartLoop
                        waitpne _InputPin, _InputPin                            'wait for the input pin to be low
                        mov     StartTime, cnt                                  'record the start time                        

:WaitLoop
                        'If the pin is high, branch back to waitLoop
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
d_field                 long    $0000_0200
d_and_s_field           long    $0000_0201
millisecond             long    80_000_000 / 1000


_InputPin               res     1
_BaudDelay              res     1
_HubChannels            res     1

timer                   res     1
StartTime               res     1

Index                   res     1
temp                    res     1
inByte                  res     1
outChannel              res     1
outShift                res     1        
bitCount                res     1

HubAddress              res     1
LoopCounter             res     1

InputIndex              res     1
OutputIndex             res     1

inputBytes              res     25
channelData             res     18



fit 496

DAT
{{
}}