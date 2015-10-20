{{
************************************
* Servo32-HighRes Driver v1.1      *
* Author: Jason Dorie              *
* See end of file for terms of use *
************************************
}}


''
''*****************************************************************
'' Control up to 32-Servos     Version1.1                08-14-2015 
''*****************************************************************
'' Coded by Jason Dorie
''
'' This is a from-scratch implementation of a 32 output servo driver
''
'' It differs from the standard Parallax driver in a number of ways.
''
'' The Parallax driver will output up to 32 servos at the normal
'' servo update rate of 50Hz, with a precision of 1us, or 1000 steps,
'' That's 80 clock cycles when running @ 80MHz, and is perfectly fine
'' for most applications.
''
'' Some, however, require higher output rates - Multi-rotor
'' applications often want a higher output rate - up to 400Hz.
'' Some applications require higher precision - High-resolution servos
'' exist with a precision of 0.25us (4000 steps) 
''
'' This driver addresses both needs at once.  It allows the user to
'' specify the desired output rate, and has been tested up to 500Hz.
'' You can specify per-pin which outputs are fast (high rate) and
'' which are slow (standard 50Hz).  Each output has a precision of
'' 10 clocks, which translates to 8000 steps over the standard servo
'' range of 1ms to 2ms.
''
'' NOTE: Be careful about your pulse length ranges.  This driver
'' does no clamping, so if you set a pulse length LONGER than your
'' update cycle time you'll hang the driver.  For example, setting a
'' pulse length of 2ms when running at 500Hz.  It's also possible
'' to set pulse lengths too low - Sorting the pulse array is done
'' during the initial part of the pulse - Servos expect a minimum pulse
'' length of 1ms, so I do the work DURING that time.  That time
'' translates to 80_000 clocks.  Worst case the driver takes less
'' than 40_000 clocks to complete.  I've tested with pulse lengths as
'' short as 1/2ms (a value of 4000), and it worked fine. 
''
''*****************************************************************
''
'' The preferred circuit of choice is to place a 4.7K resistor on each signal input
'' to the servo.  If long leads are used, place a 1000uF cap at the servo power
'' connector.  Servo's seem to be happy with a 5V supply receiving a 3.3V signal.
''
''---------------------------------------------------------------------------------
'' Prior to calling the Start method, call either AddFastPin or AddSlowPin for each
'' output you wish to use.  AddFastPin configures the pin for the high output speed,
'' whereas AddSlowPin configures that pin for 50Hz (standard) output. 
''
'' BE VERY CAREFUL if you use this code with standard servos as you can fry them.
'' High output rates will not work with standard analog servos, and will generally
'' cause their internals to short out.  To use a normal servo, use the AddSlowPin
'' function for that output.
''
'' Digital servos are a different story - It varies by manufacturer, but most digital
'' servos will handle update rates above 200Hz, many above 300Hz.
''
'' Electronic Speed Controls (ESCs) also accept high output rates - 400Hz is normal
'' for multi-rotor applications 
''---------------------------------------------------------------------------------


VAR
        long          FastPins, SlowPins
        long          _MasterLoopDelay, _SlowUpdateCounter
        long          Cycles
        long          ServoData[32]                                              'Servo Pulse Width information

        'long          SortedPins[32]                   'Only enable these if sending results back from the COG
        'long          SortedDelays[32]

        'long          Scale        
        

PUB Start
    cognew(@ServoStart, @FastPins)                                             



PUB Init( fastRate )

  _MasterLoopDelay := ClkFreq / fastRate
  _SlowUpdateCounter := (ClkFreq / 50) / _MasterLoopDelay


  '10 clocks is the smallest amount we can wait - everything else is based on that.
  'If you're using a different clock speed, your center point will likely need to be adjusted
  Scale := 10                                          

  FastPins := 0
  SlowPins := 0


''Set a PIN index as a high-speed output (250Hz)  
PUB AddFastPin(Pin)
  FastPins |= (1<<Pin)                                            
  SlowPins |= (1<<Pin)          'Fast pins are also updated during the "slow" cycle                                              


''Set a PIN index as a standard-speed output (50Hz)  
PUB AddSlowPin(Pin)
  SlowPins |= (1<<Pin)          'Slow pins are ONLY updated during the "slow" cycle


PUB Set(ServoPin, Width)                                'Set Servo value as a raw delay, in 10 clock increments
  ServoData[ServoPin] := Width * Scale                  'Servo widths are set in 10ths of a uS, so 8000 = min, 12000 = mid, 16000 = max

PUB SetRC(ServoPin, Width)                              'Set Servo value signed, assuming 12000 is your center
  ServoData[ServoPin] := (12000 + Width) * Scale        'Servo widths are set in 10ths of a uS, so -4000 min, 0 = mid, +4000 = max


PUB GetCycles
  return Cycles

'PUB GetSortedPins
'  return @SortedPins

'PUB GetSortedDelays
'  return @SortedDelays

    
DAT

'*********************
'* Assembly language *
'*********************
org
                        
'------------------------------------------------------------------------------------------------------------------------------------------------
ServoStart
                        mov     Index,                  par                     'Set Index Pointer
                        rdlong  _FastPinMask,           Index                   'Get I/O pin directions
                        
                        add     Index,                  #4                      'Increment Index to next Pointer
                        rdlong  _SlowPinMask,           Index                   'Get slow I/O pin directions

                        add     Index,                  #4
                        rdlong  MasterLoopDelay,        Index 

                        add     Index,                  #4
                        rdlong  SlowUpdateCounter,      Index 

                        add     Index,                  #4                      'Increment Index to next Pointer
                        mov     _Cycles,                Index                   'Get HUB address to write cycle time
                        
                        add     Index,                  #4                      'Increment Index to hub servo array
                        mov     _ServoHubArrayPtr,      Index                   'Set Pointer for hub servo array

                        mov     MasterLoopTimer, cnt                            'Start time for servo cyles
                        add     MasterLoopTimer, MasterLoopDelay

                        mov     OUTA, #0                                        'Initialize outputs to zero                
                        mov     dira, _SlowPinMask                              'Enable all relevant pins (fast and slow) as outputs

SlowPinLoop
                        mov     PinMask, _SlowPinMask                           'For the first pass, include slow pins and fast pins
                        mov     OuterLoopCount, SlowUpdateCounter
'------------------------------------------------------------------------------------------------------------------------------------------------
FastPinLoop
                        call    #ServoCore                                      'Run the servo update

                        waitcnt MasterLoopTimer, MasterLoopDelay                'wait for a cycle
                        

                        mov     PinMask, _FastPinMask                           'Only update the fast pins during the fast passes
                        djnz    OuterLoopCount, #FastPinLoop
                        
                        jmp     #SlowPinLoop


                        
'------------------------------------------------------------------------------------------------------------------------------------------------
ServoCore

                        mov     Clock, cnt
                        mov     PulseStartTime, Clock                           'Grab the time the pulses start

                        nop                                                     'Offset a little to account for the delay in UN-setting the pins
                        mov     OUTA, PinMask                                   'Set all output pins high - we now have ~1ms to start turning them off
                        
                        'Create the array of pin masks and copy the servo delays from hub to cog
                        call    #CreateServoArray
                        
                        'Sort the array of pin masks / delay values in pin mask order
                        call    #SortServoArray
                        
                        'Congeal entries with identical delays and decrement the output count
                        call    #CompactServoArray

                        'Calculate how long that all took
                        subs    Clock, cnt             'Subtract the current time from the start time
                        neg     Clock, Clock            'EarlierTime - LaterTime = a negative number (negate it)

                        wrlong  Clock, _Cycles          'Write the cycle count back to the HUB so it can be printed                                                                        


                        'Output the servo pulses
                        call    #OutputServoPulses

                        'call    #CopyToHub      'For testing only - Un-comment SortedPins / SortedDelays for this to work

ServoCore_RET           ret


'------------------------------------------------------------------------------------------------------------------------------------------------
CreateServoArray
                        movd    :pinWrite, #ServoPins                           'Starting location to write the pin masks to (self modifying code)                        
                        movd    :delayWrite, #ServoDelays                       'Starting location in COG to copy the servo delays array to
                        
                        mov     HubAddress, _ServoHubArrayPtr                   'Address of the source data in HUB ram                        
                        mov     PinMask, #1                                     'First pin mask value to write                        

                        mov     LoopCounter, #32                                'Number of array entries to set

:Loop
   :pinWrite            mov     ServoPins, PinMask                              'Set the current pin mask value
   :delayWrite          rdlong  ServoDelays, HubAddress                         'Read the HUB value into COG memory
   
                        add     :pinWrite, d_field                              'Increment the destination address
                        shl     PinMask, #1                                     'Shift the pin mask to the next pin

                        add     :delayWrite, d_field                            'Increment the COG address to write to
                        add     HubAddress, #4                                  'Increment the HUB address to read from

                        djnz    LoopCounter, #:Loop                             'Loop until all 32 values are written                                    

CreateServoArray_ret    ret


'------------------------------------------------------------------------------------------------------------------------------------------------
'These instructions are only here so they can be copied into code below

        InitialCompare          cmp     ServoDelays+1, ServoDelays     wz, wc
        InitialDelaySwap        mov     ServoDelays, ServoDelays+1
        InitialPinSwap          mov     ServoPins, ServoPins+1


'------------------------------------------------------------------------------------------------------------------------------------------------
SortServoArray
                        mov     PassCounter, #31

:PassLoop               'Do a full pass over the array of PassCounter servo entries
                        mov     LoopCounter, PassCounter
                        mov     :compare, InitialCompare

                        movs    :swapDelay1, #ServoDelays                       'Set up the intial delay swap instructions
                        mov     :swapDelay2, InitialDelaySwap
                        movd    :swapDelay3, #ServoDelays+1

                        movs    :swapPin1, #ServoPins                           'Set up the intial pin swap instructions
                        mov     :swapPin2, InitialPinSwap
                        movd    :swapPin3, #ServoPins+1

:CompareLoop

   :compare             cmp     ServoDelays, ServoDelays     wz, wc             'Comparison of delay values (to be self modified)
              if_nc     jmp     #:SkipSwap

   :swapDelay1          mov     temp, ServoDelays
   :swapDelay2          mov     ServoDelays, ServoDelays
   :swapDelay3          mov     ServoDelays, temp

   :swapPin1            mov     temp, ServoPins
   :swapPin2            mov     ServoPins, ServoPins
   :swapPin3            mov     ServoPins, temp

:SkipSwap
                        'Advance all the delay swap instruction source and destination addresses by one
                        add     :swapDelay1, #1
                        add     :swapDelay2, d_and_s_field
                        add     :swapDelay3, d_field

                        'Advance all the pins swap instruction source and destination addresses by one
                        add     :swapPin1, #1
                        add     :swapPin2, d_and_s_field
                        add     :swapPin3, d_field

                        add     :compare, d_and_s_field                         'Increment the addresses being compared                        
                        djnz    LoopCounter, #:CompareLoop

                        djnz    PassCounter, #:PassLoop                                                                        

SortServoArray_ret      ret



'------------------------------------------------------------------------------------------------------------------------------------------------
CompactServoArray

                        mov     OutputIndex, #0
                        mov     LoopCounter, #31
                        
                        movs    :pinMerge, #ServoPins+1          'Set the first input address for all the self-modifying instructions
                        movs    :pinMove, #ServoPins+1

                        movs    :delayCompare, #ServoDelays+1
                        movd    :delayCompare, #ServoDelays                     'Write the output address into the compare instruction
                        movs    :delayMove, #ServoDelays+1

                        movd    :pinMerge, #ServoPins                           'Set the output addresses for pinMerge, pinMove, delayMove
                        movd    :pinMove, #ServoPins
                        movd    :delayMove, #ServoDelays

:Loop
   :delayCompare        cmp     ServoDelays, ServoDelays  wz, wc                'Compare delay value against current output index (self modifying)
              if_ne     jmp     #:moveEntry

:mergeEntry
                        'OR the pin mask for OutputIndex and InputIndex together

   :pinMerge            or      ServoPins, ServoPins                            'OR the pin masks together (self modifying)                        
                        jmp     #:LoopEnd
                        
:moveEntry
                        'Increment all the output addresses              
                        add     OutputIndex, #1                                 'Increment the OutputIndex
                        add     :pinMerge, d_field
                        add     :pinMove, d_field
                        add     :delayMove, d_field
                        add     :delayCompare, d_field
                        
                        'Copy the pin mask value from InputIndex to OutputIndex

   :pinMove             mov     ServoPins, ServoPins                            'Move the pin mask from InputIndex to OutputIndex
   :delayMove           mov     ServoDelays, ServoDelays                        'Copy the delay value from InputIndex to OutputIndex

:LoopEnd
                        add     :pinMerge, #1                                   'Increment the InputIndex (for all the self-modifying instructions)
                        add     :pinMove, #1
                        add     :delayMove, #1
                        add     :delayCompare, #1
                        
                        djnz    LoopCounter, #:Loop


                        add     OutputIndex, #1
                        mov     ServoCount, OutputIndex                         'Number of servo delays in the compacted array

CompactServoArray_ret   ret



'------------------------------------------------------------------------------------------------------------------------------------------------
AddPulseStartTime
                        add     ServoDelays+0,  PulseStartTime
                        add     ServoDelays+1,  PulseStartTime
                        add     ServoDelays+2,  PulseStartTime
                        add     ServoDelays+3,  PulseStartTime
                        add     ServoDelays+4,  PulseStartTime
                        add     ServoDelays+5,  PulseStartTime
                        add     ServoDelays+6,  PulseStartTime
                        add     ServoDelays+7,  PulseStartTime
                        add     ServoDelays+8,  PulseStartTime
                        add     ServoDelays+9,  PulseStartTime
                        add     ServoDelays+10, PulseStartTime
                        add     ServoDelays+11, PulseStartTime
                        add     ServoDelays+12, PulseStartTime
                        add     ServoDelays+13, PulseStartTime
                        add     ServoDelays+14, PulseStartTime
                        add     ServoDelays+15, PulseStartTime
                        add     ServoDelays+16, PulseStartTime
                        add     ServoDelays+17, PulseStartTime
                        add     ServoDelays+18, PulseStartTime
                        add     ServoDelays+19, PulseStartTime
                        add     ServoDelays+20, PulseStartTime
                        add     ServoDelays+21, PulseStartTime
                        add     ServoDelays+22, PulseStartTime
                        add     ServoDelays+23, PulseStartTime
                        add     ServoDelays+24, PulseStartTime
                        add     ServoDelays+25, PulseStartTime
                        add     ServoDelays+26, PulseStartTime
                        add     ServoDelays+27, PulseStartTime
                        add     ServoDelays+28, PulseStartTime
                        add     ServoDelays+29, PulseStartTime
                        add     ServoDelays+30, PulseStartTime
                        add     ServoDelays+31, PulseStartTime

AddPulseStartTime_ret   ret


'------------------------------------------------------------------------------------------------------------------------------------------------
OutputServoPulses
                        cmp     ServoDelays, #0 wz                              'Check the first entry to see if the delay is zero
              if_nz     mov     :EntryJump, :AllOutputs                                             
              if_z      mov     :EntryJump, :SkipFirst                                             

                        call    #AddPulseStartTime                              'Add the PulseStartTime value to all the servo delays

                        mov     temp, ServoCount
                        shl     temp, #1
                        add     temp, #:DoPulseOutputs                          'Find the LAST waitcnt instruction we're going to use

                        movs    :saveWait, temp         'Alter some instructions to write into the table below                                
                        movd    :restoreWait, temp        
                        movd    :createJump, temp

   :saveWait            mov     SavedWaitcnt, :DoPulseOutputs                   'Save the instrction we're about to overwrite
   :createJump          mov     :DoPulseOutputs, :doneJump                      'Replace it with a jmp :done instruction
'{

                        'The table below may look dodgy, but it gives us a granularity of 10 clocks

  :EntryJump            jmp     #:DoPulseOutputs        'We're going to replace this instruction with one of the two below

  :AllOutputs           jmp     #:DoPulseOutputs        'This jump is used when all 32 servo outputs are valid
  :SkipFirst            jmp     #:SkipFirstOutput       'This jump is used to bypass the first delay entry, when some servos are unused

:DoPulseOutputs
                        waitcnt ServoDelays+0, #0
                        andn    OUTA, ServoPins+0
:SkipFirstOutput                        
                        waitcnt ServoDelays+1, #0
                        andn    OUTA, ServoPins+1
                        waitcnt ServoDelays+2, #0
                        andn    OUTA, ServoPins+2
                        waitcnt ServoDelays+3, #0
                        andn    OUTA, ServoPins+3
                        waitcnt ServoDelays+4, #0
                        andn    OUTA, ServoPins+4
                        waitcnt ServoDelays+5, #0
                        andn    OUTA, ServoPins+5
                        waitcnt ServoDelays+6, #0
                        andn    OUTA, ServoPins+6
                        waitcnt ServoDelays+7, #0
                        andn    OUTA, ServoPins+7
                        waitcnt ServoDelays+8, #0
                        andn    OUTA, ServoPins+8
                        waitcnt ServoDelays+9, #0
                        andn    OUTA, ServoPins+9
                        waitcnt ServoDelays+10, #0
                        andn    OUTA, ServoPins+10
                        waitcnt ServoDelays+11, #0
                        andn    OUTA, ServoPins+11
                        waitcnt ServoDelays+12, #0
                        andn    OUTA, ServoPins+12
                        waitcnt ServoDelays+13, #0
                        andn    OUTA, ServoPins+13
                        waitcnt ServoDelays+14, #0
                        andn    OUTA, ServoPins+14
                        waitcnt ServoDelays+15, #0
                        andn    OUTA, ServoPins+15
                        waitcnt ServoDelays+16, #0
                        andn    OUTA, ServoPins+16
                        waitcnt ServoDelays+17, #0
                        andn    OUTA, ServoPins+17
                        waitcnt ServoDelays+18, #0
                        andn    OUTA, ServoPins+18
                        waitcnt ServoDelays+19, #0
                        andn    OUTA, ServoPins+19
                        waitcnt ServoDelays+20, #0
                        andn    OUTA, ServoPins+20
                        waitcnt ServoDelays+21, #0
                        andn    OUTA, ServoPins+21
                        waitcnt ServoDelays+22, #0
                        andn    OUTA, ServoPins+22
                        waitcnt ServoDelays+23, #0
                        andn    OUTA, ServoPins+23
                        waitcnt ServoDelays+24, #0
                        andn    OUTA, ServoPins+24
                        waitcnt ServoDelays+25, #0
                        andn    OUTA, ServoPins+25
                        waitcnt ServoDelays+26, #0
                        andn    OUTA, ServoPins+26
                        waitcnt ServoDelays+27, #0
                        andn    OUTA, ServoPins+27
                        waitcnt ServoDelays+28, #0
                        andn    OUTA, ServoPins+28
                        waitcnt ServoDelays+29, #0
                        andn    OUTA, ServoPins+29
                        waitcnt ServoDelays+30, #0
                        andn    OUTA, ServoPins+30
                        waitcnt ServoDelays+31, #0
                        andn    OUTA, ServoPins+31

   :doneJump            jmp     #:done          'These are 'fake' entries for output #33, in case we output all servos
                        nop                     'The first instruction is the template for the jmp :done instruction we copy over other things
                                                
'}
:done
   :restoreWait         mov     :DoPulseOutputs, SavedWaitcnt                   'Put back the instruction we overwrote
                        

OutputServoPulses_ret   ret


{
'This function is primarily for testing - copies the sorted pins / delays arrays back to HUB memory for inspection
'To use, you must un-comment the SortedPins and SortedDelays arrays or you will overwrite your program
'------------------------------------------------------------------------------------------------------------------------------------------------
CopyToHub
                        movd    :hubWrite, #ServoPins                           'Starting location in COG to copy the sorted data from
                        
                        mov     HubAddress, _ServoHubArrayPtr                   'Address of the source data in HUB ram
                        add     HubAddress, #128                                'Hub address of the sorted array                        
                                                
                        mov     LoopCounter, #64                                'Number of array entries to transfer (pins + delays)

:Loop
   :hubWrite            wrlong  ServoPins, HubAddress                           'Write the COG value to HUB memory
   
                        add     HubAddress, #4                                  'Increment the HUB address to write to
                        add     :hubWrite, d_field                              'Increment the COG address to read from
                        
                        djnz    LoopCounter, #:Loop                             'Loop until all 64 values are written                                    

CopyToHub_ret           ret
}


'------------------------------------------------------------------------------------------------------------------------------------------------
d_field                 long    $0000_0200
d_and_s_field           long    $0000_0201
MasterLoopDelay         long    80_000_000 / 400        'Note that this value gets replaced on init, before sending to the cog for execution
SlowUpdateCounter       long    8

_ServoHubArrayPtr       res     1
_Cycles                 res     1
Clock                   res     1
PulseStartTime          res     1
MasterLoopTimer         res     1
SavedWaitcnt            res     1
DidSwap                 res     1

temp                    res     1
Index                   res     1
HubAddress              res     1

PassCounter             res     1
LoopCounter             res     1

_FastPinMask            res     1
_SlowPinMask            res     1
OuterLoopCount          res     1

InputIndex              res     1
OutputIndex             res     1

ServoCount              res     1
PinMask                 res     1


ServoPins               res     32
ServoDelays             res     32



fit 496

DAT
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