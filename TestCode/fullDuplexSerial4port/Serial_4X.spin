'__________________________________________________________________________________
'   
'       Permission is hereby granted, free of charge, to any person obtaining a
'       copy of this software and associated documentation files (the "Software"),
'       to deal in the Software without restriction, including without limitation
'       the rights to use, copy, modify, merge, publish, distribute, sublicense,
'       and/or sell copies of the Software, and to permit persons to whom the
'       Software is furnished to do so, subject to the following conditions:                                                                   
'                                                                                                                              
'       The above copyright notice and this permission notice shall be included in
'       all copies or substantial portions of the Software.
'
'       THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
'       IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
'       FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
'       AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
'       LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
'       FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
'       IN THE SOFTWARE.
'
'       Portions of this work is derived in whole or in part from pre existing,
'       copyrighted work, subject to the "MIT License" terms and conditions,
'       each of which is described below:
'
'       Based in part upon full-duplex serial driver v1.01                                
'       Tracy Allen (TTA)   (c) 22-Jan-2011
'       Copyright (c) 2006 Parallax, Inc.                             *
'
'       Other attributions include, in alphabetic order:
'               Juergen Buchmueller
'               Duane Degn
'               Tim Moore
'____________________________________________________________________________________________
'
' Copyright (c) 2015 by Wayne Meretsky
'
' This version of the serial port driver was rewritten entirely from scratch in early
' 2015 with three goals, in order: Correctness, Performance, Code Size. While it is
' an entirely new implementation, it draws heavily on the design and implementation of
' many original and derived works that preceed it. 
'
' Tx and Rx pins are user defined and each port may use neither, either or both. The
' Baud rate is identical for Tx and Rx of a given port. Baud rates up to 460800K have
' been characterized and tested.

' To use the object:
'       1) Call Initialize
'       2) Call Define_Port for each port that will be used
'       3) Call Start
'       4) Make calls to any of the various input and output routines.
'       5) Call Stop.
'       6) Repeat as desired.
'
' Notes:
'       1) Associating the same pin with both Tx and Rx channels has undefined results
'       2) Rx and Tx buffers must not overlap and must be at least two bytes in size;
'          other than the limits of available memory there is no upper bound on their sizes 
'       3) Framing and overrun errors are not detected. Overruns of the receive buffer 
'          simply over write previously received data
'       4) Put operations, with the exception of Can_Put are blocking. They will not 
'          return until their data has been buffered for output. Put_Bytes buffers data
'          to the greatest extent possible and may be called with a Count that is greater
'          than the buffer size.
'       5) Flush_Output blocks until the last byte has been extracted from the buffer; but
'          that byte has not yet been transmitted at the time Flush returns
'       6) Get is a blocking operation; it will not return until the associated buffer has data.
'       7) Get_Bytes_Timed will not return until all requested data has arrived or the timer
'          has expired. It will not return partial data. Calling this routine with a Count
'          that is greater than the size of the receive buffer will never return any data
'
' Revision History
' 150220        Substantial rewrite including the following changes
'                 Improved calculation of TPB and SO
'                 Changed Can_Put to take a byte count
'                 Deleted Try_Put
'                 Modified Get_Timed to use Get_Bytes_Timed
'                 Modified Put_ZString to call Put_Bytes
'                 Modified Put_Bytes to use bytemove
'                 Modified Get_Bytes_Timed to use bytemove
'                 Reduced latency receiver
'                 Stopped shifting in stop bit
'                 Reduced latency of transmitter
'____________________________________________________________________________________________
'
' Data structures:
'
' The data structures used are not for the faint of heart. Worse yet, the Propeller
' architecture makes shared data structures both complicated to code and difficult
' to understand.
'
' The fundamental buffer structures are ring buffers designed for a single producer
' and a single consumer. They are inherently thread safe without the use of any kind
' of locking between producer and consumer. However, they are not tolerant of multiple
' producers or multiple consumers and any use requiring such must be serialized
' externally.
'
' The buffer storage is provided by the caller of Initialize. The buffers may be of any
' length greater than two bytes and as large as all available memory.
'
' Each channel (receive and transmit) has four variables used to control access. These are the
' Buffer Base Address, the Buffer Size (in bytes), the Insertion Index and the Extraction Index.
'
' Throughout, these things are called:
'   TxB   - Transmit Buffer Base
'   TxS   - Tranmsit Buffer Size
'   Tx_II - Transmit Buffer Insertion Index
'   Tx_EI - Tranmsit Buffer Extraction Index
'   RxB   - Receive Buffer Base
'   RxS   - Receive Buffer Size
'   Rx_II - Receive Buffer Insertion Index
'   Rx_EI - Receive Buffer Extraction Index
'
' If a channel is not in use, make sure its Pin No_Pin. There is no special code anywhere
' to make this work, it's simply a by product of the implementation.
'
' Insertion and Extraction indices are zero based and always in the range 0 .. Size - 1
'
' The Insertion Index is the index of the last inserted byte
' The Extraction Index is the index of the next byte available
' The Buffer is empty if the Insertion Index is equal to the Extraction Index
'
' The aforementioned buffer control variables are shared between the Spin and the Assembly
' language portions of the driver. The buffer base and buffer sizes are static and, thus,
' shared by copy. However, the Insertion and Extraction indicies are dynamic and must be
' shared by reference. Variables ending in _Ref and the pointers to the associated indicies
' so, for example, Tx_II_Ref must point to Tx_II. These reference are static and are shared
' by copy.
'  
con
  No_Pin        = 32            ' Valid pins are 0-31; this could be any other value

  Port_First      = 0
  Port_Last       = 3
  Ports           = Port_Last - Port_First + 1

pub Initialize | N

  bytefill (@Block_Clear_Start, 0, @Block_Clear_End - @Block_Clear_Start + 1)
'
' Despite being invariant, we cannot initialize the Index References
' statically because compile-time references aren't fixed up by the linker
'
  repeat N from Port_First to Port_Last
    Rx_II_Ref [N] := @Rx_II [N]
    Tx_II_Ref [N] := @Tx_II [N]
    Tx_EI_Ref [N] := @Tx_EI [N]

pub Define_Port (The_Port, The_Baud, The_TxP, The_TxB, The_TxS, The_RxP, The_RxB, The_RxS) | The_TPB
' 
' Initialize and define the port, initializes the structures and launches
' the assembly language portion of the driver. The parameters have the
' following semantics:
'
' Cog   The Cog on which to run the driver
' Baud  The bit rate of the Transmitter and Receiver expressed in Bits Per Second
'       9600 means 9600 Baud. There is no requirement to use industry standard baud
'       rates. All operations are with one start, eight data, no parity and one stop.     
' TxP   Pin number of the pin on which data is to be Transmitted. Use No_Pin if no
'       transmit functionality is desired.
' TxB   The address of the transmit buffer
' TxS   The size, in bytes, of the transmit buffer
' RxP   Pin number of the pin on which data is to be received. Use No_Pin if no
'       receive functionality is desired.
' RxB   The address of the receive buffer
' RxS   The size, in bytes, of the receive buffer
'
  if (The_RxP <> No_Pin)
    RxM [The_Port] := |< The_RxP

  if (The_TxP <> No_Pin)
    TxM [The_Port] := |< The_TxP
'
' At high baud rates, there's not a lot of margin for the Shift Offset. In limited
' testing, values of 5-28% work. 29% fails. 22% is used here. Rounding of both
' Ticks-per-bit and the Shift Offset are critical for high data rates.
'
  The_TPB        := (clkfreq + (The_Baud >> 1)) / The_Baud
  TPB [The_Port] := The_TPB
  SO  [The_Port] := ((The_TPB * 22) + 50) / 100

  TxB [The_Port] := The_TxB
  TxS [The_Port] := The_TxS
  RxB [The_Port] := The_RxB
  RxS [The_Port] := The_RxS
    
pub Start (The_Cog)

  coginit (The_Cog, @Serial_Driver, 0)

pub Stop (The_Cog)

  cogstop (The_Cog)

'__________________________________________________________________________________
'
' The Transmit Primitives
'
' Flush_Output, Put, Try_Put, Put_Bytes, Put_ZString
'
pub Flush_Output (The_Port)

  repeat while (Tx_EI [The_Port] <> Tx_II [The_Port])

pub Put (The_Port, The_Byte) | I, N
'
' Waits until buffer space is available and then buffers The_Byte for transmit
'
  I := Tx_II [The_Port]
  N := (I + 1) // TxS [The_Port]

  repeat while (Tx_EI [The_Port] == N)                  ' Wait for room

  byte [TxB [The_Port] + I] := The_Byte
  Tx_II [The_Port]          := N

pub Can_Put (The_Port, The_Count) | BC

  BC := Tx_EI [The_Port] - Tx_II [The_Port] - 1
  if (BC =< 0)
    BC := BC + TxS [The_Port] 

  return (BC => The_Count)

pub Put_Bytes (The_Port, The_Bytes, The_Count) | I, S, B, M, C

  S := TxS [The_Port]                                   ' Get the size
  B := TxB [The_Port]                                   ' Get the buffer addrss
  repeat while (The_Count <> 0)
    I := Tx_II [The_Port]                               ' I := insertion point
    M := I - Tx_EI [The_Port]                           ' M := Bytes already buffered
    if (M < 0)
      M := M + S
    M := (S - M - 1) <# The_Count                       ' M := Byte to move into the buffer now
    C := (S - I) <# M                                   ' C := bytes to move into the end of the buffer
    bytemove (B + I, The_Bytes,     C)                  ' Fill to the end of the buffer
    bytemove (B,     The_Bytes + C, M - C)              ' Fill the head of the buffer - might be nothing
    Tx_II [The_Port] := (I + M) // S                    ' Update the insertion point       
    The_Bytes        := The_Bytes + M                   ' Bump address
    The_Count        := The_Count - M                   ' decrement the bytes remaining

pub Put_ZString (The_Port, The_String) | B

  Put_Bytes (The_Port, The_String, strsize (The_String))
'__________________________________________________________________________________
'
' The Receive primitives
'
'
' Expunge_Input, Peek, Check, Get, Get_Timed, Get_Bytes_Timed
'
' Note that Get and Get_Timed are optimized for code size, not performance.
'
pub Expunge_Input (The_Port)

  Rx_EI [The_Port] := Rx_II [The_Port]

pub Peek (The_Port) | E
' Checks to see if a character is sitting in the Receive buffer If so, the character
' is returned to the caller.  If no character is available, a value of -1 is returned.
'
' In no case is the character removed from the buffer. A subsequent call to Get, for
' example, is required to debuffer the character.
'
  E := Rx_EI [The_Port]
  if (E == Rx_II [The_Port])
    return -1
    
  return byte [RxB [The_Port] + E]

pub Check (The_Port) | E, B

' Checks to see if a character is sitting in the Receive buffer of
' The_Port.  If so, the character is returned to the caller.  If no
' character is available, a value of -1 is returned.
'
  E := Rx_EI [The_Port]
  if (E == Rx_II [The_Port])
    return -1
    
  B := byte [RxB [The_Port] + E]
  Rx_EI [The_Port] := (E + 1) // RxS [The_Port]

  return B

pub Get (The_Port) | B
'
' Waits until a byte is available and then returns it to the caller
'
  repeat
    B := Check (The_Port)
  until (B => 0)
  return B

pub Get_Timed (The_Port, MS_Timer) | Success, B

  B := 0
  ifnot Get_Bytes_Timed (The_Port, @B, 1, MS_Timer)
    return -1
  return B

pub Get_Bytes_Timed (The_Port, The_Buffer, The_Count, MS_Timer) | Stamp, E, S, B, C
'
' Waits for up to MS_Delay for a The_Count bytes to arrive and copies them to
' The_Buffer, returning True.  If The_Timer elapses prior to their arrival,
' The_Buffer is unaltered and the return value is False
'
' No data is removed from the Receive Buffer unless it is all available
'
  Stamp := cnt
  E     := Rx_EI [The_Port]                             ' Get the extraction index
  S     := RxS   [The_Port]                             ' And the size
  B     := RxB   [The_Port]                             ' And the buffer base
  
  repeat
    C := Rx_II [The_Port] - E                           ' Compute bytes in buffer
    if (C < 0)
      C := C + S

    if (C => The_Count)                                 ' Is it all here yet?
      C := (S - E) <# The_Count                         ' Determine bytes to end of buffer
      bytemove (The_Buffer,     B + E, C)               ' Move first part
      bytemove (The_Buffer + C, B,     The_Count - C)   ' Move the rest, often none
      Rx_EI [The_Port] := (E + The_Count) // S          ' Update the extraction index 

      return True    

  until (((cnt - Stamp) / (clkfreq / 1000)) > MS_Timer)  

  return False
'
' The Assembly Language quad port driver
'
dat
'__________________________________________________________________________________
'
' The quad port driver
'
' The driver is organized as eight ligthweight threads that are cooperatively
' scheduled using the jmpret instruction. A jmpret R, S stores its PC + 1 at R
' and loads the PC from S. By organizing the threads in a circular fashion,
' we get Rx0 -> Tx0 -> Rx1 -> Tx1 ... Tx3 -> Rx0.
'
' Theoretical Performance
'
' By keeping the number of instructions between jmpret small, we increase the bit
' rate at which any given channel can operate. By keeping the number of instructions
' between jmpret identical, we reduce jitter.
'
' Receive Latency:
'       Awaiting start bit:     12 
'       Shifting in a byte:     32 
'       Shifting last bit:      36    <- Worst case while receiving 
'       Buffering a byte:       36-51 <- Worst case is 51 clocks 
'       Awaiting stop bit:      12 
'       After stop seen:        24 
'
' Transmit Latency:
'       Awaiting data:          20-35
'       Found data:             28-43
'       Extract buffered byte:  28-43
'       Maintain pointers:      36-51 <- Worst case is 51 clocks
'       Transmit first bit:     32
'       Check bit time:         20
'       Transmit other bits:    36    <- Worst case while transmitting
'       Transmit last bit:      32
'
' The worst case receive latency is a combination of the worst case receiver
' latency while receiving plus the worst case for all other channels regardless
' of what they're doing. Worst case transmit latency is computed using the same
' approach. It's a bit of work and a bit of coincidence that the paths are highly
' symmetric.
'
' Here's a summary of sustainable baud rate presuming full duplex operation
' of any active channels. 
'
' Channels                Worst Case Latency               Baud
' --------      ----------------------------------------- ------
'               Me +  Others  +  Idle Rx + Idle Tx
'     1         36 + (1 * 51) + (3 * 12) + (3 * 35) = 228 350877
'     2         36 + (3 * 51) + (2 * 12) + (2 * 35) = 283 282685
'     3         36 + (5 * 51) + (1 * 12) + (1 * 35) = 338 236686
'     4         36 + (7 * 51)                       = 393 203562 
'__________________________________________________________________________________
                        org     0
Serial_Driver
'
' Initialize the co-routine variables, direction register and output
' pin for each port and then simply flow into the first co-routine.
'
                        mov     Rx0_PC,#P0_Rx           ' Port 0 Initialization
                        mov     Tx0_PC,#P0_Tx
                        or      outa,P0_TxM             ' Start marking
                        or      dira,P0_TxM             ' Drive the TxD wire

                        mov     Rx1_PC,#P1_Rx           ' Port 1 Initialization 
                        mov     Tx1_PC,#P1_Tx
                        or      outa,P1_TxM             ' Start marking
                        or      dira,P1_TxM             ' Drive the TxD wire

                        mov     Rx2_PC,#P2_Rx           ' Port 2 Initialization 
                        mov     Tx2_PC,#P2_Tx
                        or      outa,P2_TxM             ' Start marking
                        or      dira,P2_TxM             ' Drive the TxD wire

                        mov     Rx3_PC,#P3_Rx           ' Port 3 Initialization 
                        mov     Tx3_PC,#P3_Tx
                        or      outa,P3_TxM             ' Start marking
                        or      dira,P3_TxM             ' Drive the TxD wire
'__________________________________________________________________________________
'
' Receiver for Port 0
'
P0_Rx                   mov     P0_Rx_BC,#8             ' We need to read 8 data bits
                        mov     P0_Rx_Timer,P0_SO       ' Get the sample offset

:wait_for_start         jmpret  Rx0_PC,Tx0_PC           ' Run some Transmit code and return here
                        test    P0_RxM,ina      wc      ' Move the RxD into Carry
                if_c    jmp     #:wait_for_start        ' Wait for the start bit to arrive

                        add     P0_Rx_Timer,cnt         ' Compute the sample time                          
:next_bit               add     P0_Rx_Timer,P0_TPB      ' Add one bit time
                        
:wait                   jmpret  Rx0_PC,Tx0_PC           ' Run some Transmit code and return here
                        mov     t1,P0_Rx_Timer          ' Get the sample time
                        sub     t1,cnt                  ' Compare to the current time
                        cmps    t1,#0           wc
                if_nc   jmp     #:wait                  ' And wait until it's time to sample

                        test    P0_RxM,ina      wc      ' Get RxD into Carry
                        rcr     P0_RxD,#1               ' Rotate it into our byte; we receive LSB first
                        djnz    P0_Rx_BC,#:next_bit

                        jmpret  Rx0_PC,Tx0_PC           ' Run some Transmit code and return here  
                        
                        shr     P0_RxD,#32-8            ' Position the byte into the LSBs
                        mov     t1,P0_RxB
                        add     t1,P0_Rx_II
                        wrbyte  P0_RxD,t1
                        add     P0_Rx_II,#1             ' and maintain the buffer structure
                        cmpsub  P0_Rx_II,P0_RxS
                        wrlong  P0_Rx_II,P0_Rx_II_Ref

:wait_for_stop          jmpret  Rx0_PC,Tx0_PC           ' Run some Transmit code and return here
                        test    P0_RxM,ina      wc      ' Move the RxD into Carry
                if_nc   jmp     #:wait_for_stop         ' Wait for the stop bit to arrive

                        jmp     #P0_Rx                  ' Go get the next byte
'
' Transmitter for Port 0
'
P0_Tx                   jmpret  Tx0_PC,Rx1_PC           ' Run some Receive code and return here
                        rdlong  t1,P0_Tx_II_Ref         ' Read the insertion index
                        cmp     t1,P0_Tx_EI     wz      ' Compare to the extraction index
                if_z    jmp     #P0_Tx                  ' If they are the same, the buffer is empty

                        mov     P0_TxD,P0_TxB
                        add     P0_TxD,P0_Tx_EI
                        jmpret  Tx0_PC,Rx1_PC           ' Run some Receive code and return here  

                        rdbyte  P0_TxD,P0_TxD           ' Read a byte of data
                        add     P0_Tx_EI,#1             ' Maintain the buffer structure
                        cmpsub  P0_Tx_EI,P0_TxS
                        wrlong  P0_Tx_EI,P0_Tx_EI_Ref     
                        
                        jmpret  Tx0_PC,Rx1_PC           ' Run some Receive code and return here  

                        or      P0_TxD,#%1_0000_0000    ' Add the Stop bit
                        add     P0_TxD,P0_TxD           ' Add the Start bit
                        mov     P0_Tx_BC,#10
                        mov     P0_Tx_Timer,cnt         ' Get the start of the frame

:next_bit               shr     P0_TxD,#1       wc      ' Get a data bit into carry
                        muxc    outa,P0_TxM             ' Drive the bit out
                        add     P0_Tx_Timer,P0_TPB      ' Compute the end-of-bit time

:wait                   jmpret  Tx0_PC,Rx1_PC           ' Run some Receive code and return here
                        mov     t1,P0_Tx_Timer          ' Get the end-of-bit time
                        sub     t1,cnt                  ' Subtract the current time
                        cmps    t1,#0           wc      ' Is it less than zero?
                if_nc   jmp     #:wait                  ' Wait for the end of this bit time

                        djnz    P0_Tx_BC,#:next_bit
                        jmp     #P0_Tx                  ' Transmit the next byte
'
' Receiver for Port 1
'
P1_Rx                   mov     P1_Rx_BC,#8             ' We need to read 8 data bits
                        mov     P1_Rx_Timer,P1_SO       ' Get the sample offset

:wait_for_start         jmpret  Rx1_PC,Tx1_PC           ' Run some Transmit code and return here
                        test    P1_RxM,ina      wc      ' Move the RxD into Carry
                if_c    jmp     #:wait_for_start        ' Wait for the start bit to arrive

                        add     P1_Rx_Timer,cnt         ' Compute the sample time                          
:next_bit               add     P1_Rx_Timer,P1_TPB      ' Add one bit time
                        
:wait                   jmpret  Rx1_PC,Tx1_PC           ' Run some Transmit code and return here
                        mov     t1,P1_Rx_Timer          ' Get the sample time
                        sub     t1,cnt                  ' Compare to the current time
                        cmps    t1,#0           wc
                if_nc   jmp     #:wait                  ' And wait until it's time to sample
        
                        test    P1_RxM,ina      wc      ' Get RxD into Carry
                        rcr     P1_RxD,#1               ' Rotate it into our byte; we receive LSB first
                        djnz    P1_Rx_BC,#:next_bit

                        jmpret  Rx1_PC,Tx1_PC           ' Run some Transmit code and return here  
                        
                        shr     P1_RxD,#32-8            ' Position the byte into the LSBs
                        mov     t1,P1_RxB
                        add     t1,P1_Rx_II
                        wrbyte  P1_RxD,t1
                        add     P1_Rx_II,#1             ' and maintain the buffer structure
                        cmpsub  P1_Rx_II,P1_RxS
                        wrlong  P1_Rx_II,P1_Rx_II_Ref

:wait_for_stop          jmpret  Rx1_PC,Tx1_PC           ' Run some Transmit code and return here
                        test    P1_RxM,ina      wc      ' Move the RxD into Carry
                if_nc   jmp     #:wait_for_stop         ' Wait for the stop bit to arrive

                        jmp     #P1_Rx                  ' Go get the next byte
'
' Transmitter for Port 1
'
P1_Tx                   jmpret  Tx1_PC,Rx2_PC           ' Run some Receive code and return here
                        rdlong  t1,P1_Tx_II_Ref         ' Read the insertion index
                        cmp     t1,P1_Tx_EI     wz      ' Compare to the extraction index
                if_z    jmp     #P1_Tx                  ' If they are the same, the buffer is empty

                        mov     P1_TxD,P1_TxB
                        add     P1_TxD,P1_Tx_EI
                        jmpret  Tx1_PC,Rx2_PC           ' Run some Receive code and return here  

                        rdbyte  P1_TxD,P1_TxD           ' Read a byte of data
                        add     P1_Tx_EI,#1             ' Maintain the buffer structure
                        cmpsub  P1_Tx_EI,P1_TxS
                        wrlong  P1_Tx_EI,P1_Tx_EI_Ref     
                        
                        jmpret  Tx1_PC,Rx2_PC           ' Run some Receive code and return here  

                        or      P1_TxD,#%1_0000_0000    ' Add the Stop bit
                        add     P1_TxD,P1_TxD           ' Add the Start bit
                        mov     P1_Tx_BC,#10
                        mov     P1_Tx_Timer,cnt         ' Get the start of the frame

:next_bit               shr     P1_TxD,#1       wc      ' Get a data bit into carry
                        muxc    outa,P1_TxM             ' Drive the bit out
                        add     P1_Tx_Timer,P1_TPB      ' Compute the end-of-bit time

:wait                   jmpret  Tx1_PC,Rx2_PC           ' Run some Receive code and return here
                        mov     t1,P1_Tx_Timer          ' Get the end-of-bit time
                        sub     t1,cnt                  ' Subtract the current time
                        cmps    t1,#0           wc      ' Is it less than zero?
                if_nc   jmp     #:wait                  ' Wait for the end of this bit time

                        djnz    P1_Tx_BC,#:next_bit                      
                        jmp     #P1_Tx                  ' Transmit the next byte
'
' Receiver for Port 2
'
P2_Rx                   mov     P2_Rx_BC,#8             ' We need to read 8 data bits
                        mov     P2_Rx_Timer,P2_SO       ' Get the sample offset

:wait_for_start         jmpret  Rx2_PC,Tx2_PC           ' Run some Transmit code and return here
                        test    P2_RxM,ina      wc      ' Move the RxD into Carry
                if_c    jmp     #:wait_for_start        ' Wait for the start bit to arrive

                        add     P2_Rx_Timer,cnt         ' Compute the sample time                          
:next_bit               add     P2_Rx_Timer,P2_TPB      ' Add one bit time
                        
:wait                   jmpret  Rx2_PC,Tx2_PC           ' Run some Transmit code and return here
                        mov     t1,P2_Rx_Timer          ' Get the sample time
                        sub     t1,cnt                  ' Compare to the current time
                        cmps    t1,#0           wc
                if_nc   jmp     #:wait                  ' And wait until it's time to sample
                                                        
                        test    P2_RxM,ina      wc      ' Get RxD into Carry
                        rcr     P2_RxD,#1               ' Rotate it into our byte; we receive LSB first
                        djnz    P2_Rx_BC,#:next_bit

                        jmpret  Rx2_PC,Tx2_PC           ' Run some Transmit code and return here  
                        
                        shr     P2_RxD,#32-8            ' Position the byte into the LSBs
                        mov     t1,P2_RxB
                        add     t1,P2_Rx_II
                        wrbyte  P2_RxD,t1
                        add     P2_Rx_II,#1             ' and maintain the buffer structure
                        cmpsub  P2_Rx_II,P2_RxS
                        wrlong  P2_Rx_II,P2_Rx_II_Ref

:wait_for_stop          jmpret  Rx2_PC,Tx2_PC           ' Run some Transmit code and return here
                        test    P2_RxM,ina      wc      ' Move the RxD into Carry
                if_nc   jmp     #:wait_for_stop         ' Wait for the stop bit to arrive

                        jmp     #P2_Rx                  ' Go get the next byte
'
' Transmitter for Port 2
'
P2_Tx                   jmpret  Tx2_PC,Rx3_PC           ' Run some Receive code and return here
                        rdlong  t1,P2_Tx_II_Ref         ' Read the insertion index
                        cmp     t1,P2_Tx_EI     wz      ' Compare to the extraction index
                if_z    jmp     #P2_Tx                  ' If they are the same, the buffer is empty

                        mov     P2_TxD,P2_TxB
                        add     P2_TxD,P2_Tx_EI
                        jmpret  Tx2_PC,Rx3_PC           ' Run some Receive code and return here  

                        rdbyte  P2_TxD,P2_TxD           ' Read a byte of data
                        add     P2_Tx_EI,#1             ' Maintain the buffer structure
                        cmpsub  P2_Tx_EI,P2_TxS 
                        wrlong  P2_Tx_EI,P2_Tx_EI_Ref     
                                                
                        jmpret  Tx2_PC,Rx3_PC           ' Run some Receive code and return here  

                        or      P2_TxD,#%1_0000_0000    ' Add the Stop bit
                        add     P2_TxD,P2_TxD           ' Add the Start bit
                        mov     P2_Tx_BC,#10
                        mov     P2_Tx_Timer,cnt         ' Get the start of the frame

:next_bit               shr     P2_TxD,#1       wc      ' Get a data bit into carry
                        muxc    outa,P2_TxM             ' Drive the bit out
                        add     P2_Tx_Timer,P2_TPB      ' Compute the end-of-bit time

:wait                   jmpret  Tx2_PC,Rx3_PC           ' Run some Receive code and return here
                        mov     t1,P2_Tx_Timer          ' Get the end-of-bit time
                        sub     t1,cnt                  ' Subtract the current time
                        cmps    t1,#0           wc      ' Is it less than zero?
                if_nc   jmp     #:wait                  ' Wait for the end of this bit time

                        djnz    P2_Tx_BC,#:next_bit                      
                        jmp     #P2_Tx                  ' Transmit the next byte
'
' Receiver for Port 3
'
P3_Rx                   mov     P3_Rx_BC,#8             ' We need to read 8 data bits
                        mov     P3_Rx_Timer,P3_SO       ' Get the sample offset

:wait_for_start         jmpret  Rx3_PC,Tx3_PC           ' Run some Transmit code and return here
                        test    P3_RxM,ina      wc      ' Move the RxD into Carry
                if_c    jmp     #:wait_for_start        ' Wait for the start bit to arrive

                        add     P3_Rx_Timer,cnt         ' Compute the sample time                          
:next_bit               add     P3_Rx_Timer,P3_TPB      ' Add one bit time
                        
:wait                   jmpret  Rx3_PC,Tx3_PC           ' Run some Transmit code and return here
                        mov     t1,P3_Rx_Timer          ' Get the sample time
                        sub     t1,cnt                  ' Compare to the current time
                        cmps    t1,#0           wc
                if_nc   jmp     #:wait                  ' And wait until it's time to sample

                        test    P3_RxM,ina      wc      ' Get RxD into Carry
                        rcr     P3_RxD,#1               ' Rotate it into our byte; we receive LSB first
                        djnz    P3_Rx_BC,#:next_bit

                        jmpret  Rx3_PC,Tx3_PC           ' Run some Transmit code and return here  
                        
                        shr     P3_RxD,#32-8            ' Position the byte into the LSBs
                        mov     t1,P3_RxB
                        add     t1,P3_Rx_II
                        wrbyte  P3_RxD,t1
                        add     P3_Rx_II,#1             ' and maintain the buffer structure
                        cmpsub  P3_Rx_II,P3_RxS
                        wrlong  P3_Rx_II,P3_Rx_II_Ref

:wait_for_stop          jmpret  Rx3_PC,Tx3_PC           ' Run some Transmit code and return here
                        test    P3_RxM,ina      wc      ' Move the RxD into Carry
                if_nc   jmp     #:wait_for_stop         ' Wait for the stop bit to arrive

                        jmp     #P3_Rx                  ' Go get the next byte
'
' Transmitter for Port 3
'

P3_Tx                   jmpret  Tx3_PC,Rx0_PC           ' Run some Receive code and return here
                        rdlong  t1,P3_Tx_II_Ref         ' Read the insertion index
                        cmp     t1,P3_Tx_EI     wz      ' Compare to the extraction index
                if_z    jmp     #P3_Tx                  ' If they are the same, the buffer is empty

                        mov     P3_TxD,P3_TxB
                        add     P3_TxD,P3_Tx_EI
                        jmpret  Tx3_PC,Rx0_PC           ' Run some Receive code and return here  

                        rdbyte  P3_TxD,P3_TxD           ' Read a byte of data
                        add     P3_Tx_EI,#1             ' Maintain the buffer structure
                        cmpsub  P3_Tx_EI,P3_TxS 
                        wrlong  P3_Tx_EI,P3_Tx_EI_Ref     
                        
                        jmpret  Tx3_PC,Rx0_PC           ' Run some Receive code and return here  

                        or      P3_TxD,#%1_0000_0000    ' Add the Stop bit
                        add     P3_TxD,P3_TxD           ' Add the Start bit
                        mov     P3_Tx_BC,#10
                        mov     P3_Tx_Timer,cnt         ' Get the start of the frame

:next_bit               shr     P3_TxD,#1       wc      ' Get a data bit into carry
                        muxc    outa,P3_TxM             ' Drive the bit out
                        add     P3_Tx_Timer,P3_TPB      ' Compute the end-of-bit time

:wait                   jmpret  Tx3_PC,Rx0_PC           ' Run some Receive code and return here
                        mov     t1,P3_Tx_Timer          ' Get the end-of-bit time
                        sub     t1,cnt                  ' Subtract the current time
                        cmps    t1,#0           wc      ' Is it less than zero?
                if_nc   jmp     #:wait                  ' Wait for the end of this bit time

                        djnz    P3_Tx_BC,#:next_bit
                        jmp     #P3_Tx                  ' Transmit the next byte            

'__________________________________________________________________________________
'
' Data declarations
Block_Clear_Start
'
' Note - Order Matters!
'
' The following declarations are grouped in quads, one for each port.
' This partial ordering must be maintained because the Spin code makes
' array references, indexing by Port. There are no dependencies among
' the various quads and they may be reordered at will provided those
' that get block cleared remain between the labels that designate that
' region. 
'
' The Insertion and Extraction Indices are handled differently for
' Transmit and Receive.
'
' For the transmitter, there is a local extraction index and an external
' insertion and extraction index. The external insertion index is always
' accessed throught the reference. The external extraction index is updated
' by reference whenever the internal extraction index is modified.
'
' For the receiver, there is local and an external insertion index. The
' external insertion index is updated whenever the local index is modified.
' The receiver neither knows nor cares about the extraction index and will
' overrun the buffer if the client does not keep up.
'
' The Index References are initialized explicitly because the dumb 
' compiler doesn't have any sort of relocation fixup!
'  
Rx_II_Ref               long
P0_Rx_II_Ref            long    0               ' Pointers to Rx insertion indices
P1_Rx_II_Ref            long    0
P2_Rx_II_Ref            long    0
P3_Rx_II_Ref            long    0

Tx_II_Ref               long
P0_Tx_II_Ref            long    0               ' Pointers to Tx insertion indices
P1_Tx_II_Ref            long    0
P2_Tx_II_Ref            long    0
P3_Tx_II_Ref            long    0

Tx_EI_Ref               long
P0_Tx_EI_Ref            long    0               ' Pointers to Tx extraction indices
P1_Tx_EI_Ref            long    0
P2_Tx_EI_Ref            long    0
P3_Tx_EI_Ref            long    0

Rx_II                   long
P0_Rx_II                long    0               ' Rx insertion indices
P1_Rx_II                long    0                           
P2_Rx_II                long    0
P3_Rx_II                long    0

Rx_EI                   long
P0_Rx_EI                long    0               ' Rx extraction indices
P1_Rx_EI                long    0 
P2_Rx_EI                long    0
P3_Rx_EI                long    0 

Tx_II                   long
P0_Tx_II                long    0               ' Tx insertion indices
P1_Tx_II                long    0
P2_Tx_II                long    0
P3_Tx_II                long    0

Tx_EI                   long
P0_Tx_EI                long    0               ' Tx extraction indices
P1_Tx_EI                long    0
P2_Tx_EI                long    0
P3_Tx_EI                long    0
 
RxB                     long
P0_RxB                  long    0               ' Pointers to Rx buffers
P1_RxB                  long    0
P2_RxB                  long    0
P3_RxB                  long    0

TxB                     long
P0_TxB                  long    0               ' Pointers to Tx buffers
P1_TxB                  long    0
P2_TxB                  long    0
P3_TxB                  long    0

TxS                     long
P0_TxS                  long    0               ' Sizes of Tx buffers
P1_TxS                  long    0
P2_TxS                  long    0
P3_TxS                  long    0

RxS                     long
P0_RxS                  long    0               ' Sizes of Rx buffers
P1_RxS                  long    0
P2_RxS                  long    0
P3_RxS                  long    0

RxM                     long
P0_RxM                  long    0               ' Rx pin masks
P1_RxM                  long    0
P2_RxM                  long    0
P3_RxM                  long    0

TxM                     long
P0_TxM                  long    0               ' Tx pin masks
P1_TxM                  long    0
P2_TxM                  long    0
P3_TxM                  long    0

TPB                     long
P0_TPB                  long    0               ' Ticks per bit
P1_TPB                  long    0
P2_TPB                  long    0
P3_TPB                  long    0

SO                      long
P0_SO                   long    0               ' Sample offsets in ticks
P1_SO                   long    0
P2_SO                   long    0
P3_SO                   long    0

Block_Clear_End
                        fit     288
'
' This storage is only used by the assembly language driver itself
'
' The order of these declarations is inconsequential 
'
P0_RxD                  res     1             
P0_Rx_BC                res     1
P0_Rx_Timer             res     1
P0_TxD                  res     1
P0_Tx_BC                res     1
P0_Tx_Timer             res     1

P1_RxD                  res     1
P1_Rx_BC                res     1
P1_Rx_Timer             res     1
P1_TxD                  res     1
P1_Tx_BC                res     1
P1_Tx_Timer             res     1

P2_RxD                  res     1
P2_Rx_BC                res     1
P2_Rx_Timer             res     1
P2_TxD                  res     1
P2_Tx_BC                res     1
P2_Tx_Timer             res     1

P3_RxD                  res     1
P3_Rx_BC                res     1
P3_Rx_Timer             res     1
P3_TxD                  res     1
P3_Tx_BC                res     1
P3_Tx_Timer             res     1

t1                      res     1
'
' These are the co-routine variables 
'
Rx0_PC                  res     1               ' The order of these declarations is inconsequential
Tx0_PC                  res     1
Rx1_PC                  res     1
Tx1_PC                  res     1
Rx2_PC                  res     1
Tx2_PC                  res     1
Rx3_PC                  res     1
Tx3_PC                  res     1

                        fit     321
                        