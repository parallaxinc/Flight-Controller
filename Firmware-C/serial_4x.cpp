
#include <Propeller.h>
#include "serial_4x.h"

//__________________________________________________________________________________
//   
//       Permission is hereby granted, free of charge, to any person obtaining a
//       copy of this software and associated documentation files (the "Software"),
//       to deal in the Software without restriction, including without limitation
//       the rights to use, copy, modify, merge, publish, distribute, sublicense,
//       and/or sell copies of the Software, and to permit persons to whom the
//       Software is furnished to do so, subject to the following conditions:                                                                   
//                                                                                                                              
//       The above copyright notice and this permission notice shall be included in
//       all copies or substantial portions of the Software.
//
//       THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//       IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//       FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//       AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//       LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//       FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//       IN THE SOFTWARE.
//
//       Portions of this work is derived in whole or in part from pre existing,
//       copyrighted work, subject to the "MIT License" terms and conditions,
//       each of which is described below:
//
//       Based in part upon full-duplex serial driver v1.01                                
//       Tracy Allen (TTA)   (c) 22-Jan-2011
//       Copyright (c) 2006 Parallax, Inc.                             *
//
//       Other attributions include, in alphabetic order:
//               Juergen Buchmueller
//               Duane Degn
//               Tim Moore
//____________________________________________________________________________________________
//
// Copyright (c) 2015 by Wayne Meretsky
//
// This version of the serial port driver was rewritten entirely from scratch in early
// 2015 with three goals, in order: Correctness, Performance, Code Size. While it is
// an entirely new implementation, it draws heavily on the design and implementation of
// many original and derived works that preceed it. 
//
// Tx and Rx pins are user defined and each port may use neither, either or both. The
// Baud rate is identical for Tx and Rx of a given port. Baud rates up to 460800K have
// been characterized and tested.
//
// To use the object:
//       1) Call Initialize
//       2) Call Define_Port for each port that will be used
//       3) Call Start
//       4) Make calls to any of the various input and output routines.
//       5) Call Stop.
//       6) Repeat as desired.
//
// Notes:
//       1) Associating the same pin with both Tx and Rx channels has undefined results
//       2) Rx and Tx buffers must not overlap and must be at least two bytes in size;
//          other than the limits of available memory there is no upper bound on their sizes 
//       3) Framing and overrun errors are not detected. Overruns of the receive buffer 
//          simply over write previously received data
//       4) Put operations, with the exception of Can_Put are blocking. They will not 
//          return until their data has been buffered for output. Put_Bytes buffers data
//          to the greatest extent possible and may be called with a Count that is greater
//          than the buffer size.
//       5) Flush_Output blocks until the last byte has been extracted from the buffer; but
//          that byte has not yet been transmitted at the time Flush returns
//       6) Get is a blocking operation; it will not return until the associated buffer has data.
//       7) Get_Bytes_Timed will not return until all requested data has arrived or the timer
//          has expired. It will not return partial data. Calling this routine with a Count
//          that is greater than the size of the receive buffer will never return any data
//
// Revision History
// 150220        Substantial rewrite including the following changes
//                 Improved calculation of TPB and SO
//                 Changed Can_Put to take a byte count
//                 Deleted Try_Put
//                 Modified Get_Timed to use Get_Bytes_Timed
//                 Modified Put_ZString to call Put_Bytes
//                 Modified Put_Bytes to use bytemove
//                 Modified Get_Bytes_Timed to use bytemove
//                 Reduced latency receiver
//                 Stopped shifting in stop bit
//                 Reduced latency of transmitter
//____________________________________________________________________________________________
//
// Data structures:
//
// The data structures used are not for the faint of heart. Worse yet, the Propeller
// architecture makes shared data structures both complicated to code and difficult
// to understand.
//
// The fundamental buffer structures are ring buffers designed for a single producer
// and a single consumer. They are inherently thread safe without the use of any kind
// of locking between producer and consumer. However, they are not tolerant of multiple
// producers or multiple consumers and any use requiring such must be serialized
// externally.
//
// The buffer storage is provided by the caller of Initialize. The buffers may be of any
// length greater than two bytes and as large as all available memory.
//
// Each channel (receive and transmit) has four variables used to control access. These are the
// Buffer Base Address, the Buffer Size (in bytes), the Insertion Index and the Extraction Index.
//
// Throughout, these things are called:
//   TxB   - Transmit Buffer Base
//   TxS   - Tranmsit Buffer Size
//   Tx_II - Transmit Buffer Insertion Index
//   Tx_EI - Tranmsit Buffer Extraction Index
//   RxB   - Receive Buffer Base
//   RxS   - Receive Buffer Size
//   Rx_II - Receive Buffer Insertion Index
//   Rx_EI - Receive Buffer Extraction Index
//
// If a channel is not in use, make sure its Pin No_Pin. There is no special code anywhere
// to make this work, it's simply a by product of the implementation.
//
// Insertion and Extraction indices are zero based and always in the range 0 .. Size - 1
//
// The Insertion Index is the index of the last inserted byte
// The Extraction Index is the index of the next byte available
// The Buffer is empty if the Insertion Index is equal to the Extraction Index
//
// The aforementioned buffer control variables are shared between the Spin and the Assembly
// language portions of the driver. The buffer base and buffer sizes are static and, thus,
// shared by copy. However, the Insertion and Extraction indicies are dynamic and must be
// shared by reference. Variables ending in _Ref and the pointers to the associated indicies
// so, for example, Tx_II_Ref must point to Tx_II. These reference are static and are shared
// by copy.


// This struct matches the buffer layout in the COG space
// Anything marked volatile is managed by the COG driver.  Non-volatile members are managed by this code

struct S4_COGVARS {
  volatile int*  Rx_II_Ref[4];             // Pointers to Rx insertion indices
  int*           Tx_II_Ref[4];             // Pointers to Tx insertion indices
  volatile int*  Tx_EI_Ref[4];             // Pointers to Tx extraction indices
  volatile int   Rx_II[4];                 // Rx insertion indices
  int            Rx_EI[4];                 // Rx extraction indices
  int            Tx_II[4];                 // Tx insertion indices
  volatile int   Tx_EI[4];                 // Tx extraction indices
  char*          RxB[4];                   // Pointers to Rx buffers
  char*          TxB[4];                   // Pointers to Tx buffers
  int            TxS[4];                   // Sizes of Tx buffers
  int            RxS[4];                   // Sizes of Rx buffers
  int            RxM[4];                   // Rx pin masks
  int            TxM[4];                   // Tx pin masks
  int            TPB[4];                   // Ticks per bit
  int            SO[4];                    // Sample offsets in ticks
};

static S4_COGVARS * cv;
static char cog;


void S4_Initialize(void)
{
  // reference the driver (pulls in the start/end symbols for the memory block)
  use_cog_driver(serial_4x_driver);

  // find the signature and assign the cv ptr to the next memory location
  uint32_t * driverMem = get_cog_driver(serial_4x_driver);
  int i=0;
  while( driverMem[i] != 0x12345678 )
    i++;

  cv = (S4_COGVARS *)(driverMem + i + 1);  
  
  memset( cv, 0, sizeof(S4_COGVARS) );

  // Despite being invariant, we cannot initialize the Index References
  // statically because compile-time references aren't fixed up by the linker

  for( int N = Port_First; N <= Port_Last; N++ )
  {
    cv->Rx_II_Ref[N] = &(cv->Rx_II[N]);
    cv->Tx_II_Ref[N] = &(cv->Tx_II[N]);
    cv->Tx_EI_Ref[N] = &(cv->Tx_EI[N]);
  }
}

void S4_Define_Port(char The_Port, int The_Baud, char The_TxP, char * The_TxB, char The_TxS, char The_RxP, char * The_RxB, char The_RxS)
{
int The_TPB;

  // Initialize and define the port, initializes the structures and launches
  // the assembly language portion of the driver. The parameters have the
  // following semantics:
  //
  // Cog   The Cog on which to run the driver
  // Baud  The bit rate of the Transmitter and Receiver expressed in Bits Per Second
  //       9600 means 9600 Baud. There is no requirement to use industry standard baud
  //       rates. All operations are with one start, eight data, no parity and one stop.     
  // TxP   Pin number of the pin on which data is to be Transmitted. Use No_Pin if no
  //       transmit functionality is desired.
  // TxB   The address of the transmit buffer
  // TxS   The size, in bytes, of the transmit buffer
  // RxP   Pin number of the pin on which data is to be received. Use No_Pin if no
  //       receive functionality is desired.
  // RxB   The address of the receive buffer
  // RxS   The size, in bytes, of the receive buffer

  if(The_RxP != No_Pin)
    cv->RxM[The_Port] = 1 << The_RxP;

  if(The_TxP != No_Pin)
    cv->TxM[The_Port] = 1 << The_TxP;

  // At high baud rates, there's not a lot of margin for the Shift Offset. In limited
  // testing, values of 5-28% work. 29% fails. 22% is used here. Rounding of both
  // Ticks-per-bit and the Shift Offset are critical for high data rates.

  The_TPB        = (CLKFREQ + (The_Baud >> 1)) / The_Baud;
  cv->TPB[The_Port] = The_TPB;
  cv->SO [The_Port] = ((The_TPB * 22) + 50) / 100;

  cv->TxB[The_Port] = The_TxB;
  cv->TxS[The_Port] = The_TxS;
  cv->RxB[The_Port] = The_RxB;
  cv->RxS[The_Port] = The_RxS;
}


void S4_Start(void)
{
  use_cog_driver(serial_4x_driver);
  cog = load_cog_driver(serial_4x_driver, 0);
}

void S4_Stop(void)
{
  if(cog) {
    cogstop(cog - 1);
    cog = 0;
  }
}


//__________________________________________________________________________________
//
// The Transmit Primitives
//
// Flush_Output, Put, Try_Put, Put_Bytes, Put_ZString
//
void S4_Flush_Output(char The_Port)
{
  while (cv->Tx_EI[The_Port] != cv->Tx_II[The_Port])
    ;
}

void S4_Put(char The_Port, char The_Byte)
{
  // Waits until buffer space is available and then buffers The_Byte for transmit
  int I = cv->Tx_II[The_Port];
  int N = (I + 1) % cv->TxS[The_Port];

  while( cv->Tx_EI[The_Port] == N )                  // Wait for room
    ;

  cv->TxB[The_Port][I] = The_Byte;
  cv->Tx_II[The_Port] = N;
}


void S4_Put_Unsafe(char The_Port, char The_Byte)
{
  // DOES NOT WAIT until buffer space is available - buffers The_Byte for transmit
  int I = cv->Tx_II[The_Port];
  int N = (I + 1) % cv->TxS[The_Port];
  cv->TxB[The_Port][I] = The_Byte;
  cv->Tx_II[The_Port] = N;
}


char S4_Can_Put(char The_Port, int The_Count)
{
  int BC = cv->Tx_EI[The_Port] - cv->Tx_II[The_Port] - 1;
  if(BC <= 0)
    BC = BC + cv->TxS[The_Port];

  return(BC >= The_Count);
}


static int min( int a, int b ) { return a < b ? a : b; }



void S4_Put_Bytes(char The_Port, void * The_Bytes, int The_Count)
{
  char * bytes = (char *)The_Bytes;
  int Size = cv->TxS[The_Port];                        // Get the size
  char *B = cv->TxB[The_Port];                         // Get the buffer address

  while (The_Count != 0)
  {
    int I = cv->Tx_II[The_Port];                       // I = insertion point
    int M = I - cv->Tx_EI[The_Port];                   // M = Bytes already buffered
    if (M < 0)
      M += Size;

    M = min((Size - M - 1), The_Count);                // M = Byte to move into the buffer now

    int C = min((Size - I), M);                        // C = bytes to move into the end of the buffer
    if( C ) {
      memcpy(B + I, bytes,     C);                     // Fill to the end of the buffer
    }      
    if( M-C ) {
      memcpy(B,     bytes + C, M - C);                 // Fill the head of the buffer - might be nothing
    }      
    cv->Tx_II[The_Port] = (I + M) % Size;              // Update the insertion point       

    bytes += M;                                        // Bump address
    The_Count -= M;                                    // decrement the bytes remaining
  }
}

//__________________________________________________________________________________
//
// The Receive primitives
//
//
// Expunge_Input, Peek, Check, Get, Get_Timed, Get_Bytes_Timed
//
// Note that Get and Get_Timed are optimized for code size, not performance.
//
void S4_Expunge_Input(char The_Port)
{
  cv->Rx_EI[The_Port] = cv->Rx_II[The_Port];
}

int S4_Peek(char The_Port)
{
  // Checks to see if a character is sitting in the Receive buffer If so, the character
  // is returned to the caller.  If no character is available, a value of -1 is returned.
  //
  // In no case is the character removed from the buffer. A subsequent call to Get, for
  // example, is required to debuffer the character.
  //
  int E = cv->Rx_EI[The_Port];
  if( E == cv->Rx_II[The_Port] )
    return -1;

  return cv->RxB[The_Port][E];
}

int S4_Check(char The_Port)
{
// Checks to see if a character is sitting in the Receive buffer of
// The_Port.  If so, the character is returned to the caller.  If no
// character is available, a value of -1 is returned.
//
  int E = cv->Rx_EI[The_Port];

  if( E == cv->Rx_II[The_Port] )
    return -1;
    
  char B = cv->RxB[The_Port][E];
  cv->Rx_EI[The_Port] = (E + 1) % cv->RxS[The_Port];

  return B;
}

char S4_Get(char The_Port)
{
//
// Waits until a byte is available and then returns it to the caller
//
  int B;
  do {
    B = S4_Check(The_Port);
  } while( B < 0 );
  return B;
}
  
int S4_Get_Timed(char The_Port, int MS_Timer)
{
  char B = 0;
  if( !S4_Get_Bytes_Timed(The_Port, &B, 1, MS_Timer) )
    return -1;

  return B;
}


char S4_Get_Bytes_Timed(char The_Port, char * The_Buffer, int The_Count, int MS_Timer)
{
//
// Waits for up to MS_Delay for a The_Count bytes to arrive and copies them to
// The_Buffer, returning True.  If The_Timer elapses prior to their arrival,
// The_Buffer is unaltered and the return value is False
//
// No data is removed from the Receive Buffer unless it is all available
//
  int Stamp = CNT;

  int   E = cv->Rx_EI[The_Port];                       // Get the extraction index
  int   S = cv->RxS  [The_Port];                       // And the size
  char *B = cv->RxB  [The_Port];                       // And the buffer base

  do {
    int C = cv->Rx_II[The_Port] - E;                   // Compute bytes in buffer

    if (C < 0)
      C = C + S;

    if( C >= The_Count )                               // Is it all here yet?
    {
      C = min( (S - E) , The_Count );                  // Determine bytes to end of buffer
      memcpy( The_Buffer,     B + E, C);               // Move first part
      memcpy( The_Buffer + C, B,     The_Count - C);   // Move the rest, often none
      cv->Rx_EI[The_Port] = (E + The_Count) % S;       // Update the extraction index 

      return 1;
    }
  } while( ! (((CNT - Stamp) / (CLKFREQ / 1000)) > MS_Timer)  );

  return 0;
}
