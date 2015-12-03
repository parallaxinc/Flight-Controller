
#ifndef __SERIAL_4X_H__
#define __SERIAL_4X_H__

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
//

const int No_Pin  = 32;           // Valid pins are 0-31; this could be any other value

const int Port_First      = 0;
const int Port_Last       = 3;
const int Ports           = Port_Last - Port_First + 1;


void S4_Initialize(void);


// S4_Define_Port:

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

void S4_Define_Port(char The_Port, int The_Baud, char The_TxP, char * The_TxB, char The_TxS, char The_RxP, char * The_RxB, char The_RxS);
    
void S4_Start(void);
void S4_Stop(void);


// The Transmit Primitives
//
// Flush_Output, Put, Try_Put, Put_Bytes, Put_ZString
//
void S4_Flush_Output(char The_Port);

void S4_Put(char The_Port, char The_Byte);
void S4_Put_Unsafe(char The_Port, char The_Byte);
char S4_Can_Put(char The_Port, char The_Count);
void S4_Put_Bytes(char The_Port, void * The_Bytes, int The_Count);


// The Receive primitives
//
// Expunge_Input, Peek, Check, Get, Get_Timed, Get_Bytes_Timed
//
void S4_Expunge_Input(char The_Port);
int  S4_Peek(char The_Port);

int  S4_Check(char The_Port);

char S4_Get(char The_Port);
int  S4_Get_Timed(char The_Port, int MS_Timer);
char S4_Get_Bytes_Timed(char The_Port, char * The_Buffer, int The_Count, int MS_Timer);


#endif
