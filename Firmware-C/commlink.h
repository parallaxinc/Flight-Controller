#ifndef _COMMLINK_H___
#define _COMMLINK_H___

#include "propeller.h"
#include "elev8types.h"
#include "serial_4x.h"


// Elev8 packets are transmitted as follows:

// u16  : signature (0x55AA - used to resync in case of data loss)
// u8   : 0
// u8   : type   (packet types documented later)
// u16  : length (total number of data bytes, including header & crc)
// u8[N]: data bytes, 2 byte aligned
// u16  : 0x#### : checksum of entire packet, including signature, length, and data

class COMMLINK
{
public:

  // Length in THIS case is just the length of data you'll submit.  I'll add the rest.
  static void StartPacket( char port, u8 type , u16 length );
  static void AddPacketData( char port, void * data , u16 Count );   // Incrementally add packet data as you like

  static void EndPacket(char port) {                                 // Call this when the packet is finished to close it and send the CRC
      S4_Put_Bytes( port, &packetChecksum, 2 );
  }

  // Use these versions to buffer the data internally so you can send to more than one port more efficiently
  static void StartPacket( u8 type , u16 length );
  static void AddPacketData( void * data , u16 Count );   // Incrementally add packet data as you like
  static void EndPacket(void);                            // Call this when the packet is finished to close it and send the CRC

  static void BuildPacket( u8 type , void * data , u16 length );
  static void SendPacket( char port ) {  // Sends the pre-built packet to the port
      S4_Put_Bytes( port, packetBuf, bufIndex );
  }

private:
  static u16 cachedLength;
  static u16 packetChecksum;
  static u8  packetBuf[64];
  static u8  bufIndex;
};

#endif
