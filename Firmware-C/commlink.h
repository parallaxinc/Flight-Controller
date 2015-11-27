#ifndef _COMMLINK_H___
#define _COMMLINK_H___

#include "propeller.h"
#include "elev8types.h"


// Elev8 packets are transmitted as follows:

// u16  : signature (0x55AA - used to resync in case of data loss)
// u16  : length (total number of data bytes, including header & crc)
// u8   : type   (packet types documented later)
// u8[N]: data bytes
// u16  : 0x#### : crc of entire packet, including signature, length, and data

class COMMLINK
{
public:
  // Send a complete packet in a single call (unbuffered)
  void SendPacket( char port, u8 type , void * data , u16 length );

  // Length in THIS case is just the length of data you'll submit.  I'll add the rest.
  void StartPacket( char port, u8 type , u16 length );
  void AddPacketData( char port, void * data , u16 Count );   // Incrementally add packet data as you like
  void EndPacket(char port);                                  // Call this when the packet is finished to close it and send the CRC


  // Use these versions to buffer the data internally so you can send to more than one port more efficiently
  void StartPacket( u8 type , u16 length );
  void AddPacketData( void * data , u16 Count );   // Incrementally add packet data as you like
  void EndPacket(void);                            // Call this when the packet is finished to close it and send the CRC

  void BuildPacket( u8 type , void * data , u16 length );
  void SendPacket( char port );   // Sends the pre-built packet to the port

private:
  int cachedLength;
  u16 packetCrc;
  u8  packetBuf[64];
  int bufIndex;
};

#endif
