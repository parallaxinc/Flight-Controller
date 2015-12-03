
#include "commlink.h"


u16 COMMLINK::cachedLength;
u16 COMMLINK::packetCrc;
u8  COMMLINK::packetBuf[64];
u8  COMMLINK::bufIndex;

u16 Checksum(u16 crc, u8 * buf , int len );


void COMMLINK::StartPacket( char port, u8 type , u16 length )
{
  cachedLength = length + 7;    // 2 byte signature, 1 byte type, 2 byte length, 2 byte crc

  u8 buf[5];
  buf[0] = 0x55;
  buf[1] = 0xAA;
  buf[2] = type;
  buf[3] = (cachedLength >> 8);
  buf[4] = cachedLength;

  packetCrc = Checksum( 0, buf, 5 );
  S4_Put_Bytes( port, buf, 5 );
}

void COMMLINK::AddPacketData( char port, void * data , u16 Count )
{
  packetCrc = Checksum( packetCrc, (u8*)data, Count );
  S4_Put_Bytes( port, data, Count );
}


void COMMLINK::StartPacket( u8 type , u16 length )
{
  cachedLength = length + 7;    // 2 byte signature, 1 byte type, 2 byte length, 2 byte crc
  
  packetBuf[0] = 0x55;
  packetBuf[1] = 0xAA;
  packetBuf[2] = type;
  packetBuf[3] = (cachedLength >> 8);
  packetBuf[4] = cachedLength;
  bufIndex = 5;
}

void COMMLINK::AddPacketData( void * data , u16 Count )
{
  memcpy( packetBuf + bufIndex , data, Count );
  bufIndex += Count;
}

void COMMLINK::EndPacket(void)
{
  packetCrc = Checksum( 0, packetBuf, bufIndex );
  memcpy( packetBuf + bufIndex, &packetCrc, 2 );
  bufIndex += 2;
}

void COMMLINK::BuildPacket( u8 type , void * data , u16 length )
{
  StartPacket( type , length );
  AddPacketData( data , length );
  EndPacket();
}



u16 Checksum(u16 crc, u8 * buf , int len )
{
  for( int i=0; i<len; i++) {
    crc = ((crc << 5) | (crc >> (16-5))) ^ buf[i];
  }
  return crc;
}
