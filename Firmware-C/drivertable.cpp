
#include <propeller.h>

#include "drivertable.h"
#include "eeprom.h"
#include "beep.h"
#include "serial_4x.h"

DRIVERS drivers;

int DriverBuffer[512];  // Globally reserved, but could be used for random stuff when not uploading a driver


void InitDrivers(void)
{
  memset(&drivers, 0, sizeof(drivers));
  EEPROM::ToRam( 	&drivers, ((char *)&drivers) + sizeof(drivers)-1, DriverTableStart );

  if( drivers.MagicNumber != DriverTableMagic ) {
    while( true ) {
      Beep();   // No drivers installed
      S4_Put(0, 'D');
      waitcnt( CNT + 10000000 );
    }      
  }

  if( drivers.Version != DriverTableVersion || drivers.Count != NumDrivers ) {
    while( true ) {
      Beep2();  // Wrong drivers version installed
      S4_Put(0, 'V');
      waitcnt( CNT + 10000000 );
    }      
  }
}

void GetDriver( int DriverIndex , int * Buffer )
{
  int size = drivers.Table[DriverIndex].Size;
  int driverAddr = drivers.Table[DriverIndex].Offset;

  EEPROM::ToRam( 	Buffer, ((char *)Buffer) + size-1, driverAddr );
}

// Helper to launch a driver
int StartDriver( int id, void * param )
{
  GetDriver( id, DriverBuffer );
  int cog = cognew(DriverBuffer, param) + 1;
  waitcnt( CNT + 512 * 16 );
  return cog;
}
