#ifndef __DRIVERTABLE_H__
#define __DRIVERTABLE_H__

#include <propeller.h>

enum {
  DRV_F32,
  DRV_PPM,
  DRV_RC,
  DRV_RemoteRX,
  DRV_SBus,
  DRV_Sensors,
  DRV_Servo32,

  DRV_Mag_Init,
  DRV_Mag_AddSample,
  DRV_Mag_SetupIter,
  DRV_Mag_CalcIter,

  NumDrivers
};

struct DRIVER_ENTRY {
  unsigned short Offset;
  unsigned short Size;
};  

const unsigned short DriverTableStart = 0x9000;
const unsigned short DriverTableMagic = 0xE803;   // Elev8 V3, get it?  :)
const unsigned short DriverTableVersion = 0x01;   // Driver table V1

struct DRIVERS {
  unsigned short  MagicNumber;        // Verify the drivers have been uploaded
  unsigned short  Version;            // Drivers version?
  unsigned short  Count;              // Number of drivers in the buffer
  unsigned short  MaxSize;            // Largest data block in the driver table

  DRIVER_ENTRY    Table[NumDrivers];
};  

extern DRIVERS drivers;
extern int DriverBuffer[512];       // Reserved so the compiler properly reports free memory

void InitDrivers(void);
inline int GetDriverSize( int DriverIndex ) { return drivers.Table[DriverIndex].Size; }
void GetDriver( int DriverIndex , int * Buffer );

// Helper function to launch a driver
int StartDriver( int id, void * param );

#endif
