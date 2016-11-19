/*

  gps.h - GPS support for the Elev8-V3

*/

#ifndef __ELEV8_GPS_H_
#define __ELEV8_GPS_H_

#define GPS
#ifdef GPS

#include "elev8types.h"

extern char SatCount;
extern int Latitude;
extern int Longitude;
extern int Dilution;
extern int GpsAltitude;

//void SendGPSConfig( u8 * data, u8 count );
void SetGPSBaud(void);
void StartGPSThread(void);

#endif  // GPS
#endif  // file include guard
