/*
  This file is part of the ELEV-8 Flight Controller Firmware
  for Parallax part #80204, Revision A
  
  Copyright 2015 Parallax Incorporated

  ELEV-8 Flight Controller Firmware is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  ELEV-8 Flight Controller Firmware is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the ELEV-8 Flight Controller Firmware.  If not, see <http://www.gnu.org/licenses/>.
  
  Written by Jason Dorie

  Prefs - User prefs storage for Elev8-FC
*/

#include <string.h>   // for memset()
#include <fdserial.h>

#include "eeprom.h"
#include "prefs.h"
#include "elev8-main.h" // for flight mode enum


PREFS Prefs;


int Prefs_Load(void)
{
  EEPROM::ToRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );    //Copy from EEPROM to DAT, address 32768

  int testChecksum = Prefs_CalculateChecksum( Prefs );
  if( testChecksum != Prefs.Checksum )
  {
    Prefs_SetDefaults();
    Prefs_Save();
    return 0;
  }
  return 1;
}

void Prefs_Save(void)
{
  Prefs.Checksum = Prefs_CalculateChecksum( Prefs );
  EEPROM::FromRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );  //Copy from DAT to EEPROM, address 32768
}

#define PI  3.141592654


void Prefs_SetDefaults(void)
{
  memset( &Prefs, 0, sizeof(Prefs) );

  Prefs.UseBattMon = 1;

  Prefs.RollCorrect[0] = 0.0f;                         //Sin of roll correction angle
  Prefs.RollCorrect[1] = 1.0f;                         //Cos of roll correction angle

  Prefs.PitchCorrect[0] = 0.0f;                        //Sin of pitch correction angle 
  Prefs.PitchCorrect[1] = 1.0f;                        //Cos of pitch correction angle 

  // MagOffsetX=0, MagScaleX=1, MagOffsetY=2, MagScaleY=3, MagOffsetZ=4, MagScaleZ=5;
  Prefs.MagScaleOfs[1] = 1024;
  Prefs.MagScaleOfs[3] = 1024;
  Prefs.MagScaleOfs[5] = 1024;

  Prefs.AutoLevelRollPitch =    (35.0f / 1024.0f) * (PI/180.0f) * 0.5f;          // 35 deg/ControlScale * Deg2Rad * HalfAngle
  Prefs.AutoLevelYawRate =    ((180.0f / 250.0f) / 1024.0f) * (PI/180.f) * 0.5f; // 180 deg/ControlScale / UpdateRate * Deg2Rad * HalfAngle
  Prefs.ManualRollPitchRate = ((120.0f / 250.0f) / 1024.0f) * (PI/180.f) * 0.5f;
  Prefs.ManualYawRate =       ((180.0f / 250.0f) / 1024.0f) * (PI/180.f) * 0.5f;


  Prefs.PitchGain = 127;
  Prefs.RollGain = 127;
  Prefs.YawGain = 127;      // Values are not allowed to be zero, so we use 0 to 255 to represent 1 to 256.  127 is middle - basline
  Prefs.AscentGain = 127;
  Prefs.AltiGain = 127;
  Prefs.PitchRollLocked = 1;

  Prefs.LowVoltageAlarmThreshold = 1050;

  Prefs.LowVoltageAlarm = 1;
  Prefs.LowVoltageAscentLimit = 0;
  Prefs.ThrottleTest = 1180 * 8;

  Prefs.MinThrottle = 1040 * 8;     // Values are in 1/8us resolution, full range is 1000us to 2000us, however many ESCs behave abnormally at the extremes
  Prefs.MaxThrottle = 1960 * 8;
  Prefs.CenterThrottle = 1500 * 8;
  Prefs.MinThrottleArmed = 1140 * 8;

  Prefs.ArmDelay = 250;
  Prefs.DisarmDelay = 125;

  Prefs.ThrustCorrectionScale = 256;  // 0 to 256  =  0 to 1
  Prefs.AccelCorrectionFilter = 16;   // 0 to 256  =  0 to 1

  Prefs.FlightMode[0] = FlightMode_Assist;
  Prefs.FlightMode[1] = FlightMode_Stable;
  Prefs.FlightMode[2] = FlightMode_Manual;
  Prefs.AccelCorrectionStrength = 96;

  Prefs.AileExpo = 0;
  Prefs.ElevExpo = 0;
  Prefs.RuddExpo = 0;

  Prefs.ThroChannel = 0;      //Standard radio channel mappings
  Prefs.AileChannel = 1;
  Prefs.ElevChannel = 2;
  Prefs.RuddChannel = 3;
  Prefs.GearChannel = 4;
  Prefs.Aux1Channel = 5;
  Prefs.Aux2Channel = 6;
  Prefs.Aux3Channel = 7;


  Prefs.ThroScale =  1024;
  Prefs.AileScale = -1024;
  Prefs.ElevScale =  1024;
  Prefs.RuddScale = -1024;
  Prefs.GearScale = -1024;
  Prefs.Aux1Scale =  1024;
  Prefs.Aux2Scale =  1024;
  Prefs.Aux3Scale =  1024;
}


int Prefs_CalculateChecksum(PREFS & PrefsStruct )
{
  unsigned int r = 0x55555555;            //Start with a strange, known value
  for( int i=0; i < (sizeof(PrefsStruct)/4)-1; i++ )
  {
    r = (r << 7) | (r >> (32-7));
    r = r ^ ((unsigned int*)&PrefsStruct)[i];     //Jumble the bits, XOR in the prefs value
  }    
  return (int)r;
}


/*
extern fdserial * dbg;

static int tGetC( void ) {
  return fdserial_rxChar( dbg );
}

static void tPutC( char c ) {
  fdserial_txChar( dbg, c );
}

static void tPutHexNibble( int x ) {
  if( x <= 9 ) {
    tPutC( x + '0' );
  }
  else {
    tPutC( x - 10 + 'a' );
  }        
}  

static void tPutHex( int x, int len ) {
  for( int i=len-1; i>=0; i-- ) {
    tPutHexNibble( (x>>(4*i)) & 15 );
  }    
}

void Prefs_Test( void )
{
  // This function exists only to test and validate the Load / Save / Checksum code
   
  tGetC();

  EEPROM::ToRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );    //Copy from EEPROM to DAT, address 32768

  int testCheck = Prefs_CalculateChecksum( Prefs );
  tPutC(0);
  tPutHex( Prefs.Checksum, 8 );
  tPutC(32);
  tPutHex( testCheck, 8 );
  tPutC(13);

  //EEPROM::FromRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );    //Copy from EEPROM to DAT, address 32768

  //tPutHex( Prefs.Checksum, 8 );
  //tPutC(32);
  //tPutHex( testCheck, 8 );
  //tPutC(13);
  //tPutC(13);


  Prefs_SetDefaults();
  testCheck = Prefs_CalculateChecksum( Prefs );
  tPutHex( testCheck, 8 );
  tPutC(13);

  Prefs_Save();

  EEPROM::ToRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );    //Copy from EEPROM to DAT, address 32768

  testCheck = Prefs_CalculateChecksum( Prefs );

  tPutHex( Prefs.Checksum, 8 );
  tPutC(32);
  tPutHex( testCheck, 8 );
  tPutC(13);

  tGetC();
}
*/
