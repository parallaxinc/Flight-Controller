/*
  This file is part of the ELEV-8 Flight Controller Firmware
  for Parallax part #80204, Revision A
  Version 1.0.2
  
  Copyright 2015 Parallax Incorporated

  ELEV-8 Flight Controller Firmware is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  ELEV-8 Flight Controller Firmware is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the ELEV-8 Flight Controller Firmware.  If not, see <http://www.gnu.org/licenses/>.
  
 
  Actively In Development / To Be Developed:
  - Altitude Hold - currently disabled
  - Heading Hold & Compass Calibratin 
  
  
  Written by Jason Dorie

  Dedicated to the memory of my father, Jim Dorie, who encouraged me endlessly
*/

#include <propeller.h>

#include "constants.h"          // Project-wide constants, like clock rate, update frequency
#include "pins.h"               // Pin assignments for the hardware
#include "sensors.h"            // Sensors (gyro,accel,mag,baro) + LEDs driver                  (1 COG)
#include "serial_4x.h"          // 4 port simultaneous serial I/O                               (1 COG)


short UsbPulse = 0;     // Periodically, the GroundStation will ping the FC to say it's still there - these are countdowns
#define LED_COUNT  2


//Sensor inputs, in order of outputs from the Sensors cog, so they can be bulk copied for speed
static SENS sens;


//Debug output mode, working variables  
static long  counter = 0;    //Main loop iteration counter
static long  LEDValue[LED_COUNT];           //LED outputs (copied to the LEDs by the Sensors cog)

static long  loopTimer;                     //Master flight loop counter - used to keep a steady update rate



void Initialize(void);
void InitSerial(void);
void All_LED( int );



static void LogInt( int x )
{
  char buf[14];
  char index = 13;
  buf[index] = ' ';   // Add the space here for speed
  char isNeg;
  if( x < 0 ) {
    isNeg = 1;
    x = -x;
  }    
  else isNeg = 0;

  do {
    buf[--index] = '0' + (x % 10);
    x /= 10;
  } while(x > 0);

  if( isNeg ) {
    buf[--index] = '-';
  }    

  char count = 14-index;

  S4_Put_Bytes( 0, buf+index, count );
}


int main()                                    // Main function
{
  All_LED( 0 );
  InitSerial();

  Sensors_Start( PIN_SDI, PIN_SDO, PIN_SCL, PIN_CS_AG, PIN_CS_M, PIN_CS_ALT, PIN_LED, (int)&LEDValue[0], LED_COUNT );

  loopTimer = CNT;
  int filter = 256;

  while(1)
  {
    //Read ALL inputs from the sensors into local memory, starting at Temperature
    memcpy( &sens, Sensors_Address(), Sensors_ParamsSize );


    S4_Put( 0, '$' );   // Send a $ as a signature (like a packet start)

    LogInt( sens.GyroY );    // Sends an integer, followed by a space
    LogInt( sens.AccelX );   // Sends an integer, followed by a space
    LogInt( sens.AccelZ );   // Sends an integer, followed by a space

    S4_Put( 0, 13 );    // Send a carriage return


    ++counter;
    loopTimer += 80000000 / 100;

    waitcnt( loopTimer );
  }
}




static char RXBuf1[32], TXBuf1[64];
static char RXBuf2[32], TXBuf2[64];
static char RXBuf3[4],  TXBuf3[4]; // GPS?
static char RXBuf4[4],  TXBuf4[4]; // Data Logger


void InitSerial(void)
{
  S4_Initialize();

  S4_Define_Port(0, 115200,      30, TXBuf1, sizeof(TXBuf1),      31, RXBuf1, sizeof(RXBuf1));
  S4_Define_Port(1,  57600, XBEE_TX, TXBuf2, sizeof(TXBuf2), XBEE_RX, RXBuf2, sizeof(RXBuf2));

  // Unused ports get a pin value of 32
  S4_Define_Port(2,   9600, 32, TXBuf3, sizeof(TXBuf3), 32, RXBuf3, sizeof(RXBuf3));

  S4_Define_Port(3, 115200, PIN_MOTOR_AUX2, TXBuf4, sizeof(TXBuf4), 32, RXBuf4, sizeof(RXBuf4));

  S4_Start();
}


void All_LED( int Color )
{
  for( int i=0; i<LED_COUNT; i++ )
    LEDValue[i] = Color;
}  

