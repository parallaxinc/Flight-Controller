#ifndef __ELEV8_MAIN_H__
#define __ELEV8_MAIN_H__

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
*/

void Initialize(void);
void InitReceiver(void);
void InitSerial(void);
void FindGyroZero(void);
void UpdateFlightLoop(void);
void UpdateFlightLEDColor(void);
void ArmFlightMode(void);
void DisarmFlightMode(void);
void StartCompassCalibrate(void);
void DoCompassCalibrate(void);
void CheckDebugInput(void);
void DoDebugModeOutput(void);
void InitializePrefs(void);
void ApplyPrefs(void);
void All_LED( int Color );


 // ESC output array indices for corresponding motors
#define   OUT_FL  0
#define   OUT_FR  1
#define   OUT_BR  2
#define   OUT_BL  3


#define EXTRA_LIGHTS

#if defined(EXTRA_LIGHTS)
#define LED_COUNT (1 + 20 + 16)    // Add-on light kit
#else
#define LED_COUNT 2   // basic - one additional status LED automatic
#endif



enum MODE {
  MODE_None = 0,
  MODE_SensorTest = 2,
  MODE_MotorTest = 3,
};

enum FLIGHTMODE {
  FlightMode_Assist = 0,          // Auto level, heading hold, altitude hold
  FlightMode_Stable = 1,          // Auto level, heading hold
  FlightMode_Manual = 2,          // Gyro stabilized, not auto level
  FlightMode_CalibrateCompass = 3,
};

// Structure to hold radio values to make sure they stay in order
struct RADIO {
  short Thro, Aile, Elev, Rudd, Gear, Aux1, Aux2, Aux3, Aux4;   // Aux4 is an additional raw channel for SBUS users only

  short & Channel(int i) { return (&Thro)[i]; }
};


//LED Brightness values - AND with color values to dim them
const int LED_Full    = 0xffffff;
const int LED_Half    = 0x7f7f7f;
const int LED_Quarter = 0x3f3f3f;
const int LED_Eighth  = 0x1f1f1f;
const int LED_Dim     = 0x0f0f0f;


//LED Color values
const int LED_Red   = 0x00FF00;
const int LED_Green = 0xFF0000;
const int LED_Blue  = 0x0000FF;
const int LED_White = 0xFFFFFF;
const int LED_Yellow = LED_Red | LED_Green;
const int LED_Violet = LED_Red | LED_Blue;
const int LED_Cyan =   LED_Blue | LED_Green;

const int LED_DimCyan = (LED_Blue | LED_Green) & LED_Half;
const int LED_DimWhite = LED_White & LED_Half;

#define COMMAND(a,b,c,d) (int)((a<<24) | (b<<16) | (c<<8) | (d<<0) )

#define Comm_Elv8       COMMAND('E','l','v','8')
#define Comm_Beat       COMMAND('B','E','A','T')
#define Comm_QueryPrefs COMMAND('Q','P','R','F')
#define Comm_SetPrefs   COMMAND('U','P','r','f')
#define Comm_Wipe       COMMAND('W','I','P','E')

#define Comm_ZeroGyro   COMMAND('Z','r','G','r')
#define Comm_ZeroAccel  COMMAND('Z','e','A','c')
#define Comm_ResetGyro  COMMAND('R','G','y','r')
#define Comm_ResetAccel COMMAND('R','A','c','l')
#define Comm_ResetRadio COMMAND('R','r','a','d')

#define Comm_Motor1     COMMAND('M','1','t','1')    // Motor test values are duplicated - very unlikely to get set this way by noise
#define Comm_Motor2     COMMAND('M','2','t','2')
#define Comm_Motor3     COMMAND('M','3','t','3')
#define Comm_Motor4     COMMAND('M','4','t','4')
#define Comm_Motor5     COMMAND('M','5','t','5')
#define Comm_Motor6     COMMAND('M','6','t','6')
#define Comm_Motor7     COMMAND('M','7','t','7')

#endif
