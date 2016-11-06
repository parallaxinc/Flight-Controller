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

#include <propeller.h>

#include "constants.h"
#include "servo32_highres.h"
#include "drivertable.h"


struct ServoData {
  long FastPins, SlowPins;
  volatile long PingPin;
  long PingPinMask;
  long MasterLoopDelay, SlowUpdateCounter;
  long Cycles;
  long ServoData[32];		//Servo Pulse Width information
} Data;

//10 clocks is the smallest amount we can wait - everything else is based on that.
//If you're using a different clock speed, your center point will likely need to be adjusted
static const int Scale = 10;

void Servo32_Start(void)
{
  //use_cog_driver(servo32_highres_driver);
  //load_cog_driver(servo32_highres_driver, &Data);
  StartDriver( DRV_Servo32, &Data );
}


void Servo32_Init( int fastRate )
{
  memset(&Data, 0, sizeof(Data));
  Data.MasterLoopDelay = Const_ClockFreq / fastRate;
  Data.SlowUpdateCounter = (Const_ClockFreq / 50) / Data.MasterLoopDelay;
}


//Set a PIN index as a high-speed output (250Hz)  
void Servo32_AddFastPin( int Pin )
{
  Data.FastPins |= (1<<Pin);                                            
  Data.SlowPins |= (1<<Pin);	//Fast pins are also updated during the "slow" cycle                                              
}


//Set a PIN index as a standard-speed output (50Hz)  
void Servo32_AddSlowPin( int Pin )
{
  Data.SlowPins |= (1<<Pin);	//Slow pins are ONLY updated during the "slow" cycle
}

void Servo32_SetPingPin( int Pin )
{
  Data.PingPin = Pin;
  Data.PingPinMask = 1<<Pin;
}  


void Servo32_Set( int ServoPin, int Width )		// Set Servo value as a raw delay, in 10 clock increments
{
  Data.ServoData[ServoPin] = Width * Scale;		// Servo widths are set in 10ths of a uS, so 8000 = min, 12000 = mid, 16000 = max
}

int Servo32_GetPing(void)
{
  return Data.PingPin;    // This value stores the return value from the driver too
}  


/*
void Servo32_SetRC( int ServoPin, int Width )	// Set Servo value signed, assuming 12000 is your center
{
	Data.ServoData[ServoPin] = (12000 + Width) * Scale;	// Servo widths are set in 10ths of a uS, so -4000 min, 0 = mid, +4000 = max
}

int Servo32_GetCycles(void)
{
	return Data.Cycles;
}
*/
