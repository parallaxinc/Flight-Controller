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
#include "sbus.h"

static char Cog;

static struct DATA {
  long  InputMask;
  long  BaudDelay;
  short Channels[18];  //Last two channels are digital
} data;


void SBUS::Start( int InputPin )
{
  data.InputMask = 1 << InputPin;
  data.BaudDelay = Const_ClockFreq / 100000;                        // SBUS is 100,000 bps 

  data.Channels[0] = 0;          // Throttle is zero'd
  for( int i=1; i<18; i++ ) {
    data.Channels[i] = 1024;     // All other channels are centered
  }

  use_cog_driver(sbus_driver);
  Cog = load_cog_driver(sbus_driver, &data) + 1;
}


void SBUS::Stop(void)
{
	// Stop driver and release cog
  if(Cog) {
    cogstop(Cog - 1);
    Cog = 0;
  }
}

short SBUS::GetRC( int i ) {
  return data.Channels[i] - 1024;
}
