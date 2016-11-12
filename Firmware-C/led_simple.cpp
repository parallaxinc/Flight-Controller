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
#include "led_simple.h"

/*
  HUNDRED_nS  = _clkfreq / 10_000_000  'Number of clock cycles per 100 nanoseconds (8 @ 80MHz)                        
  ONE_uS      = HUNDRED_nS * 10 'Number of clock cycles per 1 microsecond (1000 nanoseconds)

'WS2812B Timings
  LED_0_HI    = (ONE_uS * 35)/100       
  LED_0_LO    = (ONE_uS * 90)/100       
  LED_1_HI    = (ONE_uS * 90)/100       
  LED_1_LO    = (ONE_uS * 35)/100       


'WS2812 Timings
'  LED_0_HI    = (ONE_uS * 35)/100       
'  LED_0_LO    = (ONE_uS * 80)/100       
'  LED_1_HI    = (ONE_uS * 70)/100       
'  LED_1_LO    = (ONE_uS * 60)/100       
*/


static struct DATA {
  int  ins[3];  //LEDPin, LEDAddr, LEDCount
} data;

static int cog;

void LED_Start( int _LEDPin, int _LEDAddr, int _LEDCount )
{
// Start driver - starts a cog
// returns false if no cog available
// may be called again to change settings
//
//   LEDPin  = pin connected to WS2812B LED array
//   LEDAddr = HUB address of RGB values for LED array (updated constantly)
//   LEDCount= Number of LED values to update  

	LED_Stop();

	data.ins[0] = _LEDPin;
	data.ins[1] = _LEDAddr;
	data.ins[2] = _LEDCount;

  use_cog_driver(led_driver);
  cog = load_cog_driver(led_driver, &data.ins[0]) + 1;
}


void LED_Stop(void)
{
// Stop driver - frees a cog
	if( cog ) {
		cogstop(cog - 1);
		cog = 0;
	}
}
