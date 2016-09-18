
#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

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

//Constants
//Object used to specify cross-module constants, such as pin assigments, update rates, etc

#include <propeller.h>

#define Const_ClockFreq  80000000

#define Const_UpdateRate  200
#define Const_UpdateCycles (Const_ClockFreq / Const_UpdateRate)

#define Const_OneG  4096					//Must match the scale of the accelerometer
#define Const_Alti_UpdateRate  25		//Must match the update rate of the device  


#endif
