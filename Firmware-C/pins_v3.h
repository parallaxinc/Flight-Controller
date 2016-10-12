
#ifndef __PINS_V3_H__
#define __PINS_V3_H__

/*
  This file is part of the ELEV-8 Flight Controller Firmware
  for Parallax part #80204, Revision A3 & B
  
  Copyright 2015-2016 Parallax Incorporated

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


#define PIN_RC_0  25
#define PIN_RC_1  26
#define PIN_RC_2  24
#define PIN_RC_3  27   //R/C input channel assignments (pin values are specified in the RC_Receiver object)
#define PIN_RC_4  0
#define PIN_RC_5  1
#define PIN_RC_6  2
#define PIN_RC_7  3 

#define PIN_PING  RC_7


  //Output pins to corresponding motors
#define PIN_MOTOR_FL  12
#define PIN_MOTOR_FR  13
#define PIN_MOTOR_BR  14
#define PIN_MOTOR_BL  15

#define PIN_MOTOR_AUX1  17
#define PIN_MOTOR_AUX2  18
#define PIN_EXP_TX      19
#define PIN_EXP_RX      20

#define PIN_VBATT  16


#define PIN_CS_ALT   10
#define PIN_CS_AG    6
#define PIN_SDO      7
#define PIN_SDI      8
#define PIN_SCL      9
#define PIN_CS_M     5
#define PIN_LED      4

#define PIN_BUZZER_1  11
#define PIN_BUZZER_2  11


#define XBEE_TX       21
#define XBEE_RX       22


#define PIN_RC_0_MASK  (1<<PIN_RC_0)
#define PIN_RC_1_MASK  (1<<PIN_RC_1)
#define PIN_RC_2_MASK  (1<<PIN_RC_2)
#define PIN_RC_3_MASK  (1<<PIN_RC_3)
#define PIN_RC_4_MASK  (1<<PIN_RC_4)
#define PIN_RC_5_MASK  (1<<PIN_RC_5)
#define PIN_RC_6_MASK  (1<<PIN_RC_6)
#define PIN_RC_7_MASK  (1<<PIN_RC_7)

#define PIN_RC_MASK    (PIN_RC_0_MASK | PIN_RC_1_MASK + PIN_RC_2_MASK | PIN_RC_3_MASK | PIN_RC_4_MASK | PIN_RC_5_MASK + PIN_RC_6_MASK | PIN_RC_7_MASK)


#endif
