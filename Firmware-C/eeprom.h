#ifndef __EEPROM_H__
#define __EEPROM_H__  

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

class EEPROM
{
public:
	static void VarBackup(void * startAddr, void * endAddr);
	static void VarRestore(void * startAddr, void * endAddr);

	static void FromRam(void * startAddr, void * endAddr, int eeStart);
	static void ToRam(void * startAddr, void * endAddr, int eeStart);


//private:
	static void SetAddr(int addr);
	static void Poll(void);


	static void i2cStart(void);
	static int SendByte(unsigned char b);

	static int GetAck(void);
	static void i2cStop(void);
	static void i2cRelease(void);

	static unsigned char GetByte(void);
	static void SendAck(unsigned char ackbit);
};

#endif
