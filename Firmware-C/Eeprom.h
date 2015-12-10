#ifndef __EEPROM_H__
#define __EEPROM_H__  

/*
  Elev8 Flight Controller

  Copyright 2015 Parallax Inc

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

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
