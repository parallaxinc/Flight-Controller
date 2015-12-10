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

  This file was originally taken from the Propeller Education Kit Lab, and ported to C++
*/

#include <propeller.h>
#include "eeprom.h"

/*
File: Propeller Eeprom.spin
Version: 0.6

Developed for forthcoming Propeller Education Kit Lab: EEPROM Datalogging and I2C

See Propeller Eeprom Docs.spin for explanation and instructions.  To view this
object, press, and the file should appear in the Object Info window's Object Explorer
pane.  Double-click it to open.

Note: Greg Glenn -  Added i2cRelease EEPROM::function to fully release bus so other
                    devices can use it (gyro).  Changed all calls of i2sStop to i2cRelease

  
This object simplifies writing to and reading from the Propeller chip's 24LC256 EEPROM
program memory.  This object also has methods that can be used to back up sections of
RAM so that they are automatically reloaded into RAM during reboot.

EXAMPLES:

  In addition to the examples below, see Test Propeller Eeprom...spin

  OBJ
    eeprom : "Propeller Eeprom"
   
  ...

  VAR 
    long value[31]
    word a, b, c, d
    byte e, f, g, mybytes[20]
   
  ...

  PUB
   
    'Copy everything from value[0]..mybytes[20] from RAM to the same addresses in EEPROM.
    'These values will automatically be loaded back into RAM in the event of a reboot.  
    EEPROM::VarBackup(&value, &myBytes[20])         //Copy from RAM to EEPROM
   
    ...
   
    'Restore a previous snapshot of variables in main RAM.
    EEPROM::VarRestore(&value, &mybytes[20])
   
    ...
   
    'Copy a snapshot of just the values long array to the same addresses in EEPROM. Since
    'these are long variables, 3 has to be added to address passed to the FromRam method's
    'endAddr parameter to account for the three bytes that follow the long's address. 
    EEPROM::VarBackup(&value, &value[31] + 3)
   
    ...
   
    'Copy all the variable contents from the start of the value array through the end of
    'the mybytes array to addresses 20_000..20_158 in EEPROM.
    EEPROM::FromRam(&value, &mybytes[20], 20_000)
   
    ...
   
    'Copy only the longs and words back to RAM variables from EEPROM
    'addresses 20_000..20_135.
    EEPROM::ToRam(&value, &d + 1, 20_000)
   
NOTES:

(1) The endAddr parameters are byte addresses.  If you use &myLastLongAddress, make sure
to add 3 to the address since it's pointing to the first of the four bytes that make up
the long.  Likewise, if you use &myLastWordAddress, add 1 to account for the two bytes
that make up the word.  Byte addresses do not need to be adjusted.

(2) Regardless of the order you declare the variables, the Spin compiler sets aside longs
first, then words, then bytes. If you are backing up or restoring sections of main RAM
that span more than one variable type with the VarBackup or VarRestore methods, make sure 
to declare all your variables in this order: long, word, byte.  Multiple method calls can
also be made to back up specific sections if variables have to be declared in groups
of different sizes that are not contiguous.  

(3) Programs are stored in EEPROM and RAM, starting at the smallest byte address and
building toward larger addresses.  So, datalogging programs that use EEPROM for storage
of accumulated measurements should start at the largest address and build toward smaller
addresses.

(4) BEWARE: Downloading a program overwrites ALL EEPROM!  Your datalogging application
needs to have a built-in way of uploading the data after the measurements have been
taken because a separate program will overwrite all the collected data.
*/


  //24LC256 EEPROM constants

const int SDAPIN = 29;                                 //P29 = data line
const int SCLPIN = 28;                                 //P28 = clock line

const int SDA = (1<<SDAPIN);                           //P29 = data line
const int SCL = (1<<SCLPIN);                           //P28 = clock line
const int ACK = 0;                                     //Acknowledge bit = 0
const int NACK = 1;                                    //(No) acknowledge bit = 1


void EEPROM::VarBackup(void * startAddr, void * endAddr)
{
  //Copy contents of address range defined by startAddr..endAddr from main RAM to EEPROM.

  FromRam(startAddr, endAddr, (int)startAddr);			 //Pass addresses to the Write method
}


void EEPROM::VarRestore(void * startAddr, void * endAddr)
{
  //Copy contents of address range defined by startAddr..endAddr from EEPROM to RAM.

  ToRam(startAddr, endAddr, (int)startAddr);             //Pass addresses to the read method
}


void EEPROM::FromRam(void * startAddr, void * endAddr, int eeStart)
{
  unsigned char * addr;
  unsigned char * page;

  //Copy startAddr..endAddr from main RAM to EEPROM beginning at eeStart address.

  addr = (unsigned char *)startAddr;             //Initialize main RAM index
  int eeAddr = eeStart;                          //Initialize EEPROM index

  do {
    page = addr+64 - eeAddr%64;                  //Find next EEPROM page boundary
    if( page > (unsigned char*)endAddr+1) {
      page = (unsigned char *)endAddr+1;
    }     

    SetAddr(eeAddr);                             //Give EEPROM starting address
    do {                                         //Bytes -> EEPROM until page boundary
      SendByte(*addr++);
    } while(addr != page);

    i2cRelease();                                //From 24LC256's page buffer -> EEPROM
    eeAddr = (int)addr - (int)startAddr+eeStart; //Next EEPROM starting address
  } while( addr <= endAddr );                    //Quit when RAM index > end address
}


void EEPROM::ToRam(void * startAddr, void * endAddr, int eeStart)
{
  //Copy from EEPROM beginning at eeStart address to startAddr..endAddr in main RAM.

  unsigned char * addr;
  
  SetAddr(eeStart);                              //Set EEPROM's address pointer 
  i2cStart();
  SendByte(0xA1);                                //EEPROM I2C address + read operation
  
  if(startAddr == endAddr) {
    addr = (unsigned char *)startAddr;
  }
  else {
    addr = (unsigned char *)startAddr;
    while( addr != ((unsigned char *)endAddr) )  //Main RAM index startAddr to endAddr
    {
      *addr++ = GetByte();                       //GetByte byte from EEPROM & copy to RAM 
      SendAck(ACK);                              //Acknowledge byte received
    }
  }
  *addr = GetByte();                             //GetByte byte from EEPROM & copy to RAM 
  SendAck(NACK);
  i2cRelease();                                  //Stop sequential read
}

void EEPROM::SetAddr(int addr)
{
  //Sets EEPROM internal address pointer.

  Poll();                                        //Poll until EEPROM acknowledges
  SendByte(addr >> 8);                           //Send address high byte
  SendByte(addr);                                //Send address low byte
}

void EEPROM::Poll(void)
{
  //Poll until acknowledge.  This is especially important if the 24LC256 is copying from
  //buffer to EEPROM.

  int ackbit = 1;
  do {                                           //Send/check acknowledge loop
    i2cStart();                                  //Send I2C start condition
    ackbit = SendByte(0xA0);                     //Write command with EEPROM's address
  } while(ackbit);                               //Repeat while acknowledge is not 0
}


#define Set( var, mask )   { var |= mask; }
#define Clear( var, mask ) { var &= ~mask; }


void EEPROM::i2cStart(void)
{
  //I2C start condition.  SDA transitions from high to low while the clock is high.
  //SCL does not have the pullup resistor called for in the I2C protocol, so it has to be
  //set high. (It can't just be set to inSendByte because the resistor won't pull it up.)

  Set(OUTA, SDA);                                //Set SDA pin high
  Set(OUTA, SCL);                                //SCL pin outSendByte-high
  Set(DIRA, SCL);
  Clear(DIRA, SDA);                              //Let pulled up SDA pin go high
  Clear(OUTA, SDA);                              //Transition SDA pin low
  Set(DIRA, SDA);                                //SDA -> outSendByte for SendByte method
}


int EEPROM::SendByte(unsigned char b)
{
  //Shift a byte to EEPROM msb first.  Return if EEPROM acknowledged.  Returns
  //acknowledge bit.  0 = ACK, 1 = NACK.

  Clear(OUTA,SCL);                               //SCL low, SDA can change
  for(int i=0; i<8; i++)                        //8 reps sends 8 bits
  {
    if( b & 0x80 ) {                             //highest bit sets state of SDA
      Set(OUTA,SDA);
	 }
	 else {
      Clear(OUTA,SDA);
	 }
	 Set(OUTA,SCL);                               //Pulse the SCL line
    Clear(OUTA,SCL);

	 b <<= 1;                                     //Shift b left for next bit
  }
  return GetAck();                               //Call GetByteAck and return EEPROM's Ack
}


int EEPROM::GetAck(void)
{
  //GetByte and return acknowledge bit transmitted by EEPROM after it receives a byte.
  //0 = ACK, 1 = NACK.

  Clear(DIRA,SDA);                               //SDA -> SendByte so 24LC256 controls
  Set(OUTA,SCL);                                 //Start a pulse on SCL
  int ackbit = (INA >> SDAPIN) & 1;                 //GetByte the SDA state from 24LC256
  Clear(OUTA,SCL);                               //Finish SCL pulse
  Clear(OUTA,SDA);                               //SDA will hold low
  Set(DIRA,SDA);                                 //SDA -> outSendByte, master controls

  return ackbit;
}


void EEPROM::i2cStop(void)
{
  //Send I2C stop condition.  SCL must be high as SDA transitions from low to high.
  //See note in i2cStart about SCL line.

  Clear(OUTA,SDA);                                //SDA -> outSendByte low
  Set(DIRA,SDA);
  Set(OUTA,SCL);                                  //SCL -> high
  Clear(DIRA,SDA);                                //SDA -> inSendByte GetBytes pulled up
}


void EEPROM::i2cRelease(void)                    //Necessary to release i2c bus so other devices can use it
{                                                //SDA goes LOW to HIGH with SCL High
   
   Set(OUTA,SCL);                               //Drive SCL HIGH
   Set(OUTA,SDA);                               // then SDA HIGH
   Clear(DIRA,SCL);                             //Now let them float
   Clear(DIRA,SDA);                             //If pullups present, they'll stay HIGH
}


unsigned char EEPROM::GetByte(void)
{
  //Shift in a byte msb first.  

  unsigned char value = 0;                       //Clear value
  Clear(DIRA,SDA);                               //SDA input so 24LC256 can control
  for(int i=0; i<8; i++) {                       //Repeat shift in eight times
    Set(OUTA,SCL);                               //Start an SCL pulse
    value <<= 1;                                 //Shift the value left
    value += (INA >> SDAPIN) & 1;                     //Add the next most significant bit
    Clear(OUTA,SCL);                             //Finish the SCL pulse
  }
  return value;
}


void EEPROM::SendAck( unsigned char ackbit )
{
  //Transmit an acknowledgement bit (ackbit).
  if( ackbit ) {                                 //Set SDA output state to ackbit
    Set(OUTA,SDA);
  }
  else {
    Clear(OUTA,SDA);
  }

  Set(DIRA,SDA);                                 //Make sure SDA is an output
  Set(OUTA,SCL);                                 //Send a pulse on SCL
  Clear(OUTA,SCL);
  Clear(DIRA,SDA);                               //Let go of SDA
}
