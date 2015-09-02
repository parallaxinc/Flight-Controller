{{
File: Propeller Eeprom.spin
Version: 0.6

Developed for forthcoming Propeller Education Kit Lab: EEPROM Datalogging and I2C

See Propeller Eeprom Docs.spin for explanation and instructions.  To view this
object, press, and the file should appear in the Object Info window's Object Explorer
pane.  Double-click it to open.

Note: Greg Glenn -  Added i2cRelease pub function to fully release bus so other
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
    eeprom.VarBackup(@value, @myBytes[20])         ' Copy from RAM to EEPROM
   
    ...
   
    'Restore a previous snapshot of variables in main RAM.
    eeprom.VarRestore(@value, @mybytes[20])
   
    ...
   
    'Copy a snapshot of just the values long array to the same addresses in EEPROM. Since
    'these are long variables, 3 has to be added to address passed to the FromRam method's
    'endAddr parameter to account for the three bytes that follow the long's address. 
    eeprom.VarBackup(@value, @value[31] + 3)
   
    ...
   
    'Copy all the variable contents from the start of the value array through the end of
    'the mybytes array to addresses 20_000..20_158 in EEPROM.
    eeprom.FromRam(@value, @mybytes[20], 20_000)
   
    ...
   
    'Copy only the longs and words back to RAM variables from EEPROM
    'addresses 20_000..20_135.
    eeprom.ToRam(@value, @d + 1, 20_000)
   
NOTES:

(1) The endAddr parameters are byte addresses.  If you use @myLastLongAddress, make sure
to add 3 to the address since it's pointing to the first of the four bytes that make up
the long.  Likewise, if you use @myLastWordAddress, add 1 to account for the two bytes
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
}}

CON

  ' 24LC256 EEPROM constants

  SDA = 29                                       ' P29 = data line
  SCL = 28                                       ' P28 = clock line
  ACK = 0                                        ' Acknowledge bit = 0
  NACK = 1                                       ' (No) acknowledge bit = 1

OBJ

  'docs : "Propeller Eeprom Docs"
  
PUB VarBackup(startAddr, endAddr) | addr, page, eeAddr

  ''Copy contents of address range defined by startAddr..endAddr from main RAM to EEPROM.

  FromRam(startAddr, endAddr, startAddr)           ' Pass addresses to the Write method

PUB VarRestore(startAddr, endAddr) | addr

  ''Copy contents of address range defined by startAddr..endAddr from EEPROM to RAM.

  ToRam(startAddr, endAddr, startAddr)            ' Pass addresses to the read method

PUB FromRam(startAddr, endAddr, eeStart) | addr, page, eeAddr

  ''Copy startAddr..endAddr from main RAM to EEPROM beginning at eeStart address.

  addr := startAddr                              ' Initialize main RAM index
  eeAddr := eeStart                              ' Initialize EEPROM index
  repeat
    page := addr+64-eeAddr//64<#endaddr+1        ' Find next EEPROM page boundary
    SetAddr(eeAddr)                              ' Give EEPROM starting address
    repeat                                       ' Bytes -> EEPROM until page boundary
      SendByte(byte[addr++])
    until addr == page
    i2cRelease                                      ' From 24LC256's page buffer -> EEPROM
    eeaddr := addr - startAddr + eeStart         ' Next EEPROM starting address
  until addr > endAddr                           ' Quit when RAM index > end address

PUB ToRam(startAddr, endAddr, eeStart) | addr

  ''Copy from EEPROM beginning at eeStart address to startAddr..endAddr in main RAM.
  
  SetAddr(eeStart)                               ' Set EEPROM's address pointer 
  i2cstart
  SendByte(%10100001)                            ' EEPROM I2C address + read operation
  if startAddr == endAddr
    addr := startAddr
  else
    repeat addr from startAddr to endAddr - 1      ' Main RAM index startAddr to endAddr
      byte[addr] := GetByte                        ' GetByte byte from EEPROM & copy to RAM 
      SendAck(ACK)                                 ' Acknowledge byte received
  byte[addr] := GetByte                            ' GetByte byte from EEPROM & copy to RAM 
  SendAck(NACK)
  i2cRelease                                        ' Stop sequential read

PRI SetAddr(addr)

  'Sets EEPROM internal address pointer.

  poll                                           ' Poll until EEPROM acknowledges
  SendByte(addr >> 8)                            ' Send address high byte
  SendByte(addr)                                 ' Send address low byte

PRI Poll | ackbit

  ' Poll until acknowledge.  This is especially important if the 24LC256 is copying from
  ' buffer to EEPROM.

  ackbit~~                                       ' Make acknowledge 1
  repeat                                         ' Send/check acknowledge loop
    i2cstart                                     ' Send I2C start condition
    ackbit := SendByte(%10100000)                ' Write command with EEPROM's address
  while ackbit                                   ' Repeat while acknowledge is not 0
  
PRI I2cStart

  ' I2C start condition.  SDA transitions from high to low while the clock is high.
  ' SCL does not have the pullup resistor called for in the I2C protocol, so it has to be
  ' set high. (It can't just be set to inSendByte because the resistor won't pull it up.)

  outa[SCL]~~                                    ' SCL pin outSendByte-high
  dira[SCL]~~
  dira[SDA]~                                     ' Let pulled up SDA pin go high
  outa[SDA]~                                     ' Transition SDA pin low
  dira[SDA]~~                                    ' SDA -> outSendByte for SendByte method

PRI SendByte(b) : ackbit | i

  ' Shift a byte to EEPROM msb first.  Return if EEPROM acknowledged.  Returns
  ' acknowledge bit.  0 = ACK, 1 = NACK.

  b ><= 8                                        ' Reverse bits for shifting msb right
  outa[SCL]~                                     ' SCL low, SDA can change
  repeat 8                                       ' 8 reps sends 8 bits
    outa[SDA] := b                               ' Lowest bit sets state of SDA
    outa[SCL]~~                                  ' Pulse the SCL line
    outa[SCL]~
    b >>= 1                                      ' Shift b right for next bit
  ackbit := GetAck                               ' Call GetByteAck and return EEPROM's Ack

PRI GetAck : ackbit

  ' GetByte and return acknowledge bit transmitted by EEPROM after it receives a byte.
  ' 0 = ACK, 1 = NACK.

  dira[SDA]~                                     ' SDA -> SendByte so 24LC256 controls
  outa[SCL]~~                                    ' Start a pulse on SCL
  ackbit := ina[SDA]                             ' GetByte the SDA state from 24LC256
  outa[SCL]~                                     ' Finish SCL pulse
  outa[SDA]~                                     ' SDA will hold low
  dira[SDA]~~                                    ' SDA -> outSendByte, master controls
  
PRI I2cStop

  ' Send I2C stop condition.  SCL must be high as SDA transitions from low to high.
  ' See note in i2cStart about SCL line.

  outa[SDA]~                                     ' SDA -> outSendByte low
  dira[SDA]~~
  outa[SCL]~~                                    ' SCL -> high
  dira[SDA]~                                     ' SDA -> inSendByte GetBytes pulled up


PUB i2cRelease          ' Necessary to release i2c bus so other devices can use it
                        ' SDA goes LOW to HIGH with SCL High
   
   outa[SCL]~~                         ' Drive SCL HIGH
   outa[SDA]~~                         '  then SDA HIGH
   dira[SCL]~                          ' Now let them float
   dira[SDA]~                          ' If pullups present, they'll stay HIGH
      
PRI GetByte : value

  ' Shift in a byte msb first.  

  value~                                         ' Clear value
  dira[SDA]~                                     ' SDA input so 24LC256 can control
  repeat 8                                       ' Repeat shift in eight times
    outa[SCL]~~                                  ' Start an SCL pulse
    value <<= 1                                  ' Shift the value left
    value += ina[SDA]                            ' Add the next most significant bit
    outa[SCL]~                                   ' Finish the SCL pulse

PRI SendAck(ackbit)

  ' Transmit an acknowledgement bit (ackbit).

  outa[SDA]:=ackbit                              ' Set SDA output state to ackbit
  dira[SDA]~~                                    ' Make sure SDA is an output
  outa[SCL]~~                                    ' Send a pulse on SCL
  outa[SCL]~
  dira[SDA]~                                     ' Let go of SDA