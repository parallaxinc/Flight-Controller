''
''
''     LSM9DS1 Gyro/Accel/Magnetometer, LPS25H Barometer SPI driver
''
''     Jason Dorie               
''               
'' Note that this code assumes an 80 MHz clock

' 1 / x = 

' 1_000 = ms
' 1_000_000 = us
' 1_000_000_000 = ns

'    20_000_000 = 50 ns        (one instruction, 4 cycles @ 80MHz)
'    80_000_000 = 80MHz cycle


CON
  _clkmode = xtal1 + pll16x
  _clkfreq = 80_000_000


  HUNDRED_nS  = _clkfreq / 10_000_000  'Number of clock cycles per 100 nanoseconds (8 @ 80MHz)                        
  ONE_uS      = HUNDRED_nS * 10 'Number of clock cycles per 1 microsecond (1000 nanoseconds)

' LED_RESET   = 50 * ONE_uS     'Too big to be a constant, so it's in a variable in the DAT section

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
    

  Gy_Temp = 0
  GyroX = 1
  GyroY = 2
  GyroZ = 3
  AccX = 4
  AccY = 5
  AccZ = 6
  MagX = 7
  MagY = 8
  MagZ = 9
  Alt = 10
  AltRate = 11
  AltTemp = 12
  Pressure = 13
  Timer = 14
  ParamsSize = 15
    

VAR

  long  ins[ParamsSize]         'Temp, GX, GY, GZ, AX, AY, AZ, MX, MY, MZ, Alt, AltRate, AltTemp, Pressure, Timer
  long  DriftScale[3]
  long  DriftOffset[3]          'These values will be altered in the EEPROM by the Config Tool and Propeller Eeprom code                       
  long  AccelOffset[3]
  long  MagOffsetX, MagScaleX, MagOffsetY, MagScaleY, MagOffsetZ, MagScaleZ

  long  cog


OBJ

  const  : "Constants.spin"
  settings : "Settings.spin"


PUB start(ipin, opin, cpin, sgpin, smpin, apin, _LEDPin, _LEDAddr, _LEDCount) : okay

'' Start driver - starts a cog
'' returns false if no cog available
'' may be called again to change settings
''
''   ipin    = pin connected to DIN
''   opin    = pin connected to DOUT
''   cpin    = pin connected to CLK
''   sgpin   = pin connected to CS_AG
''   smpin   = pin connected to CS_M
''   apin    = pin connected to CS on altimeter
''   LEDPin  = pin connected to WS2812B LED array
''   LEDAddr = HUB address of RGB values for LED array (updated constantly)
''   LEDCount= Number of LED values to update  

  return startx(@ipin)



PRI startx(ptr) : okay

  stop
  longmove(@ins, ptr, 9)        'Copy the 9 parameters from the stack into the ins array

  ins[9] := @AltTable_000       'Append the HUB address of the pressure to altitude table 

  return cog := cognew(@entry, @ins) + 1


PUB stop

'' Stop driver - frees a cog

  if cog
    cogstop(cog~ - 1)


PUB in(channel)

'' Read the current value from a channel (0..ParamsSize-1)

  return ins[channel]

PUB Address
'' Get the address of the sensor readings
  return @ins


PUB TempZeroDriftValues

  longmove( @DriftScaleGX, @DriftScale[0], 6 )          'Temporarily back up the values in the DAT section so we can restore them with "ResetDriftValues"
  longfill( @DriftScale[0], 0, 6 )

PUB ResetDriftValues
  longmove( @DriftScale[0], @DriftScaleGX, 6 )



PUB TempZeroAccelOffsetValues

  longmove( @AccelOffsetX, @AccelOffset[0], 3 )         'Temporarily back up the values in the DAT section so we can restore them with "ResetAccelOffsetValues"
  longfill( @AccelOffset[0], 0, 3 )

PUB ResetAccelOffsetValues
  longmove( @AccelOffset[0], @AccelOffsetX, 3 )



PUB SetDriftValues( ScaleAndOffsetsAddr )

  longmove( @DriftScale[0], ScaleAndOffsetsAddr, 6 )
  longmove( @DriftScaleGX, ScaleAndOffsetsAddr, 6 )


PUB SetAccelOffsetValues( OffsetsAddr )
  longmove( @AccelOffset[0], OffsetsAddr, 3 )
  longmove( @AccelOffsetX, OffsetsAddr, 3 )


PUB ZeroMagnetometerScaleOffsets
  longfill( @MagOffsetX, 0, 6 )


PUB SetMagnetometerScaleOffsets( MagOffsetsAndScalesAddr )
  longmove( @MagOffsetX, MagOffsetsAndScalesAddr, 6 )
   


DAT

'*********************************************
'* Assembly language LSM9DS1 + LPS25H driver *
'*********************************************

                        org

entry                   mov     t1,par                  'read parameters

                        'First 7 params are pin masks - read them from HUB and output to COG registers in order

                        movd    :writeMask, #imask      'first hub register to write to 
                        mov     counter, #7             '7 parameters (masks) to transfer          
  :paramLoop
                        call    #param                  'read the param (t2 becomes mask, t3 is value)
  :writeMask            mov     imask,t2                'output the mask to our current hub register                
                        add     :writeMask, d_field     'increment the hub register address
                        or      dira, t2                'set the pin as an output - this works for all but one, which we fix below 
                        djnz    counter, #:paramLoop    'loop until done
                              

                        call    #param                  'setup LED Address
                        mov     ledAddress, t3

                        call    #param                  'setup LED count
                        mov     ledCount, t3

                        call    #param                  'set up AltTable address
                        mov     altTableAddr, t3        

                        mov     outAddr, par            'Store the address of the parameters array for output


                        mov     driftHubAddr, par
                        add     driftHubAddr, #ParamsSize*4     'Drift array starts (ParamsSize) longs from the beginning of the output params array                        


                        'Fix the one incorrect pin direction set by the param loop above
                        andn    dira,omask              'SDO pin is an input

                        'All of these were handled by the param loop above
                        'or      dira,imask              'output SDI
                        'or      dira,cmask              'output CLK
                        'or      dira,sgmask             'output CS_AG
                        'or      dira,smmask             'output CS_M
                        'or      dira,amask              'output CS altimeter
                        'or      dira,ledMask            'output LED pin
                        
                        or      outa,ledMask            'bring LED pin high

                        or      outa, sgmask            'bring CS pins high
                        or      outa, smmask
                        or      outa, amask


                        call    #Config_Sensors         'Configure the gyro, accelerometer, mag, altimeter
                                                


'Main sensor read loop

main_loop
                        mov     spi_cs_mask, sgmask     'Start with the gyro/accelerometer


                        mov     spi_reg, #$27           'Read status register
                        call    #SPI_ReadByte           'Read data from SPI
                        andn    spi_data, #$03  nr, wz
                        cmp     spi_data, #$03  wc      'Test status - lowest 2 bits (gyro + accel ready)
                                      
                        'loop while not data ready
              if_c      jmp     #main_loop

                        
                        mov     LoopTime, cnt


                        '---- Temperature --------------
                        mov     spi_reg, #$15
                        call    #SPI_ReadWord           'Read the Temperature register
                        mov     OutTemp, spi_data


                        '---- Gyro X -------------------
                        mov     spi_reg, #$18     
                        call    #SPI_ReadWord           'Read the Gyro X register
                        mov     OutGX, spi_data

                        '---- Gyro Y ----
                        mov     spi_reg, #$1A     
                        call    #SPI_ReadWord           'Read the Gyro Y register
                        mov     OutGY, spi_data

                        '---- Gyro Z ----
                        mov     spi_reg, #$1C     
                        call    #SPI_ReadWord           'Read the Gyro Z register
                        mov     OutGZ, spi_data
                        

                        '---- Accel X ------------------
                        mov     spi_reg, #$28     
                        call    #SPI_ReadWord           'Read the Accelerometer X register
                        mov     OutAX, spi_data

                        '---- Accel Y ----
                        mov     spi_reg, #$2A     
                        call    #SPI_ReadWord           'Read the Accelerometer Y register
                        mov     OutAY, spi_data

                        '---- Accel  Z ----
                        mov     spi_reg, #$2C     
                        call    #SPI_ReadWord           'Read the Accelerometer Z register
                        mov     OutAZ, spi_data




                        '---- Magnetometer--------------
                        mov     spi_cs_mask, smmask     'Next, read the magnetometer


                        'mov     spi_reg, #$27           'Read the Magnetometer status register to see if data is ready
                        'call    #SPI_ReadByte

                        'mov     t3, spi_data            'Store to temp register t3            


:Mag_Read_X
'                        test    t3, #1          wc      'X data available?
'              if_nc     jmp     #:Mag_Done_X                                            


                        mov     spi_reg, #$68           'read the Magnetometer X register ($28 | $40 = continuous read mode)              
                        call    #SPI_ReadWord
                        mov     OutMX, spi_data
:Mag_Done_X

:Mag_Read_Y
'                        test    t3, #2          wc      'Y data available?
'              if_nc     jmp     #:Mag_Done_Y                                            

                        mov     spi_reg, #$6a           'read the Magnetometer Y register ($2a | $40 = continuous read mode)              
                        call    #SPI_ReadWord
                        mov     OutMY, spi_data
:Mag_Done_Y

:Mag_Read_Z
'                        test    t3, #4          wc      'Z data available?
'              if_nc     jmp     #:Mag_Done_Z                                            

                        mov     spi_reg, #$6c           'read the Magnetometer Z register ($2c | $40 = continuous read mode)              
                        call    #SPI_ReadWord
                        mov     OutMZ, spi_data
:Mag_Done_Z

                        '---- End Magnetometer----------




                        '---- Altimeter ----------------
                        mov     spi_cs_mask, amask      'Finally, read the altimeter

                        mov     spi_reg, #$27           'Read the Altimeter status register to see if data is ready
                        call    #SPI_ReadByte

                        mov     t3, spi_data            'Store to temp register t3            

                        test    t3, #1          wc      'Temperature data available?
        if_nc           jmp     #:SkipAltTemperature                                   

:ReadAltTemperature
                        mov     spi_reg, #$6B           'Read the temperature register (| $40 = continuous read mode
                        call    #SPI_ReadWord
                        mov     OutAltTemp, spi_data

:SkipAltTemperature
                        test    t3, #2          wc      'Pressure data available?
        if_nc           jmp     #:SkipAltPressure                                   


:ReadAltPressure
                        mov     spi_reg, #$68           'Read the pressure register (| $40 = continuous read mode)
                        call    #SPI_StartRead
                        mov     OutAltPressure, spi_data

                        call    #SPI_ContinueRead
                        shl     spi_data, #8
                        or      OutAltPressure, spi_data

                        call    #SPI_ContinueRead
                        shl     spi_data, #16
                        or      OutAltPressure, spi_data

                        or      outa, spi_cs_mask       'Set CS high

                        call    #ComputeAltitude        'Computes altitude and difference from previous

:SkipAltPressure

                        call    #ComputeDrift           'Compute the temperature drift offsets
                        call    #ComputeAccelMedian     '~1400 cycles per 9 pt median, ~4200 cycles max
                        
                        subs    OutGX, DriftX
                        subs    OutGY, DriftY           'Apply the temperature drift offsets to the gyro readings
                        subs    OutGZ, DriftZ

                        
                        '---- Write Hub Outputs --------
                        mov     outAddr, par
                        movd    :OutHubAddr, #OutTemp   'Put the COG address to read from in the D field of the :OutHubAddr instruction
                        mov     t1, #14                 '14 parameters to copy from COG to HUB

:HubWriteLoop                                                        

:OutHubAddr             wrlong  0-0, outAddr            'Write the data to the HUB
                        add     :OutHubAddr, d_field    'Increment the COG source address (in the instruction above)
                        add     outAddr, #4             'Increment the HUB target address
                        
                        djnz    t1, #:HubWriteLoop      'Keep going for all 13 registers
                        

                        call    #WriteLEDs


                        sub     LoopTime, cnt
                        neg     LoopTime, LoopTime
                        add     outAddr, #4
                        wrlong  LoopTime, outAddr                                                
                        

                        jmp     #main_loop              'Repeat forever







''------------------------------------------------------------------------------
'' Get parameter, advance parameter pointer, result MASK in t2, VALUE in t3
''------------------------------------------------------------------------------
param                   rdlong  t3,t1                   'get parameter into t3
                        add     t1,#4                   'point to next parameter
                        mov     t2,#1                   'make pin mask in t2
                        shl     t2,t3
param_ret               ret
'------------------------------------------------------------------------------



''------------------------------------------------------------------------------
'' Configure the settings of the LSM-9DS1
''------------------------------------------------------------------------------
Config_Sensors
                        mov     spi_cs_mask, sgmask     'Set the enable pin for the gyro/accelerometer

                        'Ctrl_REG1_G (10h)
                        'Data rate, frequency select, bandwidth for gyro
                        'ODR_G[2..0]__FS_G[1..0]__0__BW_G[1..0]
                        
                        'ODR_G[2:0] := %110     'Output Data Rate = 952hz
                        'FS_G[1..0] := %11      'Full scale operation, 2000 deg/sec  (00 = 245 d/s, 11 = 2000 d/s)        
                        'BW_G[1..0] := %11      'Bandwidth cutoff = 100Hz

                        mov     spi_reg, #$10
                        mov     spi_data, #%110_11_0_11
                        call    #SPI_Write


                        'Ctrl_REG2_G (11h)
                        '0000__INT_SEL[1..0]__OUT_SEL[1..0]
                        'Interrupt Generator
                        'Output selection

                        'Ctrl_REG3_G (12h)
                        'High-pass filter enable & settings

                        'Orient_CFG_G (13h)
                        'Orientation / sign settings
                        '00__SignX_G__SignY_G__SignZ_G__Orient[2..0]                            

                        'Ctrl_REG5_XL (1fh)
                        'Decimation / enable for accelerometer
                        'DEC[1..0]__Zen_XL__Yen_XL__Xen__XL__000



                        'Ctrl_REG6_XL (20h)
                        'Data rate, frequency select, bandwidth for accelerometer
                        'ODR_XL[2..0]__FS_XL[1..0]__BW_SCAL_ODR__BW_XL[1..0]
                        
                        'ODR_XL[2..0] := %101   'Output data rate = 476hz
                        'FS_XL[1..0] := %11     'Accel scale = +/- 8g  (00=2g, 10=4g, 11=8g, 01=16g)
                        'BW_SCAL_ODR := 0       'Scale bandwidth according to sample rate = 0  (1 = use BW_XL)
                        'BW_XL[1..0] := %00     'filter bandwidth (00=408hz, 01=211hz, 10=105hz, 11=50hz), only used if BW_SCAL == 1

                        mov     spi_reg, #$20
                        mov     spi_data, #%110_11_0_00                        
                        call    #SPI_Write
                                                

                        'Remaining Gyro / Accel registers are left at startup defaults



                        'Magnetometer configuration

                        mov     spi_cs_mask, smmask     'Set the enable pin for the magnetometer

                        'Ctrl_Reg1_M (20h)
                        'TempComp___OM[1..0]__DO[2..0]__FastODR__ST
                        'TempComp := 0
                        'OM[1..0] := %11        'Ultra-high performance mode for X&Y axis
                        'DO[2..1] := %111       '80Hz output
                        'Fast_ODR := 0          'Higher than 80Hz not required        
                        'ST := 0                'Self-test disabled

                        mov     spi_reg, #$20
                        mov     spi_data, #%0_11_111_0_0                        
                        call    #SPI_Write


                        'Ctrl_Reg2_M (21h)
                        '0__FS[1..0]__0__REBOOT__SoftRST__00

                        'FS[1..0] := %01        '+/- 8 gauss
                        mov     spi_reg, #$21
                        mov     spi_data, #%0_01_0_0_0_00                        
                        call    #SPI_Write
                        

                        'Ctrl_Reg3_M (22h)
                        'I2CDisable__0__LP__00__SIM__MD[1..0]

                        'I2CDisable := 0        'Disable the I2C interface
                        'LP := 0                'Low-power mode off
                        'SIM := 1               'SPI Read/Write enable  (appears to be incorrectly documented, set to zero instead)
                        'MD[1..0] := %00        'Continuous conversion mode

                        mov     spi_reg, #$22
                        mov     spi_data, #%0_0_0_00_0_00                        
                        call    #SPI_Write
                                                                          

                        'Ctrl_Reg4_M (23h)
                        '0000__OMZ[1..0]__BLE__0

                        'OMZ[1..0] := %11       'ultra-high performance mode for Z axis
                        'BLE := 0               'LSB at low address

                        mov     spi_reg, #$23
                        mov     spi_data, #%0000_11_0_0                        
                        call    #SPI_Write


                        'Ctrl_Reg5_M (24h)
                        'FastRead__BDU_000000

                        'FastRead := 0          'Fast read disabled
                        'BDU := 1               'Block data output until MSB and LSB have been read

                        mov     spi_reg, #$24
                        mov     spi_data, #%0_1_000000                        
                        call    #SPI_Write
                                      
                                                                       
''------------------------------------------------------------------------------
'' Configure the altimeter settings of the LPS25H
''------------------------------------------------------------------------------
                        mov     spi_cs_mask, amask      'Set the enable pin for the altimeter

                        'CTRL_REG1 (20h)
                        'PD__ODR[2..0]__DIFF_EN__BDU__RESET_AZ__SIM

                        'PD := 1 (enable device)
                        'ODR := %100  (25hz output)
                        'DIFF_EN := 0  (differential enable off)
                        'BDU := 1  (block-data update)
                        'RESET_AZ := 0
                        'SIM := 0  (4-wire SPI mode)

                        mov     spi_reg, #$20
                        mov     spi_data, #%1_100_0_1_0_0                        
                        call    #SPI_Write



                        'CTRL_REG2 (21h)
                        'BOOT__FIFO_EN__WTM_EN__FIFO_MEAN_DEC__I2C__SWRESET__AUTO_ZERO__ONE_SHOT
                        'BOOT := 0  (normal operation)
                        'FIFO_EN := 1  (enable fifo)
                        'WTM_EN := 1  (enable watermark level use)
                        'FIFO_MEAN_DEC := 0  (1hz output data rate decimation disabled)

                        'I2C := 0  (I2C mode disabled)
                        'SWRESET := 0  (normal operation)
                        'AUTO_ZERO := 0  (auto-zero mode disabled)
                        'ONE_SHOT := 0  (continuous operation)  

                        mov     spi_reg, #$21
                        mov     spi_data, #%0_1_1_0_0000                        
                        call    #SPI_Write



                        'This register doesn't seem to do anything - Might only be for 1Hz decimation mode?
                        'RES_CONF (0fh) - resolution configure
                        '0000__AVGP[1..0]__AVGT[1..0]                                                

                        'AVGP := 01
                        'AVGT := 01
                        
                        'mov     spi_reg, #$0F
                        'mov     spi_data, #%0000_11_11                        
                        'call    #SPI_Write



                        'FIFO_CTRL (2Eh)
                        'F_MODE[2..0]__WTM_POINT[4..0]

                        'F_MODE := %110 = FIFO mean mode (running average)   (000 = bypass mode)
                        'WTM_POINT := %11111 = 32 sample moving average (%11111 = 32, %00111 = 8, %00011 = 4, %00001 = 2)                         

                        mov     spi_reg, #$2E
                        mov     spi_data, #%110_11111                        
                        call    #SPI_Write
                        

                        'Remaining registers are left at startup defaults
                                                                       

Config_Sensors_ret      ret



''------------------------------------------------------------------------------
'' SPI ReadByte - Read a byte from a register on the gyro/accel device
''
'' spi_reg  - the register to read from
'' spi_data - the resulting 8-bit value read from the device 
''------------------------------------------------------------------------------
SPI_ReadByte
                        call    #SPI_StartRead          'Sets CS low, sends address byte, reads first result
                        or      outa, spi_cs_mask       'Set CS high
SPI_ReadByte_ret        ret
''------------------------------------------------------------------------------


''------------------------------------------------------------------------------
'' SPI_StartRead - Initiate a read from an SPI device, reads first result byte,
'' but leaves the CS line held low to allow additional sequential byte reads by
'' calling SPI_ContinueRead
''
'' spi_reg  - the register to read from
'' spi_data - the resulting 8-bit value read from the device 
''------------------------------------------------------------------------------
SPI_StartRead
                        mov     spi_bits, spi_reg       'Copy the address value into the output bit rack
                        or      spi_bits, #$80          'Set the read bit on the output address value
                        mov     spi_bitcount, #8        '8 bits to send

                        andn    outa, spi_cs_mask       'Set CS low
                        call    #SPI_SendBits           'Send the bits currently in the spi_bits register
                        call    #SPI_ContinueRead       'Reads a result from device
                         
SPI_StartRead_ret       ret
''------------------------------------------------------------------------------


''------------------------------------------------------------------------------
'' SPI_ContinueRead - Continue reading from an SPI device, assuming sequential
'' address mode is enabled on the device
''
'' spi_data - the resulting 8-bit value read from the device 
''------------------------------------------------------------------------------
SPI_ContinueRead
                        mov     spi_data, #0            'Zero the input register
                        mov     spi_bitcount, #8        '8 bits to receive
                        call    #SPI_RecvBits

SPI_ContinueRead_ret    ret
''------------------------------------------------------------------------------


''------------------------------------------------------------------------------
'' SPI ReadWord - read a two-byte value in low-high order from a device
''
'' spi_reg  - the register of the low-byte to read.  High byte is (spi_reg+1)
'' spi_data - the resulting 16-bit value read from the device 
''------------------------------------------------------------------------------
SPI_ReadWord
                        call    #SPI_StartRead
                        'call    #SPI_ContinueRead

                        'The chip will auto-increment registers, so we can just keep reading bits without telling it to stop

                        'Since the low-byte is first, rotate it around, so the register looks like this: 0_L_0_0  (each char is 8 bits)
                        ror     spi_data, #16
                        mov     spi_bitcount, #8        '8 more bits to receive

                        call    #SPI_RecvBits
                        
                        'The high byte was just read into the lowest 8-bits, so it now looks like this: L_0_0_H
                        'Rotate the bits to the left by 8, to move them like this: 0_0_H_L 
                        rol     spi_data, #8

                        or      outa, spi_cs_mask       'Set CS high
                        
                        test    spi_data, bit_15   wc   'Test the sign bit of the result
                        muxc    spi_data, sign_extend   'Replicate the sign bit to the upper-16 bits of the long

SPI_ReadWord_ret        ret



''------------------------------------------------------------------------------
'' SPI Write - write a value to a register on the gyro/accel device
''
'' spi_reg  - the register index to write to
'' spi_data - the value to write to that register  
''------------------------------------------------------------------------------
SPI_Write
                        mov     spi_bits, spi_reg       'Copy the address value into the output bit rack
                        andn    spi_bits, #$80          'Set the read bit on the output address value
                        shl     spi_bits, #8            'Shift the address register up 8 bits to make room for the data value

                        or      spi_bits, spi_data      'OR in the data value

'Call this entry point with 16 bits + write bit already in spi_bits
SPI_WriteFast                        
                        mov     spi_bitcount, #16       'Now have 16 bits total to write                                

                        andn    outa, spi_cs_mask       'Set CS low

                        call    #SPI_SendBits           'Send the bits in the spi_bits register
                        
                        or      outa, spi_cs_mask       'Set CS high
                                                 
SPI_WriteFast_ret
SPI_Write_ret           ret



''------------------------------------------------------------------------------
'' SPI Send Bits - shift bits OUT of spi_bits while toggling the clock
''------------------------------------------------------------------------------
SPI_SendBits
                        ror     spi_bits, spi_bitcount  'Rotate the bits around so the next bit to go out is the HIGH bit

:_loop
                        shl     spi_bits, #1    wc      'Shift the next output bit into the carry                        
                        muxc    outa, imask             'Set SDI to output bit

                        andn    outa, cmask             'Set CLK low
                        nop                             'Wait a teeny bit, just in case 

                        or      outa, cmask             'Set CLK high (device tests bit during transition)
                        nop                             'Wait a teeny bit, just in case 

                        djnz    spi_bitcount, #:_loop   'Loop for all the bits                   

SPI_SendBits_ret        ret


''------------------------------------------------------------------------------
'' SPI Read Bits - shift bits IN to spi_data while toggling the clock
''------------------------------------------------------------------------------
SPI_RecvBits

:_loop
                        andn    outa, cmask             'Set CLK low
                        nop                             'Wait a teeny bit, just in case 

                        test    omask, ina      wc      'Test input bit, sets the carry based on result
                        rcl     spi_data, #1            'Rotate the carry bit into the low bit of the output

                        or      outa, cmask             'Set CLK high
                        nop                             'Wait a teeny bit, just in case 

                        djnz    spi_bitcount, #:_loop   'Loop for all the bits

SPI_RecvBits_ret        ret




''------------------------------------------------------------------------------
'' Write RGB values out to the WS2812b LED array
''------------------------------------------------------------------------------
WriteLEDs
                        andn    outa, ledMask           'Drive the LED line low to reset

                        mov     t3, ledCount
                        mov     t1, ledAddress               
                        
                        mov     ledDelay, cnt
                        add     ledDelay, LED_RESET     'wait for the reset time

                        waitcnt ledDelay, #0

:ledLoop

                        rdlong  spi_bits, t1            'Read the RGB triple from hub memory
                        add     t1, #4                  'Increment to the next address
                        
                        shl     spi_bits, #8            'high bit is the first one out, so shift it into position
                        mov     spi_bitcount, #24       '24 bits to send
:bitLoop
                        rcl     spi_bits, #1    wc

        if_nc           mov     ledDelay, #LED_0_HI  
        if_c            mov     ledDelay, #LED_1_HI                

                        or      outa, ledMask
                        add     ledDelay, cnt           'sync the timer to the bit-delay time
                          
        if_nc           waitcnt ledDelay, #LED_0_LO
        if_c            waitcnt ledDelay, #LED_1_LO                
         
                        andn    outa, ledMask           'pull the LED pin low             
                        waitcnt ledDelay, #0            'hold for the bit duration

                        djnz    spi_bitcount, #:bitLoop

                        djnz    t3, #:ledLoop
                        
WriteLEDs_ret           ret



''------------------------------------------------------------------------------
'' ComputeDrift - calculate corrected gyro values accounting for temperature drift
''------------------------------------------------------------------------------


ComputeDrift

                        mov     t3, driftHubAddr        'Pull the current drift values out of the HUB (allows for dynamic config)
                        add     t3, #12                 'Skip Scale values for the moment
                        rdlong  DriftX, t3              'Read the DriftOffset values into DriftX, Y, Z   
                        add     t3, #4   
                        rdlong  DriftY, t3   
                        add     t3, #4   
                        rdlong  DriftZ, t3   

                        mov     t3, driftHubAddr        'Rewind t3 back to DriftScaleGX

                        mov     t1, #3
                        movd    :addDrift, #DriftX
  :driftLoop                                                       

                        'Compute drift value for X axis
                        rdlong  divisor, t3             'DriftScaleGX, GY, GZ
                        add     t3, #4                                                
                        mov     dividend, OutTemp
                        mov     divResult, #0

                        cmp     divisor, #0     wz, wc
              if_nz     call    #Divide
              
  :addDrift             add     DriftX, divResult
                        add     :addDrift, d_field      'Increment the register to output

                        djnz    t1, #:driftLoop


                        add     t3, #12   
                        rdlong  t2, t3          'AccelOffsetX
                        sub     OutAX, t2
                           
                        add     t3, #4   
                        rdlong  t2, t3          'AccelOffsetY
                        sub     OutAY, t2
                           
                        add     t3, #4   
                        rdlong  t2, t3   
                        sub     OutAZ, t2       'AccelOffsetZ


                        'Apply the magnetometer scale and offset values

  :magScale
                        add     t3, #4                  't3 now points at MagOffsetX
                        mov     t1, #3                  'loop counter
                        
                        movs    :readMag, #OutMX
                        movd    :writeMag, #OutMX
  :magLoop                                                       
                        rdlong  t2, t3                  'Read the MagOffset from the hub
                        add     t3, #4                  'Increment the hub source address
                                                
  :readMag              mov     mul_x, OutMX
                        subs    mul_x, t2               'Subtract the MagOffset value

                        rdlong  mul_y, t3       wz      'Read the MagScale into mul_y
                        add     t3, #4                  'Increment the hub source address
                        
              if_z      jmp     #:skipMul
                        call    #multiply               'Multiply the two values together (if the scale is non-zero)
                        sar     mul_x, #11              '/= 2048

  :skipMul                        
  :writeMag             mov     OutMX, mul_x
                        add     :writeMag, d_field
                        add     :readMag, #1
  
                        djnz    t1, #:magLoop


ComputeDrift_Ret        ret



''------------------------------------------------------------------------------
''------------------------------------------------------------------------------
Divide
                        mov     signbit, dividend
                        xor     signbit, divisor        'Figure out the sign of the result
                        shr     signbit, #31

                        abs     dividend, dividend
                        abs     divisor, divisor

                        shl     divisor, #15      
                        mov     divCounter, #16
                        mov     divResult, dividend     'Copy the source value into the result

:divLoop                cmpsub  divResult, divisor  wc  'if y =< x then subtract it, quotient bit into c
                        rcl     divResult,#1            'rotate c into quotient, shift dividend
                        djnz    divCounter, #:divLoop

                        and     divResult, word_mask    'mask off the remainder (don't need it)

                        cmp     signbit, #0     wc, wz
                        negnz   divResult, divResult

Divide_Ret              ret



'-------------------------------------
'signed multiply, taken from spin interpreter source
'-------------------------------------
'http://forums.parallax.com/forums/default.aspx?f=25&m=394199
' 32 to 64 bit signed multiply mul_x by mul_y
' mul_y absvaled, result in mul_t1:mul_x

'------------------------------------------------------------------------------
multiply
                        abs       mul_x, mul_x  wc      'abs(x)
                        muxc      mul_n, #1             'store sign of x
                        abs       mul_y, mul_y  wc,wz   'abs(y)
              if_c      xor       mul_n, #1             'store sign of y
                        mov       mul_t1, #0
                        mov       mulCounter, #32
                        shr       mul_x, #1     wc
             
:mloop        if_c      add       mul_t1, mul_y wc
                        rcr       mul_t1, #1    wc
                        rcr       mul_x, #1     wc
                        djnz      mulCounter, #:mloop
                        
                        test      mul_n, #1     wz
        if_nz           neg       mul_t1, mul_t1        'restore sign, upper 32 bits
        if_nz           neg       mul_x, mul_x  wz      'restore sign, lower 32 bits
        if_nz           sub       mul_t1, #1
             
multiply_ret            ret
                             

mul_t1                  long                    $0


'------------------------------------------------------------------------------
ComputeAccelMedian
                        'Add the x, y, and z values to the running tables
                        mov     t1, AccelTableIndex
                        add     t1, #AccelXTable
                        movd    :XDest, t1
                        mov     t1, AccelTableIndex
          :XDest        mov     0-0, OutAX 

                        add     t1, #AccelYTable
                        movd    :YDest, t1
                        mov     t1, AccelTableIndex
          :YDest        mov     0-0, OutAY
           
                        add     t1, #AccelZTable
                        movd    :ZDest, t1
                        add     AccelTableIndex, #1
          :ZDest        mov     0-0, OutAZ 

                        cmp     AccelTableIndex, #9     wz
              if_z      mov     AccelTableIndex, #0


                        'Select the middle value from each of the running lists
                        mov     SrcAddr, #AccelXTable
                        call    #SelectTableMedian
                        mov     OutAX, Smallest                        

                        mov     SrcAddr, #AccelYTable
                        call    #SelectTableMedian
                        mov     OutAY, Smallest                        

                        mov     SrcAddr, #AccelZTable
                        call    #SelectTableMedian
                        mov     OutAZ, Smallest                        


ComputeAccelMedian_Ret
                        ret


'------------------------------------------------------------------------------
SelectTableMedian

' Find the median value in a list of 9 signed values
' Go through the list (N+1)/2 times (5 iterations for a 9-entry list)
' In each iteration, choose the smallest value that hasn't already been chosen
' It's basically half a bubble sort

                        mov     UsedMask, #0
                        'Iterate the list 5 times (outer loop)
                        mov     t3, #5

  :Outer
                        mov     Smallest, #1
                        shl     Smallest, #30   'Big number to start with                      

                        'Iterate all 9 entries
                        mov     t2, #9
                        mov     t1, #1
                        movs    :TestAddr, SrcAddr      'Write the address of the start of the table to test
  :Inner                        
                        
                        'If this entry is not masked
                        test    UsedMask, t1            wz
              if_nz     jmp     #:SkipEntry
                        
                        'If this table entry is smaller than the smallest chosen
        :TestAddr       maxs    Smallest, 0-0           wz, wc
        if_nc_or_z      mov     SmallIndex, t1

  :SkipEntry
                        shl     t1, #1
                        add     :TestAddr, #1           'Increment the source address we're testing
                        djnz    t2, #:Inner 

                        or      UsedMask, SmallIndex    'Mark the smallest entry this loop as used
                        djnz    t3, #:Outer                        
                        
SelectTableMedian_Ret
                        ret


'------------------------------------------------------------------------------
ComputeAltitude

' Use the current pressure reading to interpolate altitude readings from the
' altitude table.  Calculation looks like this in C/C++:
 
        'int Index = ((Pressure >> 12) - 260) / 4;      reduces to (Pressure >> 14) - 65
        'if(Index < 0 || Index > 250) return 0.0f;
         
        'int A1 = ((Index * 4) + 260) << 12;
        'int A2 = (((Index+1) * 4) + 260) << 12;
        'int Delta = A2 - A1;    // Always 16384
        'int Frac = Pressure - A1;
         
        'int Tab1 = AltTable[Index];
        'int Tab2 = AltTable[Index+1];
        'int diff = Tab2 - Tab1
         
        'int ResultMM = (Diff * Frac + Alt_Round) / Delta + Tab1;

                        'Cache the previous altitude value, negated        
                        neg     OutAltRate, OutAlt 
         
                        mov     t1, OutAltPressure
                        shr     t1, #14
                        sub     t1, #65         wc      'Carry will be set if the result goes < 0
                        
              if_c      jmp     #:EarlyExit             'Table index is out of range

                        cmp     t1, #251        wc      'Is the index within 0 - 250 range?
              if_nc     jmp     #:EarlyExit             'Nope, quit


                        'At this point we have a table index between 0 and 250
                        'Read the two table values from the hub 

                        shl     t1, #2                  'Table is indexed by longs
                        mov     t2, AltTableAddr        'Compute the HUB address of the first table entry (Index)
                        add     t2, t1
                        mov     t3, t2                  'Copy that address value into t3
                        rdlong  t2, t2                  'Read the 1st table value into T3 (replaces the addres)

                        add     t3, #4                  'Add 4 bytes (1 long) to hub address
                        rdlong  t3, t3                  'Read the 2nd table value into T3 (replaces the address)

                        mov     alt_A1, t1              'Compute the 'base pressure' of the first table index (t1 is already shifted up by 2)
                        adds    alt_A1, #260
                        shl     alt_A1, #12

                        mov     mul_y, OutAltPressure'Compute the difference between the first table pressure and our reading
                        subs    mul_y, alt_A1

                        mov     mul_x, t3            'Compute the difference between the two sequential table entries
                        subs    mul_x, t2                        

                        'mov     mul_x, alt_diff
                        'mov     mul_y, alt_frac
                        call    #multiply               '(Diff * Frac)

                        adds    mul_x, ALT_ROUND        ' + Alt_Round
                        sar     mul_x, #14              ' / Delta

                        adds    mul_x, t2               ' + Tab1
                        mov     OutAlt, mul_x

                        adds    mul_x, OutAltRate       'Equivalent to AltDifference = Alt - prevAlt
                        
                        mov     mul_y, #(Const#Alti_UpdateRate)
                        call    #multiply
                        mov     OutAltRate, mul_x       'AltRate is now in mm/sec
                        
                          
:EarlyExit

ComputeAltitude_ret     ret


        alt_A1          long    0

'------------------------------------------------------------------------------


'
' Initialized data
'
loopDelay               long    _clkfreq / 500          'Main loop at 500 hz
d_field                 long    $200

bit_15                  long    $8000                   'Sign bit of a 16-bit value
sign_extend             long    $FFFF_0000              'Bits to mask in for 16 to 32 bit sign extension
word_mask               long    $0000_FFFF              'lower 16 bits mask

LED_RESET               long    5000                    'minimum of 50 * ONE_uS = 4000 @ 80MHz
ALT_ROUND               long    (16384-1)               'Difference in pressure between two sequential table entries, minus one


AccelTableIndex         long    0                       'Index into the accel values median table
AccelXTable             long    0,0,0,0,0,0,0,0,0       '9 entries per accel table
AccelYTable             long    0,0,0,0,0,0,0,0,0
AccelZTable             long    0,0,0,0,0,0,0,0,0
 
'
' Uninitialized data

spi_reg                 res     1                       'SPI register to read
spi_data                res     1                       'SPI value to output, or result from a read

spi_bits                res     1                       'SPI bits to output in SPI_SendBits function
spi_bitcount            res     1                       'Number of bits to send / receive
spi_cs_mask             res     1                       'Set prior to read - The CS pin mask to enable

t1                      res     1                       '
t2                      res     1                       'internal temporary registers
t3                      res     1                       '

imask                   res     1                       'Device input pin mask (Prop output) 
omask                   res     1                       'Device output pin mask (Prop input)
cmask                   res     1                       'Clock pin mask
sgmask                  res     1                       'Select Gyro pin mask
smmask                  res     1                       'Select Magnetometer pin mask
amask                   res     1                       'Select Altimeter pin mask

ledmask                 res     1                       'LED pin mask
ledAddress              res     1                       'HUB Address of LED values
ledCount                res     1
ledDelay                res     1                       'Next counter value to wait for when sending / receiving

outAddr                 res     1                       'Output hub address        

counter                 res     1                       'generic counter value
driftHubAddr            res     1                       'Hub address of the drift values (for dynamic configuration)

mul_x         'Shared to save space
dividend                res     1

mul_y         'Shared to save space
divisor                 res     1

divResult               res     1
resultShifted           res     1

mul_n         'Shared to save space
signbit                 res     1

mulCounter    'Shared to save space
divCounter              res     1

SrcAddr                 res     1
Smallest                res     1
SmallIndex              res     1                       'Used by the Accelerometer Median computation        
UsedMask                res     1

DriftX                  res     1
DriftY                  res     1
DriftZ                  res     1

OutTemp                 res     1                       'Output of temperature sensor on Gyro/Accel

OutGX                   res     1                       
OutGY                   res     1                       'Output gyro rotation values
OutGZ                   res     1                       

OutAX                   res     1                       
OutAY                   res     1                       'Output accelerometer values
OutAZ                   res     1                       

OutMX                   res     1                       
OutMY                   res     1                       'Output magnetometer field strength values
OutMZ                   res     1

OutAlt                  res     1                       'Output computed altitude from pressure value
OutAltRate              res     1                       'When there's a new altitude, compute this from the differences between readings

OutAltTemp              res     1                       'Output altimeter temperature and pressure values
OutAltPressure          res     1

altTableAddr            res     1                       'HUB ram location of altimeter pressure-to-altitude table

LoopTime                res     1                       'Register used to measure how much time a single loop actually takes


FIT 496       'Make sure all of the above fits into the cog (from the org statement to here)





DAT

        DriftScaleGX            long    0                
        DriftScaleGY            long    0                
        DriftScaleGZ            long    0
                        
        DriftOffsetGX           long    0
        DriftOffsetGY           long    0
        DriftOffsetGZ           long    0
         
        AccelOffsetX            long    0
        AccelOffsetY            long    0
        AccelOffsetZ            long    0
         
        'Table used to convert pressure to altitude.  The Pressure to Altitude conversion is complex,
        'and requires Log and Pow functions, which take a considerable length of CPU time.  A table lookup
        'is a suitable alternative.  I use linear interpolation, but the table has enough points to give an
        'absolute accuracy of +/- 6 inches, with an average accuracy of +/- 1 inch.

        'Table entries are altitude in mm, table index is (hPa - 260)/4, table range is 260 to 1260 hPa   
         
        AltTable_000    long    10108515
        AltTable_001    long    10008960
        AltTable_002    long    9910619
        AltTable_003    long    9813460
        AltTable_004    long    9717451
        AltTable_005    long    9622562
        AltTable_006    long    9528765
        AltTable_007    long    9436031
        AltTable_008    long    9344335
        AltTable_009    long    9253650
        AltTable_010    long    9163951
        AltTable_011    long    9075217
        AltTable_012    long    8987422
        AltTable_013    long    8900546
        AltTable_014    long    8814567
        AltTable_015    long    8729465
        AltTable_016    long    8645220
        AltTable_017    long    8561814
        AltTable_018    long    8479226
        AltTable_019    long    8397441
        AltTable_020    long    8316440
        AltTable_021    long    8236207
        AltTable_022    long    8156726
        AltTable_023    long    8077982
        AltTable_024    long    7999958
        AltTable_025    long    7922642
        AltTable_026    long    7846018
        AltTable_027    long    7770072
        AltTable_028    long    7694793
        AltTable_029    long    7620166
        AltTable_030    long    7546179
        AltTable_031    long    7472819
        AltTable_032    long    7400077
        AltTable_033    long    7327938
        AltTable_034    long    7256394
        AltTable_035    long    7185432
        AltTable_036    long    7115043
        AltTable_037    long    7045215
        AltTable_038    long    6975940
        AltTable_039    long    6907207
        AltTable_040    long    6839008
        AltTable_041    long    6771332
        AltTable_042    long    6704171
        AltTable_043    long    6637517
        AltTable_044    long    6571360
        AltTable_045    long    6505694
        AltTable_046    long    6440508
        AltTable_047    long    6375797
        AltTable_048    long    6311552
        AltTable_049    long    6247765
        AltTable_050    long    6184430
        AltTable_051    long    6121540
        AltTable_052    long    6059086
        AltTable_053    long    5997064
        AltTable_054    long    5935466
        AltTable_055    long    5874285
        AltTable_056    long    5813516
        AltTable_057    long    5753153
        AltTable_058    long    5693188
        AltTable_059    long    5633617
        AltTable_060    long    5574434
        AltTable_061    long    5515633
        AltTable_062    long    5457209
        AltTable_063    long    5399155
        AltTable_064    long    5341469
        AltTable_065    long    5284143
        AltTable_066    long    5227173
        AltTable_067    long    5170554
        AltTable_068    long    5114281
        AltTable_069    long    5058350
        AltTable_070    long    5002756
        AltTable_071    long    4947494
        AltTable_072    long    4892560
        AltTable_073    long    4837950
        AltTable_074    long    4783660
        AltTable_075    long    4729685
        AltTable_076    long    4676021
        AltTable_077    long    4622665
        AltTable_078    long    4569611
        AltTable_079    long    4516858
        AltTable_080    long    4464400
        AltTable_081    long    4412235
        AltTable_082    long    4360358
        AltTable_083    long    4308766
        AltTable_084    long    4257455
        AltTable_085    long    4206423
        AltTable_086    long    4155665
        AltTable_087    long    4105179
        AltTable_088    long    4054961
        AltTable_089    long    4005008
        AltTable_090    long    3955317
        AltTable_091    long    3905884
        AltTable_092    long    3856708
        AltTable_093    long    3807785
        AltTable_094    long    3759112
        AltTable_095    long    3710686
        AltTable_096    long    3662505
        AltTable_097    long    3614565
        AltTable_098    long    3566865
        AltTable_099    long    3519400
        AltTable_100    long    3472170
        AltTable_101    long    3425171
        AltTable_102    long    3378400
        AltTable_103    long    3331856
        AltTable_104    long    3285535
        AltTable_105    long    3239436
        AltTable_106    long    3193556
        AltTable_107    long    3147893
        AltTable_108    long    3102444
        AltTable_109    long    3057207
        AltTable_110    long    3012181
        AltTable_111    long    2967362
        AltTable_112    long    2922749
        AltTable_113    long    2878339
        AltTable_114    long    2834132
        AltTable_115    long    2790123
        AltTable_116    long    2746313
        AltTable_117    long    2702697
        AltTable_118    long    2659276
        AltTable_119    long    2616046
        AltTable_120    long    2573006
        AltTable_121    long    2530154
        AltTable_122    long    2487488
        AltTable_123    long    2445006
        AltTable_124    long    2402707
        AltTable_125    long    2360589
        AltTable_126    long    2318650
        AltTable_127    long    2276889
        AltTable_128    long    2235303
        AltTable_129    long    2193891
        AltTable_130    long    2152652
        AltTable_131    long    2111584
        AltTable_132    long    2070685
        AltTable_133    long    2029953
        AltTable_134    long    1989388
        AltTable_135    long    1948988
        AltTable_136    long    1908751
        AltTable_137    long    1868676
        AltTable_138    long    1828761
        AltTable_139    long    1789004
        AltTable_140    long    1749406
        AltTable_141    long    1709963
        AltTable_142    long    1670676
        AltTable_143    long    1631541
        AltTable_144    long    1592559
        AltTable_145    long    1553727
        AltTable_146    long    1515045
        AltTable_147    long    1476511
        AltTable_148    long    1438124
        AltTable_149    long    1399883
        AltTable_150    long    1361786
        AltTable_151    long    1323832
        AltTable_152    long    1286020
        AltTable_153    long    1248349
        AltTable_154    long    1210818
        AltTable_155    long    1173425
        AltTable_156    long    1136170
        AltTable_157    long    1099051
        AltTable_158    long    1062067
        AltTable_159    long    1025217
        AltTable_160    long    988500
        AltTable_161    long    951915
        AltTable_162    long    915461
        AltTable_163    long    879136
        AltTable_164    long    842941
        AltTable_165    long    806873
        AltTable_166    long    770932
        AltTable_167    long    735117
        AltTable_168    long    699426
        AltTable_169    long    663859
        AltTable_170    long    628415
        AltTable_171    long    593093
        AltTable_172    long    557893
        AltTable_173    long    522812
        AltTable_174    long    487850
        AltTable_175    long    453006
        AltTable_176    long    418280
        AltTable_177    long    383671
        AltTable_178    long    349177
        AltTable_179    long    314797
        AltTable_180    long    280532
        AltTable_181    long    246380
        AltTable_182    long    212339
        AltTable_183    long    178411
        AltTable_184    long    144593
        AltTable_185    long    110884
        AltTable_186    long    77285
        AltTable_187    long    43794
        AltTable_188    long    10410
        AltTable_189    long    -22865
        AltTable_190    long    -56037
        AltTable_191    long    -89102
        AltTable_192    long    -122064
        AltTable_193    long    -154922
        AltTable_194    long    -187676
        AltTable_195    long    -220329
        AltTable_196    long    -252880
        AltTable_197    long    -285330
        AltTable_198    long    -317680
        AltTable_199    long    -349931
        AltTable_200    long    -382083
        AltTable_201    long    -414136
        AltTable_202    long    -446093
        AltTable_203    long    -477952
        AltTable_204    long    -509716
        AltTable_205    long    -541383
        AltTable_206    long    -572957
        AltTable_207    long    -604435
        AltTable_208    long    -635821
        AltTable_209    long    -667113
        AltTable_210    long    -698313
        AltTable_211    long    -729422
        AltTable_212    long    -760439
        AltTable_213    long    -791365
        AltTable_214    long    -822202
        AltTable_215    long    -852949
        AltTable_216    long    -883608
        AltTable_217    long    -914178
        AltTable_218    long    -944661
        AltTable_219    long    -975057
        AltTable_220    long    -1005366
        AltTable_221    long    -1035589
        AltTable_222    long    -1065726
        AltTable_223    long    -1095779
        AltTable_224    long    -1125747
        AltTable_225    long    -1155632
        AltTable_226    long    -1185433
        AltTable_227    long    -1215151
        AltTable_228    long    -1244787
        AltTable_229    long    -1274341
        AltTable_230    long    -1303814
        AltTable_231    long    -1333206
        AltTable_232    long    -1362518
        AltTable_233    long    -1391750
        AltTable_234    long    -1420903
        AltTable_235    long    -1449977
        AltTable_236    long    -1478973
        AltTable_237    long    -1507890
        AltTable_238    long    -1536730
        AltTable_239    long    -1565494
        AltTable_240    long    -1594180
        AltTable_241    long    -1622791
        AltTable_242    long    -1651326
        AltTable_243    long    -1679786
        AltTable_244    long    -1708171
        AltTable_245    long    -1736482
        AltTable_246    long    -1764719
        AltTable_247    long    -1792882
        AltTable_248    long    -1820973
        AltTable_249    long    -1848991
        AltTable_250    long    -1876937
        AltTable_251    long    -1876937        'Last entry is duplicated so we don't have to check / clamp in the code 
        
{{
********************************************************************************************************************************
*                                                   TERMS OF USE: MIT License                                                  *                                                            
********************************************************************************************************************************
*Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    * 
*files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    *
*modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software*
*is furnished to do so, subject to the following conditions:                                                                   *
*                                                                                                                              *
*The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.*
*                                                                                                                              *
*THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          *
*WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         *
*COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   *
*ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         *
********************************************************************************************************************************
}}