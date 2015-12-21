
#ifndef __SENSORS_H__
#define __SENSORS_H__

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

void Sensors_Start(int ipin, int opin, int cpin, int sgpin, int smpin, int apin, int _LEDPin, int _LEDAddr, int _LEDCount);

void Sensors_Stop(void);

int Sensors_In(int channel);
int* Sensors_Address(void);

void Sensors_TempZeroDriftValues(void);
void Sensors_ResetDriftValues(void);

void Sensors_TempZeroAccelOffsetValues(void);
void Sensors_ResetAccelOffsetValues(void);
void Sensors_SetDriftValues( int * ScaleAndOffsetsAddr );

void Sensors_SetAccelOffsetValues( int * OffsetsAddr );
void Sensors_ZeroMagnetometerScaleOffsets(void);
void Sensors_SetMagnetometerScaleOffsets( int * MagOffsetsAndScalesAddr );


struct SENS {
  long Temperature;               // Gyro temperature
  long GyroX, GyroY, GyroZ;       // Gyro readings
  long AccelX, AccelY, AccelZ;    // Accelerometer readings
  long MagX, MagY, MagZ;          // Magnetometer readings
  long Alt, AltRate;              // Computed altimeter height (mm) and rate (mm/sec)
  long AltTemp, Pressure;         // Altimeter temperature and pressure
  long SensorTime;                //How long sensors took to read (debug / optimization test value)
};

#define Sensors_ParamsSize  sizeof(SENS)
#define Sensors_ParamsCount (sizeof(SENS) / sizeof(long))

#endif
