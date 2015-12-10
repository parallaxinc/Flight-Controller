
#ifndef __SENSORS_H__
#define __SENSORS_H__

/*
  Elev8 Flight Controller

  Copyright 2015 Parallax Inc

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

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
