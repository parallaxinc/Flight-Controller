
#ifndef __SENSORS_H__
#define __SENSORS_H__


void Sensors_Start(int ipin, int opin, int cpin, int sgpin, int smpin, int apin, int _LEDPin, int _LEDAddr, int _LEDCount);

void Sensors_Stop(void);

long Sensors_In(int channel);
long* Sensors_Address(void);

void Sensors_TempZeroDriftValues(void);
void Sensors_ResetDriftValues(void);

void Sensors_TempZeroAccelOffsetValues(void);
void Sensors_ResetAccelOffsetValues(void);
void Sensors_SetDriftValues( int * ScaleAndOffsetsAddr );

void Sensors_SetAccelOffsetValues( int * OffsetsAddr );
void Sensors_ZeroMagnetometerScaleOffsets(void);
void Sensors_SetMagnetometerScaleOffsets( int MagOffsetsAndScalesAddr );


#define Sensors_ParamsSize 15


#endif
