
//Constants
//
//Object used to specify cross-module constants, such as pin assigments, update rates, etc

#include <propeller.h>

#define Const_ClockFreq  80000000

#define Const_UpdateRate  250
#define Const_UpdateCycles (Const_ClockFreq / Const_UpdateRate)

#define Const_OneG  4096					//Must match the scale of the accelerometer
#define Const_Alti_UpdateRate  25			//Must match the update rate of the device  
