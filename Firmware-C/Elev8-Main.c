/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include <stdio.h>
#include "RC_Receiver.h"
#include "Servo32_HighRes.h"
#include "Sensors.h"
#include "QuatIMU.h"
#include "F32.h"
#include "IntPID.h"
#include <Propeller.h>

const int CS_ALT = 9;
const int CS_AG  = 11;
const int SDO    = 13;
const int SDI    = 14;
const int SCL    = 12;
const int CS_M   = 10;
const int LED_PIN = 8;

const long LED_COUNT = 1;
long LEDValue[1];

int sensVals[16];
INTPID pid;

int main()                                    // Main function
{
  // Add startup code here.
  RC_Start();
  F32_Start();

  QuatIMU_Start();

  Servo32_Init( 400 );
  Servo32_AddFastPin( 15 );
  Servo32_AddFastPin( 16 );
  Servo32_Set( 15, 8000 );
  Servo32_Set( 16, 8000 );
  
  Servo32_Start();

  LEDValue[0] = 0x0F000F;
  
  Sensors_Start( SDI, SDO, SCL, CS_AG, CS_M, CS_ALT, LED_PIN, (int)&LEDValue[0], LED_COUNT );

  unsigned int LoopTimer = CNT;
  int counter = 0;


  while(1)
  {
    int * addr = Sensors_Address();
    memcpy( sensVals, addr, sizeof(int)*11 );
    
    QuatIMU_Update( sensVals + 1 );
    QuatIMU_WaitForCompletion();

    if( (counter & 3) == 0 ) {
      printf( "%d  %d\r", sensVals[1], QuatIMU_GetFixedMatrix()[0] );
      LoopTimer += 80000000/250;
    }      

    counter++;
    LoopTimer += 80000000/250;
    waitcnt( LoopTimer );
  }
}
