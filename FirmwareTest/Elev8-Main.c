/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/

#include <stdio.h>
#include "QuatIMU.h"
//#include <Propeller.h>

int main()                                    // Main function
{
  QuatIMU_Start();

  unsigned int LoopTimer = 0;	//CNT;
  int counter = 0;

  while(1)
  {

    if( (counter & 3) == 0 ) {
      printf( "%08x\t%08x\t%08x\r", QuatIMU_GetTest(), QuatIMU_GetSensors()+1, QuatIMU_GetMatrix() );

      LoopTimer += 80000000/250;
    }      

    counter++;
    LoopTimer += 80000000/250;
    //waitcnt( LoopTimer );
  }
}
