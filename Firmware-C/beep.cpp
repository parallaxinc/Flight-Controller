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

#include <propeller.h>
#include "beep.h"
#include "pins.h"


void BeepHz( int Hz , int Delay )
{
  int i, loop, d, ctr;

  //Note that each loop does a high and low cycle, so we divide clkfreq by 2 and 2000 instead of 1 and 1000

  d = (80000000/2) / Hz;                        //Compute the amount of time to delay between pulses to get the right frequency
  loop = (Delay * (80000000/2000)) / d;         //How many iterations of the loop to make "Delay" milliseconds?

  if( PIN_BUZZER_1 == PIN_BUZZER_2 )
  {
    //Revision 3 firmware has one buzzer pin  

    int d2 = d>>2;    // First phase is short so we don't hold the power line for too long
    d = d + (d-d2);   // Second phase makes up the difference in the delay

    ctr = CNT;
    for( i=0; i<=loop; i++ )
    {
      OUTA ^= (1<<PIN_BUZZER_1);
      ctr += d2;
      waitcnt( ctr );

      OUTA ^= (1<<PIN_BUZZER_1);
      ctr += d;
      waitcnt( ctr );
    }
  }
  else
  {
    //Revision 2 firmware has two buzzer pins  

    ctr = CNT;
    for( i=0; i<=loop; i++ )
    {
      OUTA |= (1<<PIN_BUZZER_1);
      OUTA &= ~(1<<PIN_BUZZER_2);
     
      ctr += d;
      waitcnt( ctr );
     
      OUTA &= ~(1<<PIN_BUZZER_1);
      OUTA |= (1<<PIN_BUZZER_2);
     
      ctr += d;
      waitcnt( ctr );
    }
  }

  OUTA &= ~((1<<PIN_BUZZER_1) | (1<<PIN_BUZZER_2)); // Make sure the pin is off when we're done so counters can still toggle it
}    


void BeepTune(void)
{
//  BeepHz( 1174, 150 );           //D5
//  BeepHz( 1318, 150 );           //E5
//  BeepHz( 1046, 150 );           //C5
//  BeepHz(  522, 150 );           //C4
//  BeepHz(  784, 300 );           //G4

  BeepHz( 1046, 150 );           //C5
  BeepHz( 1318, 300 );           //E5
}


void Beep(void)
{
  BeepHz( 5000 , 80 );
}

void Beep2(void)
{
  Beep();
  waitcnt( 5000000 + CNT );
  Beep();
}  


void Beep3(void)
{
  Beep();
  waitcnt( 5000000 + CNT );
  Beep();
  waitcnt( 5000000 + CNT );
  Beep();
}

// Return the lower 32 bits of a 32.32 division of (a.0) by (b.0)
static int fraction( int a, int b )
{
  int f = 0;

  a <<= 1;                              // to maintain significant bits
  for( int i=0; i<32; i++ )             // perform long division of a/b
  {
    f <<= 1;
    if( a >= b ) {
      a -= b;
      f++;
    }
    a <<= 1;
  }
  return f;
}


void BeepOn(int CtrAB, int Pin, int Freq)
{
  int ctr, frq;

  //Freq = Freq #> 0 <# 500_000         // limit frequency range

  ctr = 4 << 26;                        // ..set NCO mode

  frq = fraction(Freq, CLKFREQ);        // Compute FRQA/FRQB value
  ctr |= Pin;                           // set PINA to complete CTRA/CTRB value

  if(CtrAB == 'A' )
  {
     CTRA = ctr;                        // set CTRA
     FRQA = frq;                        // set FRQA                   
  }
  else if( CtrAB == 'B' )
  {
     CTRB = ctr;                        // set CTRB
     FRQB = frq;                        // set FRQB                   
  }
  DIRA |= (1<<Pin);                    // make pin output
}

void BeepOff( int CtrAB )
{
  if( CtrAB == 'A' )
    CTRA = 0;
  else if (CtrAB == 'B' )
    CTRB = 0;
}
