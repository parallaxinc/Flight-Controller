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

    loop <<= 1;   // Double the loop count, because we're doing one toggle per loop
    ctr = CNT;

    for( i=0; i<=loop; i++ )
    {
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
    OUTA &= ~((1<<PIN_BUZZER_1) | (1<<PIN_BUZZER_2));
  }    
}    


void BeepTune(void)
{
  BeepHz( 1174, 150 );           //D5
  BeepHz( 1318, 150 );           //E5
  BeepHz( 1046, 150 );           //C5
  BeepHz(  522, 150 );           //C4
  BeepHz(  784, 300 );           //G4
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


static int fraction( int a, int b, int shift)
{
  int f;

  if(shift > 0)                         // if shift, pre-shift a or b left
    a <<= shift;                        // to maintain significant bits while 
  if(shift < 0)                         // insuring proper result
    b <<= -shift;

  for( int i=0; i<32; i++ )             // perform long division of a/b
  {
    f <<= 1;
    if( a >= b )
    {
      a -= b;
      f++;
    }      
    a <<= 1;
  }
  return f;
}  


void BeepOn(int CtrAB, int Pin, int Freq)
{
  int s, d, ctr, frq;

  //Freq = Freq #> 0 <# 500_000         // limit frequency range
  
  ctr = 4 << 26;                        // ..set NCO mode
  s = 1;                                // ..shift = 1

  frq = fraction(Freq, CLKFREQ, s);     // Compute FRQA/FRQB value
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
  else if (CtrAB == 'B' ) {
    CTRB = 0;
  }    
}
