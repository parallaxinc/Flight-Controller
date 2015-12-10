/*
  Elev8 Flight Controller

  Copyright 2015 Parallax Inc

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

  Written by Jason Dorie
*/

#include <propeller.h>

#include "constants.h"
#include "sbus.h"

static char Cog;

static struct DATA {
  long  InputMask;
  long  BaudDelay;
  short Channels[18];  //Last two channels are digital
} data;


void SBUS::Start( int InputPin )
{
  data.InputMask = 1 << InputPin;
  data.BaudDelay = Const_ClockFreq / 100000;                        // SBUS is 100,000 bps 

  data.Channels[0] = 0;          // Throttle is zero'd
  for( int i=1; i<18; i++ ) {
    data.Channels[i] = 1024;     // All other channels are centered
  }

  use_cog_driver(sbus_driver);
  Cog = load_cog_driver(sbus_driver, &data) + 1;
}


void SBUS::Stop(void)
{
	// Stop driver and release cog
  if(Cog) {
    cogstop(Cog - 1);
    Cog = 0;
  }
}

short SBUS::GetRC( int i ) {
  return data.Channels[i] - 1024;
}
