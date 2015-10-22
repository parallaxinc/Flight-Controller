#include <propeller.h>
#include "constants.h"
#include "sbus.h"

static int Cog;

static struct DATA {
  long  InputMask;
  long  BaudDelay;
  short Channels[18];  //Last two channels are digital
  short CenterOffset;
} data;


void SBUS::Start( int InputPin , short Center )
{
  data.CenterOffset = Center;
  data.InputMask = 1 << InputPin;
  data.BaudDelay = Const_ClockFreq / 100000;                        //SBUS is 100,000 bps 

  use_cog_driver(SBUS_driver);
  Cog = load_cog_driver(SBUS_driver, &data) + 1;
}


void SBUS::Stop(void)
{
	// Stop driver and release cog
  if(Cog) {
    cogstop(Cog - 1);
	Cog = 0;
  }
}

short SBUS::Get( int i )
{
  return data.Channels[i];
}

short SBUS::GetRC( int i )
{
  return data.Channels[i] - data.CenterOffset;
}
