
#include <propeller.h>
#include "rc.h"
#include "pins.h"

static const int Scale = 80/2; // System clock frequency in Mhz, halved - we're converting outputs to 1/2 microsecond resolution

static struct {
  long Pins[8];
  long PinMask;
} data;


void RC::Start(void)
{
	// Input pins are P0,1,2,3,4,5,26,27
  data.PinMask = PIN_RC_MASK; // Elev8-FC pins are defined in pins.h

#if defined( __PINS_V2_H__ )
  use_cog_driver(rc_driver_v2);
  load_cog_driver(rc_driver_v2, &data);
#endif

#if defined( __PINS_V3_H__ )
  use_cog_driver(rc_driver_v3);
  load_cog_driver(rc_driver_v3, &data);
#endif
}

//int RC::Get( int _pin ) {
//	// Get receiver servo pulse width in uS
//	return data.Pins[_pin] / Scale;     // Get pulse width from Pins[..] , convert to uSec
//}

int RC::GetRC(int _pin) {
	// Get receiver servo pulse width as normal r/c values (+/-1000) 
	return data.Pins[_pin] / Scale - 3000; // Get pulse width from Pins[..], convert to uSec, make 0 center
}

//int RC::Channel( int _pin ) {
//	return data.Pins[_pin] / Scale;
//}
