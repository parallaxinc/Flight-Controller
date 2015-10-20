
#ifndef __RC_RECEIVER__
#define __RC_RECEIVER__


long RC_Pins[8];
long RC_PinMask;

void RC_Start(void);
int RC_Get( int _pin );
int RC_GetRC(int _pin);
int RC_Channel( int _pin );


#endif
