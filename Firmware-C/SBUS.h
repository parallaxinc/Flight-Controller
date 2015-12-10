#ifndef __SBUS_H__
#define __SBUS_H__

/*
  Elev8 Flight Controller

  Copyright 2015 Parallax Inc

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

  Written by Jason Dorie
*/

class SBUS
{
public:
	static void Start( int InputPin );
	static void Stop(void);

	//static short Get( int i );
	static short GetRC( int i );
};

#endif
