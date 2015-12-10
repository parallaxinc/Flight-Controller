
#ifndef __RC_RECEIVER__
#define __RC_RECEIVER__

/*
  Elev8 Flight Controller

  Copyright 2015 Parallax Inc

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

  Written by Jason Dorie
*/

class RC
{
public:
  static void Start(char UsePPM);
  static void Stop(void);
  //static int  Get(int _pin);
  static int  GetRC(int _pin);
  //static int  Channel(int _pin);
};

#endif
