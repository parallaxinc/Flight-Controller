
#ifndef __BEEP_H__
#define __BEEP_H__

/*
  Elev8 Flight Controller

  Copyright 2015 Parallax Inc

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

  Written by Jason Dorie
*/


void BeepHz( int Hz , int Delay );
void BeepTune(void);

void Beep(void);
void Beep2(void);
void Beep3(void);

void BeepOn(int CtrAB, int Pin, int Freq);
void BeepOff(int CtrAB);

#endif
