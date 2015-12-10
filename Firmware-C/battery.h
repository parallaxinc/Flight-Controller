#ifndef __BATTERY_H__
#define __BATTERY_H__

/*
  Elev8 Flight Controller - V1.0

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

  Written by Jason Dorie
*/


class Battery
{
public:
  static void Init( long _pin );

  static void DischargePin(void);
  static void ChargePin(void);
  static long ReadResult(void);

  static long ComputeVoltage( long ChargeTime );

  static long pin, pinMask;
};


#endif
