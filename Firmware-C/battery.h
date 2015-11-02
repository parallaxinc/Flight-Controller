#ifndef __BATTERY_H__
#define __BATTERY_H__


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
