/*
  Elev8 Flight Controller - V1.0

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

  Written by Jason Dorie
*/

#include <propeller.h>
#include "battery.h"

long Battery::pin = 0;
long Battery::pinMask = 0;

// This table represents how many cycles it takes for the battery monitor pin to charge to 1.65 volts
// These values are computed for a resistor of 470k and a capacitor of 0.1uF
static long ChargeTimeTable[] = 
{
  348635, // 16.0v
  358083, // 15.6v
  368058, // 15.2v
  386046, // 14.8v
  389774, // 14.4v
  401624, // 14.0v
  414217, // 13.6v
  427628, // 13.2v
  441937, // 12.8v
  457238, // 12.4v
  473639, // 12.0v
  491262, // 11.6v
  510251, // 11.2v
  530770, // 10.8v
  553012, // 10.4v
  577205, // 10.0v
  603618, //  9.6v
  632572, //  9.2v
  664454, //  8.8v
  699733, //  8.4v
  738985, //  8.0v
};  

const int FirstChargeValue = 1600; // 16.00v
const int LastChargeValue  =  800; //  8.00v
const int ChargeStep =  40;        //  0.40v per entry

const int ChargeTimeCount = sizeof(ChargeTimeTable) / sizeof(long);


void Battery::Init( long _pin )
{
  // Set which pin the battery voltage monitor is connected to
  pin = _pin;
  pinMask = 1 << _pin;
}  


void Battery::DischargePin( void )
{
  // Discharge the battery monitor capacitor and time how long it takes to come back up to the threshold voltage
  CTRB = (12 << 26) | (7 << 23) | pin;

  DIRA |= pinMask;
  OUTA &= ~pinMask;
}


void Battery::ChargePin( void )
{
  FRQB = 1;
  PHSB = 0;
  DIRA &= ~pinMask;
}


long Battery::ReadResult( void )
{
  return PHSB;
}

long Battery::ComputeVoltage( long ChargeTime )
{
  long Result = 0;

  if( ChargeTime < ChargeTimeTable[0] )
    Result = FirstChargeValue;
  else if( ChargeTime > ChargeTimeTable[ ChargeTimeCount-1 ] )
    Result = 0;   // Unmeasurable, call it zero
  else
  {
    int low = 0;
    int high = ChargeTimeCount-1;

    do {
      int mid = (low+high) >> 1;
      long midVal = ChargeTimeTable[mid];

      if( ChargeTime < midVal ) {
        high = mid-1;
      }
      else if( ChargeTime > midVal ) {
        low = mid+1;
      }
      else {
        return FirstChargeValue - (mid * ChargeStep);
      }
    } while( low < high );

    low--;
    int lowVal = ChargeTimeTable[low];
    int highVal = ChargeTimeTable[high];

    int tableDelta = highVal - lowVal;
    int chargeDelta = lowVal - ChargeTime;

    int percent = chargeDelta * 64 / tableDelta;
    return FirstChargeValue - (low * ChargeStep) + (ChargeStep * percent) / 64;
  }

  return Result;
}
