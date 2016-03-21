#include "laserrange.h"

LASER_RANGE::LASER_RANGE()
{
	Height = 0;
	Working = 0;
	DigitMult = 1000;    // Readings are in meters, we want millimeters.  Whole digits are multiplied by this value until we hit a decimal point.
	Negative = 0;
	FoundDecimal = 0;
}

bool LASER_RANGE::AddChar(char c)
{
	switch( c )
	{
	case '-':
		Negative = -1;
		break;

	case '.':
		FoundDecimal = 1;
		break;

	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		if( FoundDecimal ) {
			DigitMult /= 10;
		}
		else {
			Working *= 10;
		}
		Working += (c - '0') * DigitMult;
		break;

	case 13:
		Height = Negative  ?  -Working : Working;
		DigitMult = 1000;
		Negative = 0;
		FoundDecimal = 0;
      Working = 0;
      return true;  // Height value was updated
	}

   return false;  // Height value hasn't changed
}
