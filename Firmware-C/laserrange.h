#ifndef LASERRANGE_H_
#define LASERRANGE_H_


class LASER_RANGE
{
public:
	LASER_RANGE();

  bool AddChar(char c); // returns true when the Height value is updated
  int   Height;

private:
	int   Working;
	short DigitMult;   // This starts at 1000, when we hit the decimal point it starts reducing
	char  Negative;
	char  FoundDecimal;
};

#endif
