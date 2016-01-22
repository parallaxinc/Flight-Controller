#ifndef LASERRANGE_H_
#define LASERRANGE_H_


class LASER_RANGE
{
public:
	LASER_RANGE();

	void  AddChar(char c);
	int   Height;

private:
	int   Working;
	short DigitMult;   // This starts at 1000, when we hit the decimal point it starts reducing
	char  Negative;
	char  FoundDecimal;
};

#endif
