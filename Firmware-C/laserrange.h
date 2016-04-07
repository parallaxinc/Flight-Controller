#ifndef LASERRANGE_H_
#define LASERRANGE_H_


// un-comment this define to enable the laser rangefinder, comment it out to disable it
//----------------------------
#define ENABLE_LASER_RANGE
//----------------------------


#ifdef ENABLE_LASER_RANGE

class LASER_RANGE
{
public:
  LASER_RANGE() : Height(0), Working(0), DigitMult(1000), Negative(0), FoundDecimal(0) {;}

  bool AddChar(char c); // returns true when the Height value is updated
  int   Height;

private:
	int   Working;
	short DigitMult;   // This starts at 1000, when we hit the decimal point it starts reducing
	char  Negative;
	char  FoundDecimal;
};

// Global laser range parser object
extern LASER_RANGE LaserRange;

#define LASER_STACK_SIZE (16 + 40)          // stack needs to accomodate thread control structure (40) plus room for functions (16)
extern int laser_stack[LASER_STACK_SIZE];

#endif

#endif
