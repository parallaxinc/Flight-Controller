#ifndef __SBUS_H__
#define __SBUS_H__

class SBUS
{
public:
	static void Start( int InputPin );
	static void Stop(void);

	//static short Get( int i );
	static short GetRC( int i );
};

#endif
