#ifndef __SBUS_H__
#define __SBUS_H__

class SBUS
{
public:
	static void Start( int InputPin , short Center );
	static void Stop(void);

	//static short Get( int i );
	static short GetRC( int i );
};

#endif
