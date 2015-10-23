
#ifndef __RC_RECEIVER__
#define __RC_RECEIVER__

class RC
{
public:
  static void Start(void);
  //static int  Get(int _pin);
  static int  GetRC(int _pin);
  //static int  Channel(int _pin);
};

#endif
