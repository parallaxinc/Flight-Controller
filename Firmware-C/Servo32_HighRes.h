
#ifndef __SERVO32_HIGHRES_H__
#define __SERVO32_HIGHRES_H__

/***********************************
* Servo32-HighRes Driver v1.1      *
* Author: Jason Dorie              *
* See end of file for terms of use *
***********************************/


/*
''*****************************************************************
'' Control up to 32-Servos     Version1.1                08-14-2015 
''*****************************************************************
'' Coded by Jason Dorie
''
'' This is a from-scratch implementation of a 32 output servo driver
''
'' It differs from the standard Parallax driver in a number of ways.
''
'' The Parallax driver will output up to 32 servos at the normal
'' servo update rate of 50Hz, with a precision of 1us, or 1000 steps,
'' That's 80 clock cycles when running @ 80MHz, and is perfectly fine
'' for most applications.
''
'' Some, however, require higher output rates - Multi-rotor
'' applications often want a higher output rate - up to 400Hz.
'' Some applications require higher precision - High-resolution servos
'' exist with a precision of 0.25us (4000 steps) 
''
'' This driver addresses both needs at once.  It allows the user to
'' specify the desired output rate, and has been tested up to 500Hz.
'' You can specify per-pin which outputs are fast (high rate) and
'' which are slow (standard 50Hz).  Each output has a precision of
'' 10 clocks, which translates to 8000 steps over the standard servo
'' range of 1ms to 2ms.
''
'' NOTE: Be careful about your pulse length ranges.  This driver
'' does no clamping, so if you set a pulse length LONGER than your
'' update cycle time you'll hang the driver.  For example, setting a
'' pulse length of 2ms when running at 500Hz.  It's also possible
'' to set pulse lengths too low - Sorting the pulse array is done
'' during the initial part of the pulse - Servos expect a minimum pulse
'' length of 1ms, so I do the work DURING that time.  That time
'' translates to 80_000 clocks.  Worst case the driver takes less
'' than 40_000 clocks to complete.  I've tested with pulse lengths as
'' short as 1/2ms (a value of 4000), and it worked fine. 
''
''*****************************************************************
''
'' The preferred circuit of choice is to place a 4.7K resistor on each signal input
'' to the servo.  If long leads are used, place a 1000uF cap at the servo power
'' connector.  Servo's seem to be happy with a 5V supply receiving a 3.3V signal.
''
''---------------------------------------------------------------------------------
'' Prior to calling the Start method, call either AddFastPin or AddSlowPin for each
'' output you wish to use.  AddFastPin configures the pin for the high output speed,
'' whereas AddSlowPin configures that pin for 50Hz (standard) output. 
''
'' BE VERY CAREFUL if you use this code with standard servos as you can fry them.
'' High output rates will not work with standard analog servos, and will generally
'' cause their internals to short out.  To use a normal servo, use the AddSlowPin
'' function for that output.
''
'' Digital servos are a different story - It varies by manufacturer, but most digital
'' servos will handle update rates above 200Hz, many above 300Hz.
''
'' Electronic Speed Controls (ESCs) also accept high output rates - 400Hz is normal
'' for multi-rotor applications 
''---------------------------------------------------------------------------------
*/

void Servo32_Init( int FastRate );


void Servo32_AddFastPin(int Pin);
void Servo32_AddSlowPin(int Pin);

void Servo32_Start(void);

void Servo32_Set(int ServoPin, int Width);
void Servo32_SetRC(int ServoPin, int Width);


int Servo32_GetCycles(void);


#endif
