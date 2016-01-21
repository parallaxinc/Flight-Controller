
#ifndef ELEV8DATA_H_
#define ELEV8DATA_H_

#include "packet.h"


class RadioData
{
public:
    short Thro, Aile, Elev, Rudd;
    short Gear, Aux1, Aux2, Aux3;						// Radio values = 16 bytes
    short BatteryVolts;                                  // Battery Monitor = 2 bytes

    // Array index operator, allowing access to the channels by index value
    short operator[](int i) const
    {
        static short dummy;

        switch(i)
        {
            case 0: return Thro;
            case 1: return Aile;
            case 2: return Elev;
            case 3: return Rudd;
            case 4: return Gear;
            case 5: return Aux1;
            case 6: return Aux2;
            case 7: return Aux3;
            case 8: return BatteryVolts;
            default: return dummy;
        }
    }


    void ReadFrom( packet * p )
    {
        Thro = p->GetShort();
        Aile = p->GetShort();
        Elev = p->GetShort();
        Rudd = p->GetShort();
        Gear = p->GetShort();
        Aux1 = p->GetShort();
        Aux2 = p->GetShort();
        Aux3 = p->GetShort();
        BatteryVolts = p->GetShort();
    }
};

class MotorData
{
public:
    short FL, FR, BR, BL;	// front-left, front-right, back-right, back-left motor outputs

    void ReadFrom( packet * p )
    {
        FL = p->GetShort();
        FR = p->GetShort();
        BR = p->GetShort();
        BL = p->GetShort();
    }
};


class SensorData
{
public:
    short Temp, GyroX, GyroY, GyroZ;
    short AccelX, AccelY, AccelZ;						// IMU sensors = 20 bytes
    short MagX, MagY, MagZ;

    void ReadFrom( packet * p )
    {
        Temp =   p->GetShort();
        GyroX =  p->GetShort();
        GyroY =  p->GetShort();
        GyroZ =  p->GetShort();
        AccelX = p->GetShort();
        AccelY = p->GetShort();
        AccelZ = p->GetShort();
        MagX =   p->GetShort();
        MagY =   p->GetShort();
        MagZ =   p->GetShort();
    }
};

class DebugValues
{
public:
    short Version;
    short MinCycles, MaxCycles, AvgCycles;
    int Counter;

    void ReadFrom( packet * p )
    {
        Version   = p->GetShort();
        MinCycles = p->GetShort();
        MaxCycles = p->GetShort();
        AvgCycles = p->GetShort();
        Counter =   p->GetInt();		// basically a sequence value
    }
};


class ComputedData
{
public:
    int Pitch, Roll, Yaw;								// IMU = 12 bytes
    int Alt, AltTemp, AltiEst;							// Altimeter = 12 bytes

    void ReadFrom( packet * p )
    {
        Pitch = p->GetInt();
        Roll =  p->GetInt();
        Yaw =   p->GetInt();

        Alt =     p->GetInt();
        AltTemp = p->GetInt();
        AltiEst = p->GetInt();
    }
};


// Used during channel scale / offset calibration
struct ChannelData
{
	short min, max;
	bool reverse;
	int scale, center;

	ChannelData() {
		min = 0;
		max = 0;
		reverse = false;
		scale = 1024;
		center = 0;
	}
};


#endif
