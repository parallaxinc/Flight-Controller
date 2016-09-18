
#ifndef ELEV8DATA_H_
#define ELEV8DATA_H_

#include "packet.h"

struct RadioPacked
{
	quint8	ThroLow;
	quint8	AileLow;
	quint8	ElevLow;
	quint8	RuddLow;
	quint8	GearLow;
	quint8	Aux1Low;
	quint8	Aux2Low;
	quint8	Aux3Low;

	quint8	ThroHigh:4;
	quint8	AileHigh:4;
	quint8	ElevHigh:4;
	quint8	RuddHigh:4;
	quint8	GearHigh:4;
	quint8	Aux1High:4;
	quint8	Aux2High:4;
	quint8	Aux3High:4;

	short	Battery;


	void ReadFrom( packet * p )
	{
		ThroLow = p->GetByte();
		AileLow = p->GetByte();
		ElevLow = p->GetByte();
		RuddLow = p->GetByte();
		GearLow = p->GetByte();
		Aux1Low = p->GetByte();
		Aux2Low = p->GetByte();
		Aux3Low = p->GetByte();

		quint8 temp;

		temp = p->GetByte();
		ThroHigh = temp & 15;
		AileHigh = temp >> 4;

		temp = p->GetByte();
		ElevHigh = temp & 15;
		RuddHigh = temp >> 4;

		temp = p->GetByte();
		GearHigh = temp & 15;
		Aux1High = temp >> 4;

		temp = p->GetByte();
		Aux2High = temp & 15;
		Aux3High = temp >> 4;

		Battery = p->GetShort();
	}
};


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

	RadioData &operator =(RadioPacked &rhs)
	{
		Thro = rhs.ThroLow | (rhs.ThroHigh << 8);
		if( rhs.ThroHigh & 8) Thro |= 0xF000;

		Aile = rhs.AileLow | (rhs.AileHigh << 8);
		if( rhs.AileHigh & 8) Aile |= 0xF000;

		Elev = rhs.ElevLow | (rhs.ElevHigh << 8);
		if( rhs.ElevHigh & 8) Elev |= 0xF000;

		Rudd = rhs.RuddLow | (rhs.RuddHigh << 8);
		if( rhs.RuddHigh & 8) Rudd |= 0xF000;

		Gear = rhs.GearLow | (rhs.GearHigh << 8);
		if( rhs.GearHigh & 8) Gear |= 0xF000;

		Aux1 = rhs.Aux1Low | (rhs.Aux1High << 8);
		if( rhs.Aux1High & 8) Aux1 |= 0xF000;

		Aux2 = rhs.Aux2Low | (rhs.Aux2High << 8);
		if( rhs.Aux2High & 8) Aux2 |= 0xF000;

		Aux3 = rhs.Aux3Low | (rhs.Aux3High << 8);
		if( rhs.Aux3High & 8) Aux3 |= 0xF000;

		BatteryVolts = rhs.Battery;

		return *this;
	}

};

class MotorData
{
public:
	short FL, FR, BR, BL, CR, CL;	// front-left, front-right, back-right, back-left, center-right, center-left motor outputs
	bool isHex;

    void ReadFrom( packet * p )
    {
        FL = p->GetShort();
        FR = p->GetShort();
        BR = p->GetShort();
        BL = p->GetShort();

		isHex = p->len == 12+2;	// Checksum is 2 extra bytes

		if( isHex ) {
			CR = p->GetShort();
			CL = p->GetShort();
		}
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
	quint16 Counter;

    void ReadFrom( packet * p )
    {
        Version   = p->GetShort();
        MinCycles = p->GetShort();
        MaxCycles = p->GetShort();
        AvgCycles = p->GetShort();
		Counter =   p->GetShort();		// basically a sequence value
    }
};


class ComputedData
{
public:
	short Pitch, Roll, Yaw;								// IMU = 6 bytes
	int Alt, AltiEst;
	int GroundHeight;									// Altimeter = 10 bytes

    void ReadFrom( packet * p )
    {
		Pitch = p->GetShort();
		Roll =  p->GetShort();
		Yaw =   p->GetShort();

        Alt =     p->GetInt();
		GroundHeight = (quint16)(p->GetShort()) * 2;	// unpack GroundHeight back to mm
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
