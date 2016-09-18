
#ifndef __PREFS_H__
#define __PREFS_H__

/*
  This file is part of the ELEV-8 Flight Controller Firmware
  for Parallax part #80204, Revision A
  
  Copyright 2015 Parallax Incorporated

  ELEV-8 Flight Controller Firmware is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  ELEV-8 Flight Controller Firmware is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the ELEV-8 Flight Controller Firmware.  If not, see <http://www.gnu.org/licenses/>.
  
  Written by Jason Dorie

//
// Preferences - User prefs storage for Elev8-FC
//
*/

typedef unsigned char byte;


typedef struct {
	int DriftScaleX,  DriftScaleY,  DriftScaleZ;
	int DriftOffsetX, DriftOffsetY, DriftOffsetZ;
	int AccelOffsetX, AccelOffsetY, AccelOffsetZ;
	int MagOfsX, MagScaleX, MagOfsY, MagScaleY, MagOfsZ, MagScaleZ;

	float RollCorrectSin, RollCorrectCos;
	float PitchCorrectSin, PitchCorrectCos;

	float AutoLevelRollPitch;
	float AutoLevelYawRate;
	float ManualRollPitchRate;
	float ManualYawRate;

	byte  PitchGain;
	byte  RollGain;
	byte  YawGain;
	byte  AscentGain;

	byte  AltiGain;
	byte  PitchRollLocked;
	byte  UseAdvancedPID;
	byte  unused;

	byte  ReceiverType;     // 0 = PWM, 1 = SBUS, 2 = PPM
	byte  unused2;
	byte  UseBattMon;
	byte  DisableMotors;

	byte  LowVoltageAlarm;
	byte  LowVoltageAscentLimit;
	short ThrottleTest;     // Typically the same as MinThrottleArmed, unless MinThrottleArmed is too low for movement

	short MinThrottle;      // Minimum motor output value
	short MaxThrottle;      // Maximum motor output value
	short CenterThrottle;   // Mid-point motor output value
	short MinThrottleArmed; // Minimum throttle output value when armed - MUST be equal or greater than MinThrottle
	short ArmDelay;
	short DisarmDelay;

	short ThrustCorrectionScale;  // 0 to 256  =  0 to 1
	short AccelCorrectionFilter;  // 0 to 256  =  0 to 1

	short VoltageOffset;    // Used to correct the difference between measured and actual voltage
	short LowVoltageAlarmThreshold;  // default is 1050 (10.50v)

	byte  FlightMode[3];    // Flight mode to use when gear switch is down, middle, up
	byte  AccelCorrectionStrength;

	byte  AileExpo;
	byte  ElevExpo;
	byte  RuddExpo;
	byte  unused3;

	byte  ThroChannel;      // Radio inputs to use for each value
	byte  AileChannel;
	byte  ElevChannel;
	byte  RuddChannel;
	byte  GearChannel;
	byte  Aux1Channel;
	byte  Aux2Channel;
	byte  Aux3Channel;

	short ThroScale;
	short AileScale;
	short ElevScale;
	short RuddScale;
	short GearScale;
	short Aux1Scale;
	short Aux2Scale;
	short Aux3Scale;

	short ThroCenter;
	short AileCenter;
	short ElevCenter;
	short RuddCenter;
	short GearCenter;
	short Aux1Center;
	short Aux2Center;
	short Aux3Center;

	int   Checksum;

	// Accessors for looping over channel assignments, scales, centers
	byte & ChannelIndex( int index )   { return (&ThroChannel)[index];}
	short & ChannelScale( int index )  { return (&ThroScale)[index];  }
	short & ChannelCenter( int index ) { return (&ThroCenter)[index]; }

} PREFS;


int Prefs_CalculateChecksum( PREFS & PrefsStruct );

#endif
