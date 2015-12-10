
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

typedef struct {
  int   DriftScale[3];
  int   DriftOffset[3];
  int   AccelOffset[3];
  int   MagScaleOfs[6];

  float RollCorrect[2];
  float PitchCorrect[2];

  float AutoLevelRollPitch;
  float AutoLevelYawRate;
  float ManualRollPitchRate;
  float ManualYawRate;

  char  PitchGain;
  char  RollGain;
  char  YawGain;
  char  AscentGain;

  char  AltiGain;
  char  PitchRollLocked;
  char  UseAdvancedPID;
  char  unused;

  char  ReceiverType;     // 0 = PWM, 1 = SBUS, 2 = PPM
  char  unused2;
  char  UseBattMon;
  char  DisableMotors;

  char  LowVoltageAlarm;
  char  LowVoltageAscentLimit;
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

  char  ThroChannel;      // Radio inputs to use for each value
  char  AileChannel;
  char  ElevChannel;
  char  RuddChannel;
  char  GearChannel;
  char  Aux1Channel;
  char  Aux2Channel;
  char  Aux3Channel;

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
  char & ChannelIndex( int index )   { return (&ThroChannel)[index];}
  short & ChannelScale( int index )  { return (&ThroScale)[index];  }
  short & ChannelCenter( int index ) { return (&ThroCenter)[index]; }

} PREFS;


extern PREFS Prefs;


int Prefs_Load(void);
void Prefs_Save(void);
void Prefs_SetDefaults(void);

void Prefs_Test(void);

int Prefs_CalculateChecksum( PREFS & PrefsStruct );

#endif
