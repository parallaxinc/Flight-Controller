/*
  This file is part of the ELEV-8 Flight Controller Firmware
  for Parallax part #80204, Revision A
  Version 1.0.2
  
  Copyright 2015 Parallax Incorporated

  ELEV-8 Flight Controller Firmware is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  ELEV-8 Flight Controller Firmware is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the ELEV-8 Flight Controller Firmware.  If not, see <http://www.gnu.org/licenses/>.
  
 
  Actively In Development / To Be Developed:
  - Altitude Hold - currently disabled
  - Heading Hold & Compass Calibratin 
  
  
  Written by Jason Dorie

  Dedicated to the memory of my father, Jim Dorie, who encouraged me endlessly
*/

#include <propeller.h>

#include "battery.h"            // Battery monitor functions (charge time to voltage)
#include "beep.h"               // Piezo beeper functions
#include "commlink.h"           // GroundStation communication link
#include "constants.h"          // Project-wide constants, like clock rate, update frequency
#include "elev8-main.h"         // Main thread functions and defines                            (Main thread takes 1 COG)
#include "f32.h"                // 32 bit IEEE floating point math and stream processor         (1 COG)
#include "intpid.h"             // Integer PID functions
#include "pins.h"               // Pin assignments for the hardware
#include "prefs.h"              // User preferences storage
#include "quatimu.h"            // Quaternion IMU and control functions
#include "rc.h"                 // High precision 8-port R/C PWM input driver                   (1 COG, if enabled)
#include "sbus.h"               // S-BUS (Futaba 1-wire receiver) driver                        (1 COG, if enabled)
#include "sensors.h"            // Sensors (gyro,accel,mag,baro) + LEDs driver                  (1 COG)
#include "serial_4x.h"          // 4 port simultaneous serial I/O                               (1 COG)
#include "servo32_highres.h"    // 32 port, high precision / high rate PWM servo output driver  (1 COG)

//#define ENABLE_LOGGING

#ifdef ENABLE_LOGGING
void DoLogOutput(void);
void DataLogThread(void *par);

#define LOG_STACK_SIZE (16 + 40)      // stack needs to accomodate thread control structure (40) plus room for functions (16)
static int log_stack[LOG_STACK_SIZE]; // allocate space for the stack - you need to set up a 

volatile char LogTrigger = 0;
#endif



short UsbPulse = 0;     // Periodically, the GroundStation will ping the FC to say it's still there - these are countdowns
short XBeePulse = 0;


// Potential new settings values
const int AltiThrottleDeadband = 100;


//Working variables for the flight controller

//Receiver inputs
static RADIO Radio;

static int  LoopCycles = 0;
static short CycleCount[8];   // Number of cycles an update loop takes, recorded over 8 cycles so we can get min/max/avg

static struct CYCLESTATS {
  short Version;
  short MinCycles;
  short MaxCycles;
  short AvgCycles;
} Stats;


//Sensor inputs, in order of outputs from the Sensors cog, so they can be bulk copied for speed
static SENS sens;


static long  GyroZX, GyroZY, GyroZZ;  // Gyro zero values

static long  AccelZSmooth;            // Smoothed (filtered) accelerometer Z value (used for height fluctuation damping)

//Debug output mode, working variables  
static long   counter = 0;    //Main loop iteration counter
static short  TxData[12];     //Word-sized copies of Temp, Gyro, Accel, Mag, for debug transfer speed
static char   Quat[16];       //Current quaternion from the IMU functions

static char Mode = MODE_None;         //Debug communication mode
static signed char NudgeMotor = -1;   // Which motor to nudge during testing (-1 == no motor)
static char NudgeCount[4];            // How long to spin the motor for (0 == stopped)
static int HostCommandUSB, HostCommandXBee;

static char AccelAssistZFactor  = 32; // 0 to 64 == 0 to 1.0

static long  AltiEst, AscentEst;                              // altitude estimate and ascent rate estimate
static long  DesiredAltitude, DesiredAscentRate;              // desired values for altitude and ascent rate

static long  RollDifference, PitchDifference, YawDifference;  //Delta between current measured roll/pitch and desired roll/pitch                         
static long  GyroRoll, GyroPitch, GyroYaw;                    //Raw gyro values altered by desired pitch & roll targets

static long  GyroRPFilter, GyroYawFilter;   // Tunable damping values for gyro noise


static short Motor[4];                     //Motor output values
static long  LEDValue[LED_COUNT];           //LED outputs (copied to the LEDs by the Sensors cog)

static long loopTimer;                      //Master flight loop counter - used to keep a steady update rate

static long PingZero, PingHeight;

static short FlightEnableStep;        //Flight arm/disarm counter
static short CompassConfigStep;       //Compass configure mode counter

static char FlightEnabled = 0;        //Flight arm/disarm flag
static char FlightMode;
static char IsHolding = 0;            // Are we currently in altitude hold? (hover mode)

static short StartupDelay;            //Used to change convergence rates for IMU, enable battery monitor

static char MotorPin[4] = {PIN_MOTOR_FL, PIN_MOTOR_FR, PIN_MOTOR_BR, PIN_MOTOR_BL };            //Motor index to pin index table

static long LEDModeColor;

static short BatteryVolts = 0;


static char calib_startQuadrant;
static char calib_Quadrants;
static char calib_step;
static long c_xmin, c_ymin, c_xmax, c_ymax, c_zmin, c_zmax;

// PIDs for roll, pitch, yaw, altitude
static IntPID  RollPID, PitchPID, YawPID, AltPID, AscentPID;


// Used to attenuate the brightness of the LEDs, if desired.  A shift of zero is full brightness
const int LEDBrightShift = 0;
const int LEDSingleMask = 0xFF - ((1<<LEDBrightShift)-1);
const int LEDBrightMask = LEDSingleMask | (LEDSingleMask << 8) | (LEDSingleMask << 16);


#ifdef ENABLE_LOGGING
// Only used for debugging
static char LogInt( int x , char * dest )
{
  char buf[14];
  char index = 13;
  buf[index] = ',';   // Add the comma here for speed
  char isNeg;
  if( x < 0 ) {
    isNeg = 1;
    x = -x;
  }    
  else isNeg = 0;

  do {
    buf[--index] = '0' + (x % 10);
    x /= 10;
  } while(x > 0);

  if( isNeg ) {
    buf[--index] = '-';
  }    

  char count = 14-index;
  memcpy( dest, buf+index, count );
  return count;
}
#endif


int main()                                    // Main function
{
  Initialize(); // Set up all the objects
  
  //Prefs_Test();

  //Grab the first set of sensor readings (should be ready by now)
  memcpy( &sens, Sensors_Address(), Sensors_ParamsSize );

  //Set a reasonable starting point for the altitude computation
  QuatIMU_SetInitialAltitudeGuess( sens.Alt );

  loopTimer = CNT;

  // Set all the motors to their low-throttle point
  for( int i=0; i<4; i++ ) {
    Motor[i] = Prefs.MinThrottle;
    Servo32_Set( MotorPin[i], Prefs.MinThrottle );
  }


  while(1)
  {
    int Cycles = CNT;

    //Read ALL inputs from the sensors into local memory, starting at Temperature
    memcpy( &sens, Sensors_Address(), Sensors_ParamsSize );

    QuatIMU_Update( (int*)&sens.GyroX );        //Entire IMU takes ~125000 cycles
    AccelZSmooth += (sens.AccelZ - AccelZSmooth) * Prefs.AccelCorrectionFilter / 256;

    if( Prefs.ReceiverType == 1 ) // SBUS?
    {
      // Unrolling these loops saves about 10000 cycles, but costs a little over 1/2kb of code space
      for( int i=0; i<8; i++ ) {
        Radio.Channel(i) =  (SBUS::GetRC(Prefs.ChannelIndex(i)) - Prefs.ChannelCenter(i)) * Prefs.ChannelScale(i) / 1024;
      }

      // Extra raw channel for SBUS users, tuning, experimentation
      Radio.Channel(8) =  ((SBUS::GetRC(8) + 32) * 1280) / 1024;  // Aux4
    }
    else
    {
      for( int i=0; i<8; i++ ) {
        Radio.Channel(i) =  (RC::GetRC( Prefs.ChannelIndex(i)) - Prefs.ChannelCenter(i)) * Prefs.ChannelScale(i) / 1024;
      }        
    }

    
    //if( UsePing )
    //  PingCycle := counter & 15
    //  if( PingCycle == 0 )
    //    Ping.Fire( PIN#PING )
    //  elseif( PingCycle == 15 )
    //    PingHeight := Ping.Millimeters( PIN#PING ) - PingZero 

      //-------------------------------------------------
    if( FlightMode == FlightMode_CalibrateCompass )
    {
      DoCompassCalibrate();
    }
    //-------------------------------------------------
    else
    //-------------------------------------------------
    {
      char NewFlightMode;

      if( Radio.Gear > 512 )
        NewFlightMode = FlightMode_Stable;    // Forward is "Stable"    (This was "Assist" but the altitude hold isn't working properly yet)
      else if( Radio.Gear < -512 )
        NewFlightMode = FlightMode_Manual;    // Back is "Manual"
      else
        NewFlightMode = FlightMode_Stable;    // Centered is "Stable"


      if( NewFlightMode != FlightMode )
      {
        if( NewFlightMode == FlightMode_Manual ) {
          QuatIMU_ResetDesiredOrientation();
        }
        else {
          QuatIMU_ResetDesiredYaw();          // Sync the heading when switching from manual to auto-level
        }

        if( NewFlightMode == FlightMode_Assist ) {
          DesiredAltitude = AltiEst;
        }

        // ANY flight mode change means you're not currently holding altitude
        IsHolding = 0;

        FlightMode = NewFlightMode;
      }

      UpdateFlightLoop();            //~72000 cycles when in flight mode
      //-------------------------------------------------
    }  


    if( Prefs.UseBattMon )
    {
      if( StartupDelay > 0 ) {
        StartupDelay--;       // Count down until the startup delay has passed
        LEDModeColor = LED_Blue;
        
        if( StartupDelay == 0 ) { // Did we JUST hit zero?
          QuatIMU_SetErrScaleMode(0);   // No longer in power-up (fast-convergence) mode
        }          
      }
      else
      {
        // Update the battery voltage
        switch( counter & 15 )
        {
          case 0: Battery::DischargePin();   break;
          case 2: Battery::ChargePin();      break;
    
          case 15:
            BatteryVolts = Battery::ComputeVoltage( Battery::ReadResult() ) + Prefs.VoltageOffset;
            break;
        }      
      }
    }

    All_LED( LEDModeColor );
    QuatIMU_WaitForCompletion();    // Wait for the IMU to finish updating


    QuatIMU_UpdateControls( &Radio , FlightMode == FlightMode_Manual );   // Now update the control quaternion
    QuatIMU_WaitForCompletion();

    PitchDifference = QuatIMU_GetPitchDifference();
    RollDifference = QuatIMU_GetRollDifference();
    YawDifference = -QuatIMU_GetYawDifference();


    AltiEst = QuatIMU_GetAltitudeEstimate();
    AscentEst = QuatIMU_GetVerticalVelocityEstimate();

    CheckDebugInput();
    DoDebugModeOutput();

#ifdef ENABLE_LOGGING
    DoLogOutput();
#endif

    LoopCycles = CNT - Cycles;    // Record how long it took for one full iteration
    CycleCount[counter & 7] = LoopCycles / 64;

    ++counter;
    loopTimer += Const_UpdateCycles;
    waitcnt( loopTimer );
  }
}


void Initialize(void)
{
  //Initialize everything - First reset all variables to known states

  FlightEnableStep = 0;                                 //Counter to know which section of enable/disable sequence we're in
  CompassConfigStep = 0;
  FlightMode = FlightMode_Stable;
  GyroRPFilter = 224;                                   //Tunable damping filters for gyro noise, 1 (HEAVY) to 256 (NO) filtering 
  GyroYawFilter = 224;
  Stats.Version = 0x0100;

  InitSerial();

  All_LED( LED_Red & LED_Half );                         //LED red on startup

  // Do this before settings are loaded, because Sensors_Start resets the drift coefficients to defaults
  Sensors_Start( PIN_SDI, PIN_SDO, PIN_SCL, PIN_CS_AG, PIN_CS_M, PIN_CS_ALT, PIN_LED, (int)&LEDValue[0], LED_COUNT );

  F32::Start();
  QuatIMU_Start();
  QuatIMU_SetErrScaleMode(1);   // Start with the IMU in fast-converge mode (takes ~3 instead of ~26 seconds to converge)

  InitializePrefs();
  InitReceiver();

  // Wait 2 seconds after startup to begin checking battery voltage, rounded to an integer multiple of 16 updates
  // Also used to reduce convergence rate for the IMU (starts up with a high convergence rate)
  StartupDelay = (Const_UpdateRate * 2) & ~15;

#ifdef __PINS_V3_H__
  Battery::Init( PIN_VBATT );
#endif

  DIRA |= (1<<PIN_BUZZER_1) | (1<<PIN_BUZZER_2);      //Enable buzzer pins
  OUTA &= ~((1<<PIN_BUZZER_1) | (1<<PIN_BUZZER_2));   //Set the pins low


  Servo32_Init( 400 );
  for( int i=0; i<4; i++ ) {
    Servo32_AddFastPin( MotorPin[i] );
    Servo32_Set( MotorPin[i], Prefs.MinThrottle );
  }    
  Servo32_Start();

  int RollPitch_P = 500;
  int RollPitch_D = 1560 * Const_UpdateRate;

  RollPID.Init( (RollPitch_P * (Prefs.RollGain+1)) >> 7, 0,  (RollPitch_D * (Prefs.RollGain+1)) >> 7 , Const_UpdateRate );
  RollPID.SetPrecision( 8 );
  RollPID.SetMaxOutput( 3000 );
  RollPID.SetPIMax( 100 );
  RollPID.SetMaxIntegral( 1900 );
  RollPID.SetDervativeFilter( 224 );


  PitchPID.Init( (RollPitch_P * (Prefs.PitchGain+1)) >> 7, 0,  (RollPitch_D * (Prefs.PitchGain+1)) >> 7 , Const_UpdateRate );
  PitchPID.SetPrecision( 8 );
  PitchPID.SetMaxOutput( 3000 );
  PitchPID.SetPIMax( 100 );
  PitchPID.SetMaxIntegral( 1900 );
  PitchPID.SetDervativeFilter( 224 );

  int YawP = (1200 * (Prefs.YawGain+1)) >> 7;
  int YawD = (625 * Const_UpdateRate * (Prefs.YawGain+1)) >> 7;

  YawPID.Init( YawP,  0,  YawD , Const_UpdateRate );
  YawPID.SetPrecision( 8 );
  YawPID.SetMaxOutput( 5000 );
  YawPID.SetPIMax( 100 );
  YawPID.SetMaxIntegral( 2000 );
  YawPID.SetDervativeFilter( 192 );

  int AltP = (900 * (Prefs.AltiGain+1)) >> 7;
  int AltI = (0 * (Prefs.AltiGain+1)) >> 7;

  // Altitude hold PID object
  // The altitude hold PID object feeds speeds into the vertical rate PID object, when in "hold" mode
  AltPID.Init( AltP, AltI, 0, Const_UpdateRate );
  AltPID.SetPrecision( 8 );
  AltPID.SetMaxOutput( 6000 );    // Fastest the altitude hold object will ask for is 6000 mm/sec (6 M/sec)
  AltPID.SetPIMax( 1000 );
  AltPID.SetMaxIntegral( 4000 );

  int AscentP = (300 * (Prefs.AscentGain+1)) >> 7;

  // Vertical rate PID object
  // The vertical rate PID object manages vertical speed in alt hold mode
  AscentPID.Init( AscentP, 0, 0 , Const_UpdateRate );
  AscentPID.SetPrecision( 8 );
  AscentPID.SetMaxOutput( 4000 );   // Limit of the control rate applied to the throttle
  AscentPID.SetPIMax( 500 );
  AscentPID.SetMaxIntegral( 2000 );


#ifdef ENABLE_LOGGING
  cogstart( &DataLogThread , NULL, log_stack, sizeof(log_stack) );
#endif
  
  
  FindGyroZero();
}


void InitReceiver(void)
{
  RC::Stop();
  SBUS::Stop();

  switch( Prefs.ReceiverType )
  {
    default:
    case 0:
      RC::Start(0);   // PWM mode
      break;

    case 2:
      RC::Start(1);   // PPM mode
      break;

    case 1:
      SBUS::Start( PIN_RC_0 );
      break;
  }
}


static char RXBuf1[32], TXBuf1[64];
static char RXBuf2[32], TXBuf2[64];

#if 0 // Currently unused - these buffers might grow later
static char RXBuf3[128],TXBuf3[4];  // GPS?
static char RXBuf4[4],  TXBuf4[64]; // Data Logger
#else
static char RXBuf3[4],  TXBuf3[4]; // GPS?
static char RXBuf4[4],  TXBuf4[4]; // Data Logger
#endif

void InitSerial(void)
{
  S4_Initialize();

  S4_Define_Port(0, 115200,      30, TXBuf1, sizeof(TXBuf1),      31, RXBuf1, sizeof(RXBuf1));
  S4_Define_Port(1,  57600, XBEE_TX, TXBuf2, sizeof(TXBuf2), XBEE_RX, RXBuf2, sizeof(RXBuf2));

  // Unused ports get a pin value of 32
  S4_Define_Port(2,   9600, 32, TXBuf3, sizeof(TXBuf3), 32, RXBuf3, sizeof(RXBuf3));

  S4_Define_Port(3, 115200, PIN_MOTOR_AUX2, TXBuf4, sizeof(TXBuf4), 32, RXBuf4, sizeof(RXBuf4));

  S4_Start();
}


static int clamp( int v, int min, int max ) {
  v = (v < min) ? min : v;
  v = (v > max) ? max : v;
  return v;
}

static int abs(int v) {
  v = (v<0) ? -v : v;
  return v;
}

static int max( int a, int b ) { return a > b ? a : b; }
static int min( int a, int b ) { return a < b ? a : b; }



void FindGyroZero(void)
{
  // The idea here is that it's VERY hard for someone to hold a thing perfectly still.
  // If we detect variation in the gyro, keep waiting until it settles down, or "too long" has elapsed.

  int vmin[3], vmax[3], avg[3];     // min, max, avg readings for each gyro axis
  int best[3], bestvar = -1;        // best set of readings found so far, and the variance for them

  int TryCounter = 0;
  const int MinTries = 2, MaxTries = 64;

  // Wait for any buzzer vibration to stop.  Yes, this is actually necessary, it can be that sensitive.
  waitcnt( CNT + Const_ClockFreq/50 );

  do {
    // Take an initial sensor reading for each axis as a starting point, and zero the average
    for( int a=0; a<3; a++) {
      vmin[a] = vmax[a] = Sensors_In(1+a);
      avg[a] = 0;
    }

    // take a bunch of readings over about 1/10th of a second, keeping track of the min, max, and sum (average)
    for( int i=0; i<64; i++ )
    {
      for( int a=0; a<3; a++) {
        int v = Sensors_In(1+a);
        vmin[a] = min(vmin[a], v);
        vmax[a] = max(vmax[a], v);
        avg[a] += v;
      }

      waitcnt( CNT + Const_ClockFreq/500 );
    }

    // Compute the mid-point between the min & max, and how different that is from the average (variation)
    int maxVar = 0;
    for( int a=0; a<3; a++)
    {
      avg[a] /= 64;

      // range is the difference between min and max over the sample period.
      // I measured this as ~15 units on all axis when totally still
      int range = vmax[a] - vmin[a];

      // variation is how centered the average is between the min and max.
      // if the craft is perfectly still, this *should* be zero or VERY close.
      int var = (vmax[a]+vmin[a])/2 - avg[a];

      maxVar = max( maxVar, abs(var) );
    }

    if( (bestvar == -1) || (maxVar < bestvar) ) {
      best[0] = avg[0];
      best[1] = avg[1];
      best[2] = avg[2];
      bestvar = maxVar;
    }

    // Every 4th loop, beep at the user to tell them what's happening
    if( (TryCounter & 3) == 3 ) {
      BeepHz( 4000, 80 );
    }

    TryCounter++;

    // Run at least MinTries iterations, wait until max variance is 2 or less, give up after MaxTries
  } while( TryCounter < MaxTries && (bestvar > 2 || TryCounter < MinTries) );

  GyroZX = best[0];
  GyroZY = best[1];
  GyroZZ = best[2];

  QuatIMU_SetGyroZero( GyroZX, GyroZY, GyroZZ );
}


void UpdateCycleStats(void)
{
  // Prime the initial values
  int avg;
  Stats.MinCycles = Stats.MaxCycles = avg = CycleCount[0];

  for( int i=1; i<8; i++ )
  {
    Stats.MinCycles = min( Stats.MinCycles, CycleCount[i] );
    Stats.MaxCycles = max( Stats.MaxCycles, CycleCount[i] );
    avg += CycleCount[i];
  }
  Stats.AvgCycles = avg >> 3;
}

void UpdateFlightLoop(void)
{
  int ThroOut, ThrustMul, AltiThrust, v, gr, gp, gy;
  char DoIntegrate;  //Integration enabled in the flight PIDs?

  UpdateFlightLEDColor();

  //Test for flight mode change-----------------------------------------------
  if( FlightEnabled == 0 )
  {
    //Are the sticks being pushed down and toward the center?

    if( (Radio.Thro < -750)  &&  (Radio.Elev < -750) )
    {
      if( (Radio.Rudd > 750)  &&  (Radio.Aile < -750) )
      {
        FlightEnableStep++;
        CompassConfigStep = 0;
        LEDModeColor = LED_Yellow & LED_Half;
                
        if( FlightEnableStep >= Prefs.ArmDelay ) {   //Hold for delay time
          ArmFlightMode();
        }          
      }
      /* // Compass calibration not enabled yet
      else if( (Radio.Rudd > 750)  &&  (Radio.Aile > 750) )
      {
        CompassConfigStep++;
        FlightEnableStep = 0;

        LEDModeColor = (LED_Blue | LED_Red) & LED_Half;

        if( CompassConfigStep == 250 ) {   //Hold for 1 second
          StartCompassCalibrate();
        }
      }
      */
      else
      {
        CompassConfigStep = 0;
        FlightEnableStep = 0;
      }
    }
    else
    {
      CompassConfigStep = 0;
      FlightEnableStep = 0;
    }
    //------------------------------------------------------------------------
  }       
  else
  {
    //Are the sticks being pushed down and away from center?

    if( (Radio.Rudd < -750)  &&  (Radio.Aile > 750)  &&  (Radio.Thro < -750)  &&  (Radio.Elev < -750) )
    {
      FlightEnableStep++;
      LEDModeColor = LED_Yellow & LED_Half;

      if( FlightEnableStep >= Prefs.DisarmDelay ) {   //Hold for delay time
        DisarmFlightMode();
        return;                  //Prevents the motor outputs from being un-zero'd
      }        
    }      
    else {
      FlightEnableStep = 0;
    }
    //------------------------------------------------------------------------


    gr =  sens.GyroY - GyroZY;
    gp = -(sens.GyroX - GyroZX);
    gy = -(sens.GyroZ - GyroZZ);

    // TODO: Flight test - is this filtering necessary?
    GyroRoll += ((gr - GyroRoll) * GyroRPFilter) >> 8;
    GyroPitch += ((gp - GyroPitch) * GyroRPFilter) >> 8;
    GyroYaw += ((gy - GyroYaw) * GyroYawFilter) >> 8;



    if( Radio.Thro < -900 )
    {
      // When throttle is essentially zero, disable all control authority

      if( FlightMode == FlightMode_Manual ) {
        QuatIMU_ResetDesiredOrientation();
      }
      else {
        // Zero yaw target when throttle is off - makes for more stable liftoff
        QuatIMU_ResetDesiredYaw();
      }

      DoIntegrate = 0;          // Disable PID integral terms until throttle is applied      
    }      
    else {
      DoIntegrate = 1;
    }


    int RollOut = RollPID.Calculate( RollDifference , GyroRoll , DoIntegrate );
    int PitchOut = PitchPID.Calculate( PitchDifference , GyroPitch , DoIntegrate );
    int YawOut = YawPID.Calculate( YawDifference, GyroYaw, DoIntegrate );


    int ThroMix = (Radio.Thro + 1024) >> 1;           // Approx 0 - 1024
    ThroMix = clamp( ThroMix, 0, 64 );                // Above 1/16 throttle, clamp it to 64
     
    //add 12000 to all Output values to make them 'servo friendly' again   (12000 is our output center)
    ThroOut = (Radio.Thro << 2) + 12000;

    //-------------------------------------------
    if( FlightMode != FlightMode_Manual )
    {
      /*  // Altitude hold is currently disabled becuase it's not working right yet - I suspect a bug in the altitude estimate

      if( FlightMode == FlightMode_Assist )
      {
        //int T0 = max( 0, (Radio.Aux1 + 1024) >> 2);
        //int T1 = max( 0, (Radio.Aux2 + 1024));
        //int T2 = max( 0, (Radio.Aux3 + 1024) >> 1);

        //AscentPID.SetPGain( T0 );
        //AltPID.SetPGain( T1 );
        //AltPID.SetIGain( T2 );

        int AdjustedThrottle = 0;

        // Throttle has to be off zero by a bit - deadband around zero helps keep it still
        if( abs(Radio.Thro) > AltiThrottleDeadband )
        {
          IsHolding = 0;    // No longer attempting to hold a hover

          // Remove the deadband area from center stick so we don't get a hiccup as you transition out of it
          AdjustedThrottle = (Radio.Thro > 0) ? (Radio.Thro - AltiThrottleDeadband) : (Radio.Thro + AltiThrottleDeadband);

          // 6 m/sec maximum rate of ascent/descent
          DesiredAscentRate = AdjustedThrottle * 6000 / (1024 - AltiThrottleDeadband);
        }
        else
        {
          // Are we just entering altitude hold mode?
          if( IsHolding == 0 ) {
            IsHolding = 1;
            DesiredAltitude = AltiEst;  // Start with our current altitude as the hold height
          }

          // Use a PID object to compute velocity requirements for the AscentPID object
          DesiredAscentRate = AltPID.Calculate( DesiredAltitude , AltiEst, DoIntegrate );
        }

        AltiThrust = AscentPID.Calculate( DesiredAscentRate , AscentEst , DoIntegrate );
        ThroOut = Prefs.CenterThrottle + AltiThrust + (AdjustedThrottle<<1); // Feed in a bit of the user throttle to help with quick throttle changes
      }*/

      if( AccelAssistZFactor > 0 )
      {
        // Accelerometer assist - add a little AccelZ into the mix if the user is trying to hover
        if( abs(Radio.Aile) < 300 && abs(Radio.Elev) < 300 && abs(Radio.Thro) < AltiThrottleDeadband ) {
          ThroOut -= ((AccelZSmooth - Const_OneG) * (int)AccelAssistZFactor) / 64;
        }
      }

      if( Prefs.ThrustCorrectionScale > 0 )
      {
        // Tilt compensated thrust assist
        ThrustMul = 256 + ((QuatIMU_GetThrustFactor() - 256) * Prefs.ThrustCorrectionScale) / 256;
        ThrustMul = clamp( ThrustMul, 256 , 384 );    //Limit the effect of the thrust modifier
        ThroOut = Prefs.MinThrottle + (((ThroOut-Prefs.MinThrottle) * ThrustMul) >> 8);
      }        
    }
    //-------------------------------------------


    //X configuration
    Motor[OUT_FL] = ThroOut + (((+PitchOut + RollOut - YawOut) * ThroMix) >> 7);
    Motor[OUT_FR] = ThroOut + (((+PitchOut - RollOut + YawOut) * ThroMix) >> 7);
    Motor[OUT_BL] = ThroOut + (((-PitchOut + RollOut + YawOut) * ThroMix) >> 7);
    Motor[OUT_BR] = ThroOut + (((-PitchOut - RollOut - YawOut) * ThroMix) >> 7);


    // The low-throttle clamp prevents combined PID output from sending the ESCs below a minimum value
    // Some ESCs appear to stall (go into "stop" mode) if the throttle gets too close to zero, even for a moment, so avoid that

    Motor[0] = clamp( Motor[0], Prefs.MinThrottleArmed , Prefs.MaxThrottle);
    Motor[1] = clamp( Motor[1], Prefs.MinThrottleArmed , Prefs.MaxThrottle);
    Motor[2] = clamp( Motor[2], Prefs.MinThrottleArmed , Prefs.MaxThrottle);
    Motor[3] = clamp( Motor[3], Prefs.MinThrottleArmed , Prefs.MaxThrottle);

    if( Prefs.DisableMotors == 0 ) {
      //Copy new Ouput array into servo values
      Servo32_Set( PIN_MOTOR_FL, Motor[0] );
      Servo32_Set( PIN_MOTOR_FR, Motor[1] );
      Servo32_Set( PIN_MOTOR_BR, Motor[2] );
      Servo32_Set( PIN_MOTOR_BL, Motor[3] );
    }
  }

#if defined( __PINS_V3_H__ )
    // Battery alarm at low voltage
    if( Prefs.UseBattMon != 0 && Prefs.LowVoltageAlarm != 0 )
    {
      // If we want to use the PING sensor *and* use a timer for the alarm, we'll need to
      // move the freq generator onto another cog.  Currently the battery monitor uses CTRB
      // to count charge time.  Ideally the PING sensor would use CTRA to count return time,
      // so we can have one or the other in the main thread, but not both.

      if( (BatteryVolts < Prefs.LowVoltageAlarmThreshold) && (BatteryVolts > 200) && ((counter & 63) == 0) )  // Make sure the voltage is above the (0 + VoltageOffset) range
      {
        BeepOn( 'A' , PIN_BUZZER_1, 5000 );
      }
      else if( (counter & 63) > 32 )
      {
        BeepOff( 'A' );
      }        
    }      
#endif
}


static int LEDColorTable[] = {
        /* LED_Assisted */     LED_White,
        /* LED_Stable  */      LED_Cyan,
        /* LED_Manual    */    LED_Yellow,
        /* LED_CompCalib */    LED_Violet,
};

static int LEDArmDisarm[] = {
        /* LED_Disarmed */     LED_Green,
        /* LED_Armed    */     LED_Red,
};


void UpdateFlightLEDColor(void)
{
  int LowBatt = 0;

#if defined( __PINS_V3_H__ )
  if( Prefs.UseBattMon != 0 && (BatteryVolts < Prefs.LowVoltageAlarmThreshold) && BatteryVolts > 200 ) {   // Make sure the voltage is above the (0 + VoltageOffset) range
    LowBatt = 1;
  }    
#endif

  if( LowBatt ) {

    int index = (counter >> 3) & 7;

    if( index < 4 ) {
      LEDModeColor = (LEDColorTable[FlightMode & 3] & LEDBrightMask) >> LEDBrightShift;
    }
    else {
      LEDModeColor = ((LED_Red | (LED_Yellow & LED_Half)) & LEDBrightMask) >> LEDBrightShift;   // Fast flash orange for battery warning
    }

  }
  else {
    int index = (counter >> 3) & 15;
    if( index < 3 || IsHolding ) {  // Temporary, so I can tell
      LEDModeColor = (LEDColorTable[FlightMode & 3] & LEDBrightMask) >> LEDBrightShift;
    }    
    else {
      LEDModeColor = (LEDArmDisarm[FlightEnabled & 1] & LEDBrightMask) >> LEDBrightShift;
    }
  }
}


void ArmFlightMode(void)
{
  FlightEnabled = 1;
  FlightEnableStep = 0;
  CompassConfigStep = 0;
  Beep2();
   
  All_LED( LED_Red & LED_Half );
  FindGyroZero();
   
  All_LED( LED_Blue & LED_Half );
  BeepTune();

  DesiredAltitude = AltiEst;
  loopTimer = CNT;
}

void DisarmFlightMode(void)
{
  for( int i=0; i<4; i++ ) {
    Motor[i] = Prefs.MinThrottle;
    Servo32_Set( MotorPin[i], Prefs.MinThrottle );
  }

  FlightEnabled = 0;
  FlightEnableStep = 0;
  CompassConfigStep = 0;
  Beep3();
  
  All_LED( LED_Green & LED_Half );
  loopTimer = CNT;
}

void StartCompassCalibrate(void)
{
  // Placeholder
}

void DoCompassCalibrate(void)
{
  // Placeholder
}


void CheckDebugInput(void)
{
  int i, HostCommand;

  char port = 0;
  int c = S4_Check(0);
  if(c < 0) {
    c = S4_Check(1);
    if(c < 0)
      return;
    port = 1;
    HostCommandXBee = (HostCommandXBee<<8) | c;
    HostCommand = HostCommandXBee;
  }
  else {
    HostCommandUSB = (HostCommandUSB<<8) | c;
    HostCommand = HostCommandUSB;
  }    

  if( HostCommand == Comm_Beat )
  {
    Mode = MODE_SensorTest;
    if( port == 0 ) {
      UsbPulse = 500;   // send USB data for the next two seconds (we'll get another heartbeat before then)
      XBeePulse = 0;
    }
    if( port == 1 ) {
      UsbPulse = 0;
      XBeePulse = 500;  // send XBee data for the next two seconds (we'll get another heartbeat before then)
    }
    return;
  }

  if( HostCommand == Comm_Elv8 ) {
    S4_Put_Bytes( port, &HostCommand , 4 );     //Simple ping-back to tell the application we have the right comm port
    return;
  }

  if( FlightEnabled ) return; // Don't allow any settings adjustment when in-flight


  switch( HostCommand )
  {
    case Comm_Motor1:
    case Comm_Motor2:
    case Comm_Motor3:
    case Comm_Motor4:
    case Comm_Motor5:
    case Comm_Motor6:
    case Comm_Motor7:
      NudgeMotor = (HostCommand&255) - '1';    // Becomes an index from 0 to 5, 0 to 3 are motors, 4 is LED, 5 is beeper
      if( NudgeMotor < 4 ) {
        NudgeCount[NudgeMotor] = 50;  // 1/5th of a second at 250 updates per sec
        NudgeMotor = -1;
      }
      break;

    case Comm_ResetRadio:
      for( int i=0; i<8; i++ ) {
        Prefs.ChannelScale(i) = 1024;
        Prefs.ChannelCenter(i) = 0;
      }
      Beep2();
      break;

    case Comm_ZeroGyro:
      // temporarily zero gyro drift settings
      Sensors_TempZeroDriftValues();
      break;

    case Comm_ResetGyro:
      // restore previous settings
      Sensors_ResetDriftValues();
      break;

    case Comm_ZeroAccel:
      //Temporarily zero accel offset settings
      Sensors_TempZeroAccelOffsetValues();
      break;

    case Comm_ResetAccel:
      //Reset accel offset settings
      Sensors_ResetAccelOffsetValues();
      break;


    case Comm_QueryPrefs:  // Query Preferences
      {
      Prefs.Checksum = Prefs_CalculateChecksum( Prefs );
      int size = sizeof(Prefs);

      COMMLINK::StartPacket( 0, 0x18 , size );
      COMMLINK::AddPacketData( 0, &Prefs, size );
      COMMLINK::EndPacket(port);
      }
      break;

    case Comm_SetPrefs:  //Store new preferences
      {
      PREFS TempPrefs;
      for( int i=0; i<sizeof(Prefs); i++ ) {
        ((char *)&TempPrefs)[i] = S4_Get_Timed(port, 50);   // wait up to 50ms per byte - Should be plenty
      }

      if( Prefs_CalculateChecksum( TempPrefs ) == TempPrefs.Checksum ) {
        memcpy( &Prefs, &TempPrefs, sizeof(Prefs) );
        Prefs_Save();

        if( Prefs_Load() ) {
          BeepOff( 'A' );   // turn off the alarm beeper if it was on
          Beep2();
          ApplyPrefs();
          InitReceiver();   // In case the user changes receiver types
          FindGyroZero();   // Prevents the IMU from wandering around when we change gyro or accel offsets
        }
        else {
          Beep();
        }
      }
      else {
        Beep();
      }
      }
      break;

    case Comm_Wipe: // Default prefs - wipe
      Prefs_SetDefaults();
      Prefs_Save();
      Beep3();
      break;

    default:
      return;
  }

  loopTimer = CNT;                                                          //Reset the loop counter in case we took too long 
}

void DoDebugModeOutput(void)
{
  int loop, addr, i, phase;
  char port = 0;

  if( UsbPulse > 0 ) {
    if( --UsbPulse == 0 ) {
      Mode = MODE_None;
      return;
    }
    phase = counter & 7;    // Translates to 31.25 full updates per second, at 250hz
  }
  else if( XBeePulse > 0 )
  {
    if( --UsbPulse == 0 ) {
      Mode = MODE_None;
      return;
    }
    port = 1;
    phase = ((counter >> 1) & 7) | ((counter & 1) << 16);    // Translates to ~15 full updates per second, at 250hz
  }    

  if( Mode == MODE_None ) return;

  switch( Mode )
  {
    case MODE_SensorTest:
    {
      switch( phase )
      {
      case 0:
        COMMLINK::StartPacket( 1, 18 );               // Radio values, 18 byte payload
        COMMLINK::AddPacketData( &Radio , 16 );       // First 8 channels of Radio struct is 16 bytes total
        COMMLINK::AddPacketData( &BatteryVolts, 2 );  // Send 2 additional bytes for battery voltage
        COMMLINK::EndPacket();
        COMMLINK::SendPacket(port);
        break;

      case 1:
        UpdateCycleStats();
        COMMLINK::StartPacket( 7, 12 );                // Debug values, 12 byte payload
        COMMLINK::AddPacketData( &Stats, 8 );          // Version number, + Stats on update cycle counts (sending debug data takes a long time)
        COMMLINK::AddPacketData( &counter, 4 );        // Send the counter (sequence timestamp)
        COMMLINK::EndPacket();
        COMMLINK::SendPacket(port);
        break;

      case 2:
        TxData[0] = sens.Temperature;       //Copy the values we're interested in into a WORD array, for faster transmission                        
        TxData[1] = sens.GyroX;
        TxData[2] = sens.GyroY;
        TxData[3] = sens.GyroZ;
        TxData[4] = sens.AccelX;
        TxData[5] = sens.AccelY;
        TxData[6] = sens.AccelZ;
        TxData[7] = sens.MagX;
        TxData[8] = sens.MagY;
        TxData[9] = sens.MagZ;

        COMMLINK::BuildPacket( 2, &TxData, 20 );   //Send 22 bytes of data from @TxData onward (sends 11 words worth of data)
        COMMLINK::SendPacket(port);
        break;

      case 4:
        COMMLINK::BuildPacket( 3, QuatIMU_GetQuaternion(), 16 );  // Quaternion data, 16 byte payload
        COMMLINK::SendPacket(port);
        break;

      case 5: // Motor data
        COMMLINK::BuildPacket( 5, &Motor[0], 8 );   // 8 byte payload
        COMMLINK::SendPacket(port);
        break;


      case 6:
        COMMLINK::StartPacket( 4, 24 );   // Computed data, 24 byte payload

        COMMLINK::AddPacketData( &PitchDifference, 4 );
        COMMLINK::AddPacketData( &RollDifference, 4 );
        COMMLINK::AddPacketData( &YawDifference, 4 );

        COMMLINK::AddPacketData( &sens.Alt, 4 );       //Send 4 bytes of data for @Alt
        COMMLINK::AddPacketData( &sens.AltTemp, 4 );   //Send 4 bytes of data for @AltTemp
        COMMLINK::AddPacketData( &AltiEst, 4 );        //Send 4 bytes for altitude estimate 
        COMMLINK::EndPacket();
        COMMLINK::SendPacket(port);
        break;

      case 7:
        QuatIMU_GetDesiredQ( (float*)TxData );
        COMMLINK::BuildPacket( 6, TxData, 16 );   // Desired Quaternion data, 16 byte payload
        COMMLINK::SendPacket(port);
        break;
      }
    }
    break;
  }


  //Motor test code---------------------------------------
  if( FlightEnabled ) return;   // Don't run this code in flight - dangerous

  // Check to see if any motors are supposed to test-spin
  for( char m=0; m<4; m++ )
  {
    if( NudgeCount[m] ) {
      if( --NudgeCount[m] > 0 ) {
        Servo32_Set(MotorPin[m], Prefs.ThrottleTest);          // Motor test - use the configured throttle test value
      }
      else {       
        Servo32_Set(MotorPin[m], Prefs.MinThrottle);           // Back to zero throttle when the timer expires
      }        
    }
  }


  if( NudgeMotor > -1 )
  {
    if( NudgeMotor == 4 )                                             //Buzzer test
    {
      BeepHz(4500, 50);
      waitcnt( CNT + 5000000 );
      BeepHz(3500, 50);
    }
    else if( NudgeMotor == 5 )                                        //LED test
    {
      //RGB led will run a rainbow
      for( i=0; i<256; i++ ) {
        All_LED( ((255-i)<<16) + (i<<8) );
        waitcnt( CNT + 160000 );
      }

      for( i=0; i<256; i++ ) {
        All_LED( i + ((255-i) << 8) );
        waitcnt( CNT + 160000 );
      }

      for( i=0; i<256; i++ ) {
        All_LED( (255-i) + (i<<16) );
        waitcnt( CNT + 160000 );
      }          
    }
    else if( NudgeMotor == 6 )                                        //ESC Throttle calibration
    {
      BeepHz(4500, 100);
      waitcnt( CNT + 5000000 );
      BeepHz(4500, 100);
      waitcnt( CNT + 5000000 );   // 4 beeps - Throttle calibration waiting power-on command
      BeepHz(4500, 100);
      waitcnt( CNT + 5000000 );
      BeepHz(4500, 100);

      if( S4_Get(0) == 0xFF )     // Safety check - Allow the user to break out by sending anything else                  
      {
        for( int i=0; i<4; i++ ) {
          Servo32_Set(MotorPin[i], Prefs.MaxThrottle);
        }

        S4_Get(0);  // Get the next character to finish

        for( int i=0; i<4; i++ ) {
          Servo32_Set(MotorPin[i], Prefs.MinThrottle);  // Must add 64 to min throttle value (in this calibration code only) if using ESCs with BLHeli version 14.0 or 14.1
        }

        Beep2();                  // Throttle calibration successful
      }
      else {
        BeepHz(3000,500);         // Throttle calibration cancelled - 1/2 second lower tone
      }        
    }        

    NudgeMotor = -1;
    loopTimer = CNT;
  }
  //End Motor test code-----------------------------------
}

#ifdef ENABLE_LOGGING
void DoLogOutput(void)
{
  if( FlightEnabled == 0 ) return;
  LogTrigger = 1;
}


void DataLogThread(void *par)
{
  char phase = 0;

  while( true )
  {
    while( LogTrigger == 0 )
      ;

    if( phase == 0 ) {
      // Probably better to write this to just dump raw int, float, etc.
      // MUCH faster, smaller, and the PC can extract it relatively easily
    
      static char tempBuf[128];
      char count = 0;

      count =  LogInt( DesiredAscentRate , tempBuf+count );
      count += LogInt( AscentEst , tempBuf+count );
      count += LogInt( AscentPID.LastPError , tempBuf+count );
      count += LogInt( AscentPID.IError , tempBuf+count );
      count += LogInt( AscentPID.Output , tempBuf+count );
      count += LogInt( DesiredAltitude , tempBuf+count );
      count += LogInt( AltiEst, tempBuf+count );
      count += LogInt( AltPID.LastPError , tempBuf+count );
      count += LogInt( AltPID.IError , tempBuf+count );
      count += LogInt( FlightMode , tempBuf+count);

      count += LogInt( Radio.Thro , tempBuf+count );
      tempBuf[count++] = 13;  // overwrite the last comma with a carriage return
    
      S4_Put_Bytes( 3, tempBuf, count );
    }

    phase ^= 1; // Cut the data rate down a little

    // Reset the log trigger
    LogTrigger = 0;
  }    
}
#endif


void InitializePrefs(void)
{
  Prefs_Load();
  ApplyPrefs();
}  

void ApplyPrefs(void)
{
  Sensors_SetDriftValues( &Prefs.DriftScale[0] );
  Sensors_SetAccelOffsetValues( &Prefs.AccelOffset[0] );
  Sensors_SetMagnetometerScaleOffsets( &Prefs.MagScaleOfs[0] );

  QuatIMU_SetRollCorrection( &Prefs.RollCorrect[0] );
  QuatIMU_SetPitchCorrection( &Prefs.PitchCorrect[0] );

  QuatIMU_SetAutoLevelRates( Prefs.AutoLevelRollPitch , Prefs.AutoLevelYawRate );
  QuatIMU_SetManualRates( Prefs.ManualRollPitchRate , Prefs.ManualYawRate );

#ifdef FORCE_SBUS
  Prefs.UseSBUS = 1;
#endif

#if defined( __V2_PINS_H__ )  // V2 hardware doesn't support the battery monitor
  Prefs.UseBattMon = 0;
#endif
}


void All_LED( int Color )
{
  for( int i=0; i<LED_COUNT; i++ )
    LEDValue[i] = Color;
}  

