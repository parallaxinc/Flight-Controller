/*
  Elev8 Flight Controller
*/

#include <propeller.h>
#include <fdserial.h>

#include "battery.h"
#include "beep.h"
#include "constants.h"
#include "eeprom.h"
#include "elev8-main.h"
#include "f32.h"
#include "intpid.h"
#include "pins.h"
#include "prefs.h"
#include "quatimu.h"
#include "rc.h"
#include "sbus.h"
#include "sensors.h"
#include "servo32_highres.h"

fdserial *dbg;
fdserial_st *dbg_st;
char* dbg_txbuf;


// Potential new settings values
const int AltiThrottleDeadband = 150;


//Working variables for the flight controller

//Receiver inputs
static RADIO Radio;
static long  iRudd;         //Integrated rudder input value
static long  LoopCycles = 0;
//Sensor inputs, in order of outputs from the Sensors cog, so they can be bulk copied for speed
static SENS sens;


static long  GyroZX, GyroZY, GyroZZ;  // Gyro zero values

static long  AccelXSmooth, AccelYSmooth;
static long  AccelZSmooth;            // Smoothed (filtered) accelerometer Z value (used for height fluctuation damping)

//Debug output mode, working variables  
static long   counter;        //Main loop iteration counter
static short  TxData[12];     //Word-sized copies of Temp, Gyro, Accel, Mag, for debug transfer speed
static char   Quat[16];       //Current quaternion from the IMU functions

static char Mode;                     //Debug communication mode
static signed char NudgeMotor;        // Which motor to nudge during testing (-1 == no motor)

static char AccelAssistZFactor  = 32; // 0 to 64 == 0 to 1.0

static long  AltiEst, AscentEst;

//Working variables - used to convert receiver inputs to desired ranges for the PIDs  
static long  DesiredAltitude, DesiredAltitudeFractional;

static long  RollDifference, PitchDifference, YawDifference;  //Delta between current measured roll/pitch and desired roll/pitch                         
static long  GyroRoll, GyroPitch, GyroYaw;                    //Raw gyro values altered by desired pitch & roll targets

static long  GyroRPFilter, GyroYawFilter;   // Tunable damping values for gyro noise


static long  Motor[4];                      //Motor output values  
static long  LEDValue[LED_COUNT];           //LED outputs (copied to the LEDs by the Sensors cog)

static long loopTimer;                      //Master flight loop counter - used to keep a steady update rate

static long PingZero, PingHeight;

static short FlightEnableStep;        //Flight arm/disarm counter
static short CompassConfigStep;       //Compass configure mode counter

static char FlightEnabled;            //Flight arm/disarm flag
static char FlightMode;

static int  BatteryMonitorDelay;      //Can't start discharging the battery monitor cap until the ESCs are armed or the noise messes them up

static char MotorPin[4];            //Motor index to pin index table


static long RollPitch_P, RollPitch_D; //Tunable values for roll & pitch Proportional gain and Derivative gain
static long LEDModeColor;

static short BatteryVolts = 0;


static char calib_startQuadrant;
static char calib_Quadrants;
static char calib_step;
static long c_xmin, c_ymin, c_xmax, c_ymax, c_zmin, c_zmax;

// PIDs for roll, pitch, yaw, altitude
static IntPID  RollPID, PitchPID, YawPID, AscentPID;


// Used to attenuate the brightness of the LEDs, if desired.  A shift of zero is full brightness
const int LEDBrightShift = 0;
const int LEDSingleMask = 0xFF - ((1<<LEDBrightShift)-1);
const int LEDBrightMask = LEDSingleMask | (LEDSingleMask << 8) | (LEDSingleMask << 16);



// BEWARE - This function bypasses ALL safety checks for buffer throughput.
// BE VERY CAREFUL that you don't send more than is possible at one time.
static void TxUnsafe( char c ) {
  dbg_txbuf[ dbg_st->tx_head ] = c;
  dbg_st->tx_head = (dbg_st->tx_head+1) & FDSERIAL_BUFF_MASK;
}

static void TxBulkUnsafe( void * data , int Bytes )
{
  char * buf = (char *)data;
  int head = dbg_st->tx_head;

  while(Bytes > 0)
  {
    int bufRemain = (FDSERIAL_BUFF_MASK+1) - head;
    if( bufRemain > Bytes ) bufRemain = Bytes;

    memcpy( dbg_txbuf+head, buf, bufRemain );
    Bytes -= bufRemain;
    buf += bufRemain;
    head = (head + bufRemain) & FDSERIAL_BUFF_MASK;
  }  
  dbg_st->tx_head = head;
}

// These versions are safe to call, and properly wait for the buffer to have space for data
static void Tx( char c ) {
  fdserial_txChar( dbg, c );
}  

static void TxBulk( const void * buf , int bytes )
{
  const char *data = (const char *)buf;
  for( int i=0; i<bytes; i++ ) {
    Tx( data[i] );
  }    
}

static void TxInt( int x , int Digits )
{
  static char nybbles[] = "0123456789ABCDEF";
  int shift = 32 - (4 + (Digits<<2));
  do {
    Tx( nybbles[ (x>>shift) & 0xf] );
    shift -= 4;
  } while( shift >= 0 );
}



int main()                                    // Main function
{
  Initialize(); // Set up all the objects
  
  //Prefs_Test();

  //Grab the first set of sensor readings (should be ready by now)
  memcpy( &sens, Sensors_Address(), Sensors_ParamsSize );

  //Set a reasonable starting point for the altitude computation
  QuatIMU_SetInitialAltitudeGuess( sens.Alt );

  Mode = MODE_None;
  counter = 0;
  NudgeMotor = -1;
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

    AccelXSmooth += (sens.AccelX - AccelXSmooth) * Prefs.AccelCorrectionFilter / 256;
    AccelYSmooth += (sens.AccelY - AccelYSmooth) * Prefs.AccelCorrectionFilter / 256;
    AccelZSmooth += (sens.AccelZ - AccelZSmooth) * Prefs.AccelCorrectionFilter / 256;

    if( Prefs.UseSBUS )
    {
      // Unrolling these loops saves about 10000 cycles, but costs a little over 1/2kb of code space
      for( int i=0; i<8; i++ ) {
        Radio.Channel(i) =  (SBUS::GetRC(Prefs.ChannelIndex(i)) - Prefs.ChannelCenter(i)) * Prefs.ChannelScale(i) / 1024;
      }
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
        NewFlightMode = FlightMode_Assisted;
      else if( Radio.Gear < -512 )
        NewFlightMode = FlightMode_Manual;
      else
        NewFlightMode = FlightMode_Automatic;


      if( NewFlightMode != FlightMode )
      {
        if( NewFlightMode != FlightMode_Manual ) {
          QuatIMU_ResetDesiredOrientation();
          QuatIMU_ResetDesiredYaw();          // Sync the heading when switching from manual to auto-level
        }

        if( NewFlightMode == FlightMode_Automatic ) {
          DesiredAltitude = AltiEst;
        }

        FlightMode = NewFlightMode;
      }

      UpdateFlightLoop();            //~72000 cycles when in flight mode
      //-------------------------------------------------
    }  


    if( Prefs.UseBattMon )
    {
      if( BatteryMonitorDelay > 0 ) {
        BatteryMonitorDelay--;  // Count down until the startup delay has passed
        LEDModeColor = LED_Blue;
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

    LoopCycles = CNT - Cycles;    // Record how long it took for one full iteration

    ++counter;
    loopTimer += Const_UpdateCycles;
    waitcnt( loopTimer );
  }
}


void Initialize(void)
{
  //Initialize everything - First reset all variables to known states

  MotorPin[0] = PIN_MOTOR_FL;
  MotorPin[1] = PIN_MOTOR_FR;
  MotorPin[2] = PIN_MOTOR_BR;
  MotorPin[3] = PIN_MOTOR_BL;

  Mode = MODE_None;
  counter = 0;
  NudgeMotor = -1;                                      //No motor to nudge
  FlightEnabled = 0; 

  FlightEnableStep = 0;                                 //Counter to know which section of enable/disable sequence we're in
  CompassConfigStep = 0;
  FlightMode = FlightMode_Assisted;
  GyroRPFilter = 192;                                   //Tunable damping filters for gyro noise, 1 (HEAVY) to 256 (NO) filtering 
  GyroYawFilter = 192;

  dbg = fdserial_open( 31, 30, 0, 115200 );
  dbg_st = (fdserial_st*)dbg->devst; // Cache a pointer to the FDSerial device structure
  dbg_txbuf = (char*)dbg_st->buffptr + FDSERIAL_BUFF_MASK+1;  

  All_LED( LED_Red & LED_Half );                         //LED red on startup

  // Do this before settings are loaded, because Sensors_Start resets the drift coefficients to defaults
  Sensors_Start( PIN_SDI, PIN_SDO, PIN_SCL, PIN_CS_AG, PIN_CS_M, PIN_CS_ALT, PIN_LED, (int)&LEDValue[0], LED_COUNT );

  F32::Start();
  QuatIMU_Start();

  InitializePrefs();

  if( Prefs.UseSBUS ) {
    SBUS::Start( PIN_RC_0 , 1024 ); // Doesn't matter - it'll be compensated for by the channel scaling/offset code
  }
  else {
    RC::Start();
  }


  // Wait 4 seconds after startup to begin checking battery voltage, rounded to an integer multiple of 16 updates
  BatteryMonitorDelay = (Const_UpdateRate * 4) & ~15;

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

  // was 30000, 15000

  RollPitch_P = 8000;           //Set here to allow an in-flight tuning baseline
  RollPitch_D = 20000 * 250;


  RollPID.Init( RollPitch_P, 0,  RollPitch_D , Const_UpdateRate );
  RollPID.SetPrecision( 12 );
  RollPID.SetMaxOutput( 3000 );
  RollPID.SetPIMax( 100 );
  RollPID.SetMaxIntegral( 1900 );
  RollPID.SetDervativeFilter( 128 );    // was 96


  PitchPID.Init( RollPitch_P, 0,  RollPitch_D , Const_UpdateRate );
  PitchPID.SetPrecision( 12 );
  PitchPID.SetMaxOutput( 3000 );
  PitchPID.SetPIMax( 100 );
  PitchPID.SetMaxIntegral( 1900 );
  PitchPID.SetDervativeFilter( 128 );


  YawPID.Init( 8000,  200 * 250,  0 , Const_UpdateRate );
  YawPID.SetPrecision( 12 );
  YawPID.SetMaxOutput( 5000 );
  YawPID.SetPIMax( 100 );
  YawPID.SetMaxIntegral( 2000 );
  YawPID.SetDervativeFilter( 192 );


  //Altitude hold PID object
  AscentPID.Init( 600, 1250 * 250, 0, Const_UpdateRate );
  AscentPID.SetPrecision( 14 );
  AscentPID.SetMaxOutput( 3000 );
  AscentPID.SetPIMax( 100 );
  AscentPID.SetMaxIntegral( 3000 );
  AscentPID.SetDervativeFilter( 128 );

  FindGyroZero();
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


void FindGyroZero(void)
{
  GyroZX = 128;
  GyroZY = 128;
  GyroZZ = 128;
  waitcnt( CNT + Const_ClockFreq/4 );
  
  for( int i=0; i<256; i++ )
  {
    GyroZX += Sensors_In(1);  // GyroX
    GyroZY += Sensors_In(2);  // GyroY
    GyroZZ += Sensors_In(3);  // GyroZ

    waitcnt( CNT + Const_ClockFreq/2000 );
  }

  GyroZX /= 256;
  GyroZY /= 256;
  GyroZZ /= 256;

  QuatIMU_SetGyroZero( GyroZX, GyroZY, GyroZZ );
}


void UpdateFlightLoop(void)
{
  int ThroOut, T1, T2, ThrustMul, AltiThrust, v, gr, gp, gy;
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
      else if( (Radio.Rudd > 750)  &&  (Radio.Aile > 750) )
      {
        CompassConfigStep++;
        FlightEnableStep = 0;

        LEDModeColor = (LED_Blue | LED_Red) & LED_Half;
                
        if( CompassConfigStep == 250 ) {   //Hold for 1 second
          StartCompassCalibrate();
        }
      }
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

    GyroRoll += ((gr - GyroRoll) * GyroRPFilter) >> 8;
    GyroPitch += ((gp - GyroPitch) * GyroRPFilter) >> 8;
    GyroYaw += ((gy - GyroYaw) * GyroYawFilter) >> 8;



    if( Radio.Thro < -800 )
    {
      // When throttle is essentially zero, disable all control authority

      if( FlightMode != FlightMode_Manual )
      {
        // Zero yaw target when throttle is off - makes for more stable liftoff
        QuatIMU_ResetDesiredYaw();
      }

      DoIntegrate = 0;          // Disable PID integral terms until throttle is applied      
    }      
    else {
      DoIntegrate = 1;
    }


    int RollOut = RollPID.Calculate_NoD2( RollDifference , GyroRoll , DoIntegrate );
    int PitchOut = PitchPID.Calculate_NoD2( PitchDifference , GyroPitch , DoIntegrate );
    int YawOut = YawPID.Calculate_NoD2( YawDifference, GyroYaw, DoIntegrate );


    int ThroMix = (Radio.Thro + 1024) >> 2;           // Approx 0 - 512
    ThroMix = clamp( ThroMix, 0, 64 );                // Above 1/8 throttle, clamp it to 64
     
    //add 12000 to all Output values to make them 'servo friendly' again   (12000 is our output center)
    ThroOut = (Radio.Thro << 2) + 12000;

    //-------------------------------------------
    if( FlightMode != FlightMode_Manual )
    {
      if( FlightMode == FlightMode_Automatic )
      {
        // Throttle has to be off zero by a bit - deadband around zero helps keep it still
        if( abs(Radio.Thro) > AltiThrottleDeadband )
        {
          // Remove the deadband area from center stick so we don't get a hiccup as you transition out of it
          int AdjustedThrottle = (Radio.Thro > 0) ? (Radio.Thro - AltiThrottleDeadband) : (Radio.Thro + AltiThrottleDeadband);

          // radio range is +/- 1024.  I'm keeping 8 bits of fractional precision, which
          // means incrementing by 1 unit per update at 250hz would result in 1 unit per
          // second of full-unit change.  Since our altitude units are millimeters, a full
          // throttle push would be 1024 units per second, or rougly 1 meter per second max rate.
          // I multiply the throttle by 4 to give a 4 m/sec ascent/descent rate

          DesiredAltitudeFractional += AdjustedThrottle << 2;
          DesiredAltitude += (DesiredAltitudeFractional >> 8);
          DesiredAltitudeFractional -= (DesiredAltitudeFractional & 0xFFFFFF00);
        }

        /*
        // in-flight PID tuning
        T1 = 2048 + (Radio.Aux2<<1);            // Left side control
        T2 = 1250 + Radio.Aux3;                 // Right side control

        AscentPID.SetDGain( T1 * 250);
        AscentPID.SetIGain( T2 * 250 );
        */

        AltiThrust = AscentPID.Calculate_NoD2( DesiredAltitude , AltiEst , DoIntegrate );
        ThroOut = Prefs.CenterThrottle + AltiThrust + (Radio.Thro << 1);
      }
      else
      {
        // Accelerometer assist    
        if( abs(Radio.Aile) < 300 && abs(Radio.Elev) < 300 && ThroMix > 32) { //Above 1/8 throttle, add a little AccelZ into the mix if the user is trying to hover
          ThroOut -= ((AccelZSmooth - Const_OneG) * (int)AccelAssistZFactor) / 64;
        }
      }

      //Tilt compensated thrust assist
      ThrustMul = 256 + ((QuatIMU_GetThrustFactor() - 256) * Prefs.ThrustCorrectionScale) / 256;
      ThrustMul = clamp( ThrustMul, 256 , 384 );    //Limit the effect of the thrust modifier
      ThroOut = Prefs.MinThrottle + (((ThroOut-Prefs.MinThrottle) * ThrustMul) >> 8);
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
        /* LED_Assisted  */    LED_Cyan,
        /* LED_Automatic */    LED_White,
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
    if( index < 3 ) {
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

static int GetDbgShort(void)
{
  short v = (fdserial_rxChar(dbg) << 8) | fdserial_rxChar(dbg);
  return v;
}  

static void TxMode( int val )
{
  Tx(0x77);
  Tx(0x77);
  Tx(val);
}

static void TxHeader( char Mode, char Len )
{
  int Header = 0x7777 | (Mode << 16) | (Len << 24);
  TxBulkUnsafe( &Header, 4 );
}  

void CheckDebugInput(void)
{
  struct FROM_HOST {
    int gsx, gsy, gsz, gox, goy, goz, aox, aoy, aoz, rollOfsSin, rollOfsCos, pitchOfsSin, pitchOfsCos;
  } host;
  int i;

  int c = fdserial_rxCheck(dbg);
  if( c < 0 ) return;

  if( c <= MODE_MotorTest ) {
    Mode = c;
    return;
  }

  if( (c & 0xF8) == 0x08 ) {  //Nudge one of the motors
      NudgeMotor = c & 7;  //Nudge the motor selected by the configuration app
      return;
  }

  if( (c & 0xF8) == 0x10 )  //Zero, Reset, or set gyro or accelerometer calibration
  {
    if( Mode == MODE_SensorTest )
    {
      if( c == 0x10 )
      {
        //Temporarily zero gyro drift settings
        Sensors_TempZeroDriftValues();
      }
      else if( c == 0x11 )
      {
        //Reset gyro drift settings
        Sensors_ResetDriftValues();
      }
      /*
      // These are now done by simply updating the prefs
      else if( c == 0x12 )
      {
        //Write new gyro drift settings - followed by 6 WORD values
        host.gsx = GetDbgShort();
        host.gsy = GetDbgShort();
        host.gsz = GetDbgShort();
        host.gox = GetDbgShort();
        host.goy = GetDbgShort();
        host.goz = GetDbgShort();

        //Copy the values into the settings object
        memcpy( &Prefs.DriftScale[0], &host.gsx, 6*sizeof(int) );

        ApplyPrefs();                                                        //Apply the settings changes
        Prefs_Save();
        loopTimer = CNT;                                                        //Reset the loop counter in case we took too long 
      }*/
      else if( c == 0x13 )
      {
        for( int i=0; i<8; i++ ) {
          Prefs.ChannelScale(i) = 1024;
          Prefs.ChannelCenter(i) = 0;
        }

        Beep2();
        loopTimer = CNT;
      }
      else if( c == 0x14 )
      {
        //Temporarily zero accel offset settings
        Sensors_TempZeroAccelOffsetValues();
      }
      else if( c == 0x15 )
      {
        //Reset accel offset settings
        Sensors_ResetAccelOffsetValues();
      }
      /*
      // These are now done by simply updating the prefs
      else if( c == 0x16 )
      {
        //Write new accelerometer offset settings - followed by 3 WORD values
        host.aox = GetDbgShort();
        host.aoy = GetDbgShort();
        host.aoz = GetDbgShort();

        //Copy the values into the settings object
        memcpy( &Prefs.AccelOffset[0], &host.aox, 3*sizeof(int) );
        
        ApplyPrefs();                                                           //Apply the settings changes
        Prefs_Save();
        loopTimer = CNT;                                                        //Reset the loop counter in case we took too long 
      }
      else if( c == 0x17 )
      {
        //Write new accelerometer rotation settings - followed by 4 FLOAT values (Sin.Cos, Sin,Cos)
        for( i=0; i<16; i++ ) {
          ((char *)&host.rollOfsSin)[i] = fdserial_rxChar(dbg);
        }

        //Copy the values into the settings object
        memcpy( &Prefs.RollCorrect[0], &host.rollOfsSin, 4*sizeof(int) );
        
        ApplyPrefs();                                                           //Apply the settings changes
        Prefs_Save();
        loopTimer = CNT;                                                        //Reset the loop counter in case we took too long 
      }
      */
    }      
  }  


  if( (c & 0xf8) == 0x18 )        //Query or modify all settings
  {
    if( c == 0x18 )               //Query current settings
    {
      TxMode( 0x18 );
      int size = sizeof(Prefs);
      Tx( size );   // Note that this will only work up to a packet size of 255 bytes - could always transmit two length bytes, perhaps?

      Prefs.Checksum = Prefs_CalculateChecksum( Prefs );
      TxBulk( &Prefs, size );

      // Wait for the data to finish sending so our unsafe Tx functions don't overwrite it
      while( fdserial_txEmpty(dbg) == 0 )
        ;

      loopTimer = CNT;                                                          //Reset the loop counter in case we took too long 
    }
    else if( c == 0x19 )          //Store new settings
    {
      PREFS TempPrefs;
      for( int i=0; i<sizeof(Prefs); i++ ) {
        ((char *)&TempPrefs)[i] = fdserial_rxTime(dbg, 50);   // wait up to 40ms per byte - Should be plenty
      }

      if( Prefs_CalculateChecksum( TempPrefs ) == TempPrefs.Checksum ) {
        memcpy( &Prefs, &TempPrefs, sizeof(Prefs) );
        Prefs_Save();

        if( Prefs_Load() ) {
          BeepOff( 'A' );   // turn off the alarm beeper if it was on
          Beep2();
          ApplyPrefs();
        }
        else {
          Beep();
        }
      }
      else {
        Beep();
      }        
      loopTimer = CNT;                                                          //Reset the loop counter in case we took too long 
    }
    else if( c == 0x1a )    // default prefs - wipe
    {
      if( fdserial_rxTime(dbg, 50) == 0x1a )
      {
        Prefs_SetDefaults();
        Prefs_Save();
        Beep3();
      }
      loopTimer = CNT;                                                          //Reset the loop counter in case we took too long 
    }
  }

  if( c == 0xff ) {
    fdserial_txChar( dbg, 0xE8);    //Simple ping-back to tell the application we have the right comm port
  }
}

void DoDebugModeOutput(void)
{
  int loop, addr, i;

  if( Mode == MODE_None ) return;
  int phase = counter & 7;    // Translates to 31.25 full updates per second, at 250hz

  switch( Mode )
  {
    case MODE_SensorTest:
    {
      switch( phase )
      {
      case 0:
        {
        TxHeader( 1, 26 );            // Radio values, 22 byte payload
        TxBulkUnsafe( &Radio , 16 );       // Radio struct is 16 bytes total
        TxBulkUnsafe( &BatteryVolts, 2 );  // Send 2 additional bytes for battery voltage
        }        
        break;

      case 1:
        TxBulkUnsafe( &LoopCycles, 4 );    // Cycles required for one complete loop  (debug data takes a LONG time)

        QuatIMU_GetDebugFloat( (float*)TxData );  // This is just a debug value, used for testing outputs with the IMU running
        TxBulkUnsafe( TxData, 4 );
        break;

      case 2:
        TxData[0] = sens.Temperature;    //Copy the values we're interested in into a WORD array, for faster transmission                        
        TxData[1] = sens.GyroX;
        TxData[2] = sens.GyroY;
        TxData[3] = sens.GyroZ;
        TxData[4] = sens.AccelX;
        TxData[5] = sens.AccelY;
        TxData[6] = sens.AccelZ;
        TxData[7] = sens.MagX;
        TxData[8] = sens.MagY;
        TxData[9] = sens.MagZ;

        TxHeader( 2, 20 );    // Sensor values, 20 byte payload
        TxBulkUnsafe( &TxData, 20 );   //Send 22 bytes of data from @TxData onward (sends 11 words worth of data)
        break;

      case 4:
        TxHeader( 3, 16 );  // Quaternion data, 16 byte payload
        TxBulkUnsafe( QuatIMU_GetQuaternion() , 16 );
        break;

      case 5: // Motor data
        TxData[0] = Motor[0];
        TxData[1] = Motor[1];
        TxData[2] = Motor[2];
        TxData[3] = Motor[3];
        TxHeader( 5, 8 );   // 8 byte payload
        TxBulkUnsafe( &TxData, 8 );
        break;


      case 6:
        TxHeader( 4, 24 );  // Computed data, 24 byte payload

        TxBulkUnsafe( &PitchDifference, 4 );
        TxBulkUnsafe( &RollDifference, 4 );
        TxBulkUnsafe( &YawDifference, 4 );

        TxBulkUnsafe( &sens.Alt, 4 );       //Send 4 bytes of data for @Alt
        TxBulkUnsafe( &sens.AltTemp, 4 );   //Send 4 bytes of data for @AltTemp
        TxBulkUnsafe( &AltiEst, 4 );        //Send 4 bytes for altitude estimate 
        break;

      case 7:
        TxHeader( 6, 16 );  // Desired Quaternion data, 16 byte payload

        // Uncomment to send the desired orientation quat instead of the measured orientation
        QuatIMU_GetDesiredQ( (float*)TxData );
        TxBulkUnsafe( TxData, 16 );
        break;
      }
    }
    break;
  }


  //Motor test code---------------------------------------
  if( NudgeMotor > -1 )
  {
    if( NudgeMotor < 4 )
    {
      Servo32_Set(MotorPin[NudgeMotor], 9500);       //Motor test - 1/8 throttle
    }
    else if( NudgeMotor == 4 )                         //Buzzer test
    {
      BeepHz(4500, 50);
      waitcnt( CNT + 5000000 );
      BeepHz(3500, 50);
    }      
    else if( NudgeMotor == 5 )                         //LED test
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
    else if( NudgeMotor == 6 )                         //ESC Throttle calibration
    {
      BeepHz(4500, 100);
      waitcnt( CNT + 5000000 );
      BeepHz(4500, 100);
      waitcnt( CNT + 5000000 );
      BeepHz(4500, 100);
      waitcnt( CNT + 5000000 );
      BeepHz(4500, 100);

      if( fdserial_rxChar(dbg) == 0xFF )  //Safety check - Allow the user to break out by sending anything else                  
      {
        for( int i=0; i<4; i++ ) {
          Servo32_Set(MotorPin[i], Prefs.MaxThrottle);
        }

        fdserial_rxChar(dbg);

        for( int i=0; i<4; i++ ) {
          Servo32_Set(MotorPin[i], Prefs.MinThrottle);  // Must add 64 to min throttle value (in this calibration code only) if using ESCs with BLHeli version 14.0 or 14.1
        }
      }
    }        
    else if( NudgeMotor == 7 )                         //Motor off (after motor test)
    {
      for( int i=0; i<4; i++ ) {
        Servo32_Set(MotorPin[i], Prefs.MinThrottle);
      }
    }        
    NudgeMotor = -1;
    loopTimer = CNT;
  }
  //End Motor test code-----------------------------------
}

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
