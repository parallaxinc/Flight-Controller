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


//Working variables for the flight controller

//Receiver inputs
static RADIO Radio;
static long  iRudd;         //Integrated rudder input value

//Sensor inputs, in order of outputs from the Sensors cog, so they can be bulk copied for speed
static SENS sens;


static long  GyroZX, GyroZY, GyroZZ;
static long  AccelZSmooth;

//Debug output mode, working variables  
static long   Mode, counter, NudgeMotor;
static short  TxData[12];     //Word-sized copies of Temp, Gyro, Accel, Mag, for debug transfer speed
static char   Quat[16];       //Current quaternion from the IMU functions

static EVERYTHING_DATA Everything;   // Used to store data for the "Everything" transmit mode


//Current IMU values for orientation estimate
static long  Pitch, Roll, Yaw, AltiEst, AscentEst;

//Working variables - used to convert receiver inputs to desired ranges for the PIDs  
static long  DesiredAltitude, DesiredAltitudeFractional;

static long  RollDifference, PitchDifference;                 //Delta between current measured roll/pitch and desired roll/pitch                         
static long  GyroRoll, GyroPitch, GyroYaw;                    //Raw gyro values altered by desired pitch & roll targets

static long  GyroRPFilter, GyroYawFilter;


static long  Motor[4];                                        //Motor output values  
static long  LEDValue[LED_COUNT];                             //LED outputs (copied to the LEDs by the Sensors cog)

static long loopTimer;                //Master flight loop counter - used to keep a steady update rate

static long PingZero, PingHeight;

static short FlightEnableStep;         //Flight arm/disarm counter
static short CompassConfigStep;        //Compass configure mode counter

static char FlightEnabled;            //Flight arm/disarm flag
static char FlightMode;

static int  BatteryMonitorDelay;      //Can't start discharging the battery monitor cap until the ESCs are armed or the noise messes them up

static char MotorIndex[4];            //Motor index to pin index table


static long RollPitch_P, RollPitch_D;
static long LEDModeColor;
static short BatteryVolts = 0;

const long LowBattery = 1050;


static char calib_startQuadrant;
static char calib_Quadrants;
static char calib_step;
static long c_xmin, c_ymin, c_xmax, c_ymax, c_zmin, c_zmax;

static IntPID  RollPID, PitchPID, YawPID, AscentPID;


//Values used to when conditioning heading and desired heading to be within 180 degrees of each other 
const int YawCircle = 0x20000;           //IMU Yaw output reading is from 0 to YawCircle-1 (change here and in IMU if desired)
const int YawMask = YawCircle - 1;
const int YawCircleHalf = YawCircle >> 1;


// Used to attenuate the brightness of the LEDs, if desired.  A shift of zero is full brightness
const int LEDBrightShift = 0;
const int LEDSingleMask = 0xFF - ((1<<LEDBrightShift)-1);
const int LEDBrightMask = LEDSingleMask | (LEDSingleMask << 8) | (LEDSingleMask << 16);


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

  while(1)
  {
    int Cycles = CNT;

    //Read ALL inputs from the sensors into local memory, starting at Temperature
    memcpy( &sens, Sensors_Address(), Sensors_ParamsSize );

    QuatIMU_Update( (int*)&sens.GyroX );        //Entire IMU takes ~92000 cycles (without altimeter fusion or thrust angle compensation)

    AccelZSmooth += (sens.AccelZ - AccelZSmooth) / 16;

    if( Prefs.UseSBUS )
    {
      Radio.Thro =  SBUS::GetRC( RC_THRO );
      Radio.Aile =  SBUS::GetRC( RC_AILE );
      Radio.Elev =  SBUS::GetRC( RC_ELEV );
      Radio.Rudd =  SBUS::GetRC( RC_RUDD );
      Radio.Gear =  SBUS::GetRC( RC_GEAR );
      Radio.Aux1 =  SBUS::GetRC( RC_AUX1 );
      Radio.Aux2 =  SBUS::GetRC( RC_AUX2 );
      Radio.Aux3 =  SBUS::GetRC( RC_AUX3 );
    }
    else
    {
      Radio.Thro =  RC::GetRC( RC_THRO );
      Radio.Aile =  RC::GetRC( RC_AILE );
      Radio.Elev =  RC::GetRC( RC_ELEV );
      Radio.Rudd =  RC::GetRC( RC_RUDD );
      Radio.Gear =  RC::GetRC( RC_GEAR );
      Radio.Aux1 =  RC::GetRC( RC_AUX1 );
      Radio.Aux2 =  RC::GetRC( RC_AUX2 );
      Radio.Aux3 =  RC::GetRC( RC_AUX3 );
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

      if( Radio.Gear < -512 )
        NewFlightMode = FlightMode_Assisted;
      else if( Radio.Gear > 512 )
        NewFlightMode = FlightMode_Manual;
      else
        NewFlightMode = FlightMode_Automatic;


      if( NewFlightMode != FlightMode )
      {
        if( NewFlightMode == FlightMode_Automatic )
          DesiredAltitude = AltiEst;

        FlightMode = NewFlightMode;
      }        

      UpdateFlightLoop();            //~82000 cycles when in flight mode
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
            BatteryVolts = Battery::ComputeVoltage( Battery::ReadResult() );
            break;
        }      
      }
    }

    All_LED( LEDModeColor );

    QuatIMU_WaitForCompletion();

    Pitch = QuatIMU_GetPitch();
    Roll  = QuatIMU_GetRoll();        //Yes, this puts these 1 cycle behind, but the flight loop gets to use current gyro values
    Yaw   = QuatIMU_GetYaw();
    AltiEst = QuatIMU_GetAltitudeEstimate();
    AscentEst = QuatIMU_GetVerticalVelocityEstimate();

    CheckDebugMode();
    DoDebugModeOutput();

    Cycles = CNT - Cycles;

    ++counter;
    loopTimer += Const_UpdateCycles;
    waitcnt( loopTimer );
  }    
}


void Initialize(void)
{
  //Initialize everything - First reset all variables to known states

  MotorIndex[0] = PIN_MOTOR_FL;
  MotorIndex[1] = PIN_MOTOR_FR;
  MotorIndex[2] = PIN_MOTOR_BR;
  MotorIndex[3] = PIN_MOTOR_BL;

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

  // Do this before settings are loaded, because Sensors_Start resets the drift coefficients to defaults
  Sensors_Start( PIN_SDI, PIN_SDO, PIN_SCL, PIN_CS_AG, PIN_CS_M, PIN_CS_ALT, PIN_LED, (int)&LEDValue[0], LED_COUNT );

  InitializePrefs();

  if( Prefs.UseSBUS ) {
    SBUS::Start( PIN_RC_0 , Prefs.SBUSCenter );
  }
  else {
    RC::Start();
  }

  F32::Start();

  All_LED( LED_Red & LED_Half );                         //LED red on startup

  QuatIMU_Start();

  // Wait 4 seconds after startup to begin checking battery voltage, rounded to an integer multiple of 16 updates
  BatteryMonitorDelay = (Const_UpdateRate * 4) & ~15;

#ifdef __PINS_V3_H__
  Battery::Init( PIN_VBATT );
#endif

  DIRA |= (1<<PIN_BUZZER_1) | (1<<PIN_BUZZER_2);      //Enable buzzer pins
  OUTA &= ~((1<<PIN_BUZZER_1) | (1<<PIN_BUZZER_2));   //Set the pins low


  Servo32_Init( 400 );
  Servo32_AddFastPin( PIN_MOTOR_FL );
  Servo32_AddFastPin( PIN_MOTOR_FR );
  Servo32_AddFastPin( PIN_MOTOR_BR );
  Servo32_AddFastPin( PIN_MOTOR_BL );
  Servo32_Set( PIN_MOTOR_FL, 8000 );
  Servo32_Set( PIN_MOTOR_FR, 8000 );
  Servo32_Set( PIN_MOTOR_BR, 8000 );
  Servo32_Set( PIN_MOTOR_BL, 8000 );
  Servo32_Start();


  RollPitch_P = 10000;           //Set here to allow an in-flight tuning baseline
  RollPitch_D = 30000 * 250; 


  RollPID.Init( RollPitch_P, 0,  RollPitch_D , Const_UpdateRate );
  RollPID.SetPrecision( 12 );
  RollPID.SetMaxOutput( 3000 );
  RollPID.SetPIMax( 100 );
  RollPID.SetMaxIntegral( 1900 );
  RollPID.SetDervativeFilter( 96 );


  PitchPID.Init( RollPitch_P, 0,  RollPitch_D , Const_UpdateRate );
  PitchPID.SetPrecision( 12 );
  PitchPID.SetMaxOutput( 3000 );
  PitchPID.SetPIMax( 100 );
  PitchPID.SetMaxIntegral( 1900 );
  PitchPID.SetDervativeFilter( 96 );


  YawPID.Init( 5000,   0,  -3750000 , Const_UpdateRate );
  YawPID.SetPrecision( 12 );
  YawPID.SetMaxOutput( 5000 );
  YawPID.SetDervativeFilter( 96 );


  //Altitude hold PID object
  AscentPID.Init( 1000, 0, 250000 , Const_UpdateRate );
  AscentPID.SetPrecision( 12 );
  AscentPID.SetMaxOutput( 4000 );
  AscentPID.SetPIMax( 100 );
  AscentPID.SetMaxIntegral( 3000 );
  AscentPID.SetDervativeFilter( 96 );

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
                
        if( FlightEnableStep == 250 ) {   //Hold for 1 second
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

      if( FlightEnableStep == 250 ) {   //Hold for 1 second
        DisarmFlightMode();
        return;                  //Prevents the motor outputs from being un-zero'd
      }        
    }      
    else {
      FlightEnableStep = 0;
    }
    //------------------------------------------------------------------------

     
    //Rudder is integrated (accumulated)
    iRudd += Radio.Rudd;


    if( FlightMode == FlightMode_Manual )
    {
      RollDifference = Radio.Aile;
      PitchDifference = -Radio.Elev;
    }
    else
    {
      //Angular output from the IMU is +/- 65536 units, or 32768 = 90 degrees
      //Input range from the controls is +/- 1000 units.  Scale that up to about 22.5 degrees       

      int DesiredRoll =  Radio.Aile << 3;
      int DesiredPitch = -Radio.Elev << 3;

      RollDifference = (DesiredRoll - Roll) >> 2;
      PitchDifference = (DesiredPitch - Pitch) >> 2;
    }

    gr =  sens.GyroY - GyroZY;
    gp = -(sens.GyroX - GyroZX);
    gy = -(sens.GyroZ - GyroZZ);

    GyroRoll += ((gr - GyroRoll) * GyroRPFilter) >> 8;
    GyroPitch += ((gp - GyroPitch) * GyroRPFilter) >> 8;
    GyroYaw += ((gy - GyroYaw) * GyroYawFilter) >> 8;

    //Yaw is different because we accumulate it - It's not specified absolutely like you can
    //with pitch and roll, so scale the stick input down a bit
    int DesiredYaw = iRudd >> 3;

 
    //Zero yaw target when throttle is off - makes for more stable liftoff
    if( Radio.Thro < -700 )
    {
      DoIntegrate = 0;          //Disable PID integral term until throttle is applied      
      DesiredYaw = Yaw;         //Desired = measured when the throttle is off
      iRudd = Yaw << 3;         //Make "measured yaw" match desired yaw until throttle is applied
    }      
    else {
      DoIntegrate = 1;
    }      


    DesiredYaw &= YawMask;
     
    if( abs(DesiredYaw - Yaw) >= YawCircleHalf )
    {
      if( Yaw < YawCircleHalf )
        DesiredYaw = DesiredYaw - YawCircle;
      else
        DesiredYaw = DesiredYaw + YawCircle;
    }

    //Uncomment to allow partial PID tuning from the controller in flight
    /*
    T1 = RollPitch_P + Aux2<<1;        //Left side control
    T2 = RollPitch_D + Aux3<<2;        //Right side control 
                    
    RollPID.SetPGain( T1 ) 
    RollPID.SetDGain( T2 )                                           
    PitchPID.SetPGain( T1 )
    PitchPID.SetDGain( T2 )
    */
     
    int RollOut = RollPID.Calculate_NoD2( RollDifference , GyroRoll , DoIntegrate );
    int PitchOut = PitchPID.Calculate_NoD2( PitchDifference , GyroPitch , DoIntegrate );
    int YawOut = YawPID.Calculate_ForceD_NoD2( DesiredYaw , Yaw , GyroYaw, DoIntegrate );


    int ThroMix = (Radio.Thro + 1024) >> 2;           // Approx 0 - 512
    ThroMix = clamp( ThroMix, 0, 64 );                // Above 1/8 throttle, clamp it to 64
     
    //add 12000 to all Output values to make them 'servo friendly' again   (12000 is our output center)
    ThroOut = (Radio.Thro << 2) + 12000;

    //-------------------------------------------
    if( FlightMode != FlightMode_Manual )
    {
      if( FlightMode == FlightMode_Automatic )
      {
        if( abs(Radio.Thro) > 100 )
        {
          DesiredAltitudeFractional += Radio.Thro >> 1;
          DesiredAltitude += (DesiredAltitudeFractional >> 8);
          DesiredAltitudeFractional -= (DesiredAltitudeFractional & 0xFFFFFF00);
        }

        //T1 = (Radio.Aux2 + 1024) << 1;        //Left side control
        //T2 = (Radio.Aux3 + 1024) << 8;        //Right side control 
         
        //AscentPID.SetPGain( T1 );
        //AscentPID.SetDGain( T2 );
      
        AltiThrust = AscentPID.Calculate_NoD2( DesiredAltitude , AltiEst , DoIntegrate );
        ThroOut = 12000 + Radio.Thro + AltiThrust;
      }
      else
      {
        //Accelerometer assist    
        if( abs(Radio.Aile) < 300 && abs(Radio.Elev) < 300 && ThroMix > 32) { //Above 1/8 throttle, add a little AccelZ into the mix if the user is trying to hover
          ThroOut -= (AccelZSmooth - Const_OneG) >> 1;
        }          
      }

      //Tilt compensated thrust assist      
      ThrustMul = clamp( QuatIMU_GetThrustFactor(), 256 , 384 );    //Limit the effect of the thrust modifier
      ThroOut = 8000 + (((ThroOut-8000) * ThrustMul) >> 8);
    }      
    //-------------------------------------------

     
    //X configuration
    Motor[OUT_FL] = ThroOut + (((+PitchOut + RollOut - YawOut) * ThroMix) >> 7);
    Motor[OUT_FR] = ThroOut + (((+PitchOut - RollOut + YawOut) * ThroMix) >> 7);
    Motor[OUT_BL] = ThroOut + (((-PitchOut + RollOut + YawOut) * ThroMix) >> 7);
    Motor[OUT_BR] = ThroOut + (((-PitchOut - RollOut - YawOut) * ThroMix) >> 7);


    //The low-throttle clamp prevents combined PID output from sending the ESCs below a minimum value
    //the ESCs appear to stall (go into "stop" mode) if the throttle gets too close to zero, even for a moment
     
    Motor[0] = clamp( Motor[0], Prefs.MinThrottle , 16000);
    Motor[1] = clamp( Motor[1], Prefs.MinThrottle , 16000);
    Motor[2] = clamp( Motor[2], Prefs.MinThrottle , 16000);
    Motor[3] = clamp( Motor[3], Prefs.MinThrottle , 16000);

    //Copy new Ouput array into servo values
    Servo32_Set( PIN_MOTOR_FL, Motor[0] );
    Servo32_Set( PIN_MOTOR_FR, Motor[1] );
    Servo32_Set( PIN_MOTOR_BR, Motor[2] );
    Servo32_Set( PIN_MOTOR_BL, Motor[3] );
  }

#if 0 && defined( __PINS_V3_H__ )
    // Battery alarm at low voltage
    if( UseBattMon != 0 && (BatteryVolts < LowBattery) && ((counter & 127) < 32) )
    {
      int ctr = CNT;
      int loop = 2;
      int d = (80000000/2) / 5000;
  
      for( int i=0; i<loop; i++ )
      {
        OUTA ^= (1<<PIN_BUZZER_1);
        ctr += d;
        waitcnt( ctr );
      }
    }      
#endif
}  


static int LEDColorTable[] = {
        /* LED_Assisted  */    LED_DimCyan,
        /* LED_Automatic */    LED_Green,
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

#if 0 && defined( __PINS_V3_H__ )
  if( UseBattMon != 0 && (BatteryVolts < LowBattery) ) {
    LowBatt = 1;
  }    
#endif

  if( LowBatt ) {
    
    int index = (counter >> 3) & 15;

    if( index < 6 ) {
      LEDModeColor = (LEDColorTable[FlightMode & 3] & LEDBrightMask) >> LEDBrightShift;
    }
    else {
      LEDModeColor = (LED_Red & LEDBrightMask) >> LEDBrightShift;
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
  Servo32_Set( PIN_MOTOR_FL, 8000 );
  Servo32_Set( PIN_MOTOR_FR, 8000 );
  Servo32_Set( PIN_MOTOR_BR, 8000 );
  Servo32_Set( PIN_MOTOR_BL, 8000 );

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

static void Tx( char c ) {
  fdserial_txChar( dbg, c );
}  

static void TxBulk( void * data , int Bytes )
{
  char * buf = (char *)data;
  for( int i=0; i<Bytes; i++ ) {
    fdserial_txChar( dbg, buf[i] );
  }    
}  

static void TxMode( int val )
{
  Tx(0x77);
  Tx(0x77);
  Tx(val);
}

void CheckDebugMode(void)
{
  struct FROM_HOST {
    int gsx, gsy, gsz, gox, goy, goz, aox, aoy, aoz, rollOfsSin, rollOfsCos, pitchOfsSin, pitchOfsCos;
  } host;
  int i;

  int c = fdserial_rxCheck(dbg);
  if( c < 0 ) return;

  if( c <= MODE_Everything ) {
    Mode = c;
    return;
  }

  if( (c & 0xF8) == 0x08 )  //Nudge one of the motors
  {
    if( Mode == MODE_MotorTest )
      NudgeMotor = c & 7;  //Nudge the motor selected by the configuration app
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
    }      
  }  


  if( (c & 0xf8) == 0x18 )        //Modify a flag setting, like PWM/SBUS, Ping, etc
  {
    if( c == 0x18 )              //Receiver type (PWM / SBUS)
    {
      Prefs.UseSBUS = fdserial_rxChar(dbg);
      Prefs_Save();
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
        TxMode(1);  // Radio values
        Tx(18);     // Packet payload is 18 bytes

        TxBulk( &Radio , 16 );       // Radio struct is 16 bytes total
        TxBulk( &BatteryVolts, 2 );  // Send 2 additional bytes for battery voltage
        break;
    
      case 1:
        TxMode(2);  // Sensor values
        Tx(20);     // Payload is 20 bytes
    
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
        TxBulk( &TxData, 20 );   //Send 22 bytes of data from @TxData onward (sends 11 words worth of data)
        break;

      case 2:
        TxMode(3);  // Quaternion data
        Tx(16);     // Quaternion is 16 bytes
        TxBulk( QuatIMU_GetQuaternion() , 16 );
        break;
        
      case 3:
        TxMode(4);  // ComputedData
        Tx(24);

        TxBulk( &Pitch, 4 );
        TxBulk( &Roll, 4 );
        TxBulk( &Yaw, 4 );

        TxBulk( &sens.Alt, 4 );       //Send 4 bytes of data for @Alt
        TxBulk( &sens.AltTemp, 4 );   //Send 4 bytes of data for @AltTemp
        TxBulk( &AltiEst, 4 );        //Send 4 bytes for altitude estimate 
        break;
      }
    }
    break;


  case MODE_MotorTest:

    //Motor test code---------------------------------------
    if( NudgeMotor > -1 )
    {
      if( NudgeMotor < 4 )
      {
        Servo32_Set(MotorIndex[NudgeMotor], 9000);       //Motor test - 1/8 throttle
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
          Servo32_Set(MotorIndex[0], 16000);
          Servo32_Set(MotorIndex[1], 16000);
          Servo32_Set(MotorIndex[2], 16000);
          Servo32_Set(MotorIndex[3], 16000);
           
          fdserial_rxChar(dbg);

          Servo32_Set(MotorIndex[0], 8000);
          Servo32_Set(MotorIndex[1], 8000);
          Servo32_Set(MotorIndex[2], 8000);
          Servo32_Set(MotorIndex[3], 8000);
        }
      }        
      else if( NudgeMotor == 7 )                         //Motor off (after motor test)
      {
        Servo32_Set(MotorIndex[0], 8000);
        Servo32_Set(MotorIndex[1], 8000);
        Servo32_Set(MotorIndex[2], 8000);
        Servo32_Set(MotorIndex[3], 8000);
      }        
      NudgeMotor = -1;
      loopTimer = CNT;
    }
    break;
    //End Motor test code-----------------------------------
  }    
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
