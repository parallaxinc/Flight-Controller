/*
  Elev8 Flight Controller
*/

#include <stdio.h>
#include <Propeller.h>
#include "RC_Receiver.h"
#include "Servo32_HighRes.h"
#include "Sensors.h"
#include "Settings.h"
#include "F32.h"
#include "IntPID.h"
#include "QuatIMU.h"
#include "fdserial.h"
#include "pins_v2.h"
#include "constants.h"
#include "beep.h"

fdserial *dbg;

void Initialize(void);
void FindGyroZero(void);
void UpdateFlightLoop(void);
void UpdateFlightLEDColor(void);
void ArmFlightMode(void);
void DisarmFlightMode(void);
void StartCompassCalibrate(void);
void DoCompassCalibrate(void);
void CheckDebugMode(void);
void DoDebugModeOutput(void);
void ApplySettings(void);

void All_LED( int Color );


#define   RC_THRO  0
#define   RC_AILE  1
#define   RC_ELEV  2
#define   RC_RUDD  3   //R/C input channel assignments (pin values are specified in the RC_Receiver object)
#define   RC_GEAR  4
#define   RC_AUX1  5
#define   RC_AUX2  6
#define   RC_AUX3  7 

 // ESC output array indices for corresponding motors
#define   OUT_FL  0
#define   OUT_FR  1
#define   OUT_BR  2
#define   OUT_BL  3

#define LED_COUNT 2

enum MODE {
  MODE_None = 0,
  MODE_RadioTest = 1,
  MODE_SensorTest = 2,
  MODE_MotorTest = 3,
  MODE_IMUTest = 4,
  MODE_IMUComp = 5,
  MODE_VibrationTest = 6,
};

enum FLIGHTMODE {
  FlightMode_Assisted = 0,
  FlightMode_Automatic = 1,
  FlightMode_Manual = 2,
  FlightMode_CalibrateCompass = 3,
};


//Working variables for the flight controller

//Receiver inputs
long  Thro, Aile, Elev, Rudd, Gear, Aux1, Aux2, Aux3;
long  iRudd;         //Integrated rudder input value

//Sensor inputs, in order of outputs from the Sensors cog, so they can be bulk copied for speed
typedef struct {
  long  Temperature, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, MagX, MagY, MagZ, Alt, AltRate, AltTemp, Pressure;
} SENS;
SENS  sens;
  
long  SensorTime;    //How long sensors took to read (debug / optimization test value)
long  GyroZX, GyroZY, GyroZZ;
long  AccelZSmooth;

//Debug output mode, working variables  
long  Mode=0, counter=0, NudgeMotor=-1;
short  TxData[10];     //Word-sized copies of Temp, Gyro, Accel, Mag, for debug transfer speed
char  Quat[16];       //Current quaternion from the IMU functions

//Current IMU values for orientation estimate
long  Pitch, Roll, Yaw, AltiEst, AscentEst;

//Working variables - used to convert receiver inputs to desired ranges for the PIDs  
long  DesiredRoll, DesiredPitch, DesiredYaw, DesiredAltitude, DesiredAltitudeFractional;

long  RollDifference, PitchDifference;                 //Delta between current measured roll/pitch and desired roll/pitch                         
long  GyroRoll, GyroPitch;                             //Raw gyro values altered by desired pitch & roll targets
long  GyroYaw;   

long  GyroRPFilter;
long  GyroYawFilter;


long  PitchOut, RollOut, YawOut;                       //Output values from the PIDs
long  ThroMix;                           

long  Motor[4];                                        //Motor output values  
long  LEDValue[LED_COUNT];                             //LED outputs (copied to the LEDs by the Sensors cog)

long loopTimer;                //Master flight loop counter - used to keep a steady update rate

long PingZero, PingHeight;

short FlightEnableStep;         //Flight arm/disarm counter
short CompassConfigStep;        //Compass configure mode counter

char FlightEnabled;            //Flight arm/disarm flag
char FlightMode;
char NewFlightMode;

char DoIntegrate;              //Integration enabled in the flight PIDs
char UsePing;              
char MotorIndex[4];            //Motor index to pin index table


long RollPitch_P, RollPitch_D;
long UseSBUS = 0, SBUSCenter;
long LEDModeColor;


char calib_startQuadrant;
char calib_Quadrants;
char calib_step;
long c_xmin, c_ymin, c_xmax, c_ymax, c_zmin, c_zmax;


INTPID  RollPID, PitchPID, YawPID, AscentPID;


//Values used to when conditioning heading and desired heading to be within 180 degrees of each other 
const int YawCircle = 0x20000;           //IMU Yaw output reading is from 0 to YawCircle-1 (change here and in IMU if desired)
const int YawMask = YawCircle - 1;
const int YawCircleHalf = YawCircle >> 1;

  //LED Color values
const int LED_Red   = 0x00FF00;
const int LED_Green = 0xFF0000;
const int LED_Blue  = 0x0000FF;
const int LED_Yellow = LED_Red | LED_Green;
const int LED_Violet = LED_Red | LED_Blue;
const int LED_Cyan =   LED_Blue | LED_Green;
const int LED_DimCyan =((LED_Blue | LED_Green) & 0xFEFEFE) >> 1;
const int LED_White   = 0xFFFFFF;

  //LED Brightness values - AND with color values to dim them
const int LED_Full    = 0xffffff;
const int LED_Half    = 0x7f7f7f;
const int LED_Quarter = 0x3f3f3f;
const int LED_Eighth  = 0x1f1f1f;
const int LED_Dim     = 0x0f0f0f;


int main()                                    // Main function
{
  Initialize(); // Set up all the objects

  //Grab the first set of sensor readings (should be ready by now)
  memcpy( &sens.Temperature, Sensors_Address(), Sensors_ParamsSize * sizeof(int) );

  //Set a reasonable starting point for the altitude computation
  QuatIMU_SetInitialAltitudeGuess( sens.Alt );

  int LoopTimer = CNT;
  counter = 0;
  int Cycles;

  while(1)
  {
    Cycles = CNT;

    //Read ALL inputs from the sensors into local memory, starting at Temperature
    memcpy( &sens.Temperature, Sensors_Address(), Sensors_ParamsSize * sizeof(int) );

    QuatIMU_Update( (int*)&sens.GyroX );        //Entire IMU takes ~92000 cycles

    AccelZSmooth += (sens.AccelZ - AccelZSmooth) / 16;

    if( UseSBUS )
    {
      //Thro =  SBUS.GetRC( RC_THRO );
      //Aile =  SBUS.GetRC( RC_AILE );
      //Elev =  SBUS.GetRC( RC_ELEV );
      //Rudd =  SBUS.GetRC( RC_RUDD );
      //Gear =  SBUS.GetRC( RC_GEAR );
      //Aux1 =  SBUS.GetRC( RC_AUX1 );
      //Aux2 =  SBUS.GetRC( RC_AUX2 );
      //Aux3 =  SBUS.GetRC( RC_AUX3 );
    }
    else
    {
      Thro =  RC_GetRC( RC_THRO );
      Aile =  RC_GetRC( RC_AILE );
      Elev =  RC_GetRC( RC_ELEV );
      Rudd =  RC_GetRC( RC_RUDD );
      Gear =  RC_GetRC( RC_GEAR );
      Aux1 =  RC_GetRC( RC_AUX1 );
      Aux2 =  RC_GetRC( RC_AUX2 );
      Aux3 =  RC_GetRC( RC_AUX3 );
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
      if( Gear < -512 )
        NewFlightMode = FlightMode_Assisted;
      else if( Gear > 512 )
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
    //waitcnt( CNT + Const_UpdateCycles );
  }    
}


void Initialize(void)
{
  RC_Start();
  F32_Start();
  dbg = fdserial_open( 31, 30, 0, 115200 );

  All_LED( LED_Red & LED_Half );                         //LED red on startup
  
  Sensors_Start( PIN_SDI, PIN_SDO, PIN_SCL, PIN_CS_AG, PIN_CS_M, PIN_CS_ALT, PIN_LED, (int)&LEDValue[0], LED_COUNT );
  QuatIMU_Start();

  FindGyroZero();

  //Initialize everything - First reset all variables to known states

  MotorIndex[0] = PIN_MOTOR_FL;
  MotorIndex[1] = PIN_MOTOR_FR;
  MotorIndex[2] = PIN_MOTOR_BR;
  MotorIndex[3] = PIN_MOTOR_BL;

  Mode = MODE_None;
  NudgeMotor = -1;                                      //No motor to nudge
  FlightEnabled = 0; 

  DesiredRoll = DesiredPitch = DesiredYaw = 0;
  FlightEnableStep = 0;                                 //Counter to know which section of enable/disable sequence we're in
  CompassConfigStep = 0;
  FlightMode = FlightMode_Assisted;
  GyroRPFilter = 192;                                   //Tunable damping filters for gyro noise, 1 (HEAVY) to 256 (NO) filtering 
  GyroYawFilter = 192;

  DIRA |= (1<<PIN_BUZZER_1);           //Enable buzzer pins    
  DIRA |= (1<<PIN_BUZZER_2);

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


  IntPID_Init( &RollPID, RollPitch_P, 0,  RollPitch_D , Const_UpdateRate );
  IntPID_SetPrecision( &RollPID, 12 );
  IntPID_SetMaxOutput( &RollPID, 3000 );
  IntPID_SetPIMax( &RollPID, 100 );
  IntPID_SetMaxIntegral( &RollPID, 1900 );
  IntPID_SetDervativeFilter( &RollPID, 96 );


  IntPID_Init( &PitchPID, RollPitch_P, 0,  RollPitch_D , Const_UpdateRate );
  IntPID_SetPrecision( &PitchPID, 12 );
  IntPID_SetMaxOutput( &PitchPID, 3000 );
  IntPID_SetPIMax( &PitchPID, 100 );
  IntPID_SetMaxIntegral( &PitchPID, 1900 );
  IntPID_SetDervativeFilter( &PitchPID, 96 );


  IntPID_Init( &YawPID, 5000,   0,  -3750000 , Const_UpdateRate );
  IntPID_SetPrecision( &YawPID, 12 );
  IntPID_SetMaxOutput( &YawPID, 5000 );
  IntPID_SetDervativeFilter( &YawPID, 96 );


  //Altitude hold PID object
  IntPID_Init( &AscentPID, 1000, 0, 250000 , Const_UpdateRate );
  IntPID_SetPrecision( &AscentPID, 12 );
  IntPID_SetMaxOutput( &AscentPID, 4000 );
  IntPID_SetPIMax( &AscentPID, 100 );
  IntPID_SetMaxIntegral( &AscentPID, 3000 );
  IntPID_SetDervativeFilter( &AscentPID, 96 );
}

static int clamp( int v, int min, int max ) {
  v = (v < min) ? min : v;
  v = (v > max) ? max : v;
  return v;
}

static int abs( int v) {
  v = (v<0) ? -v : v;
}


void FindGyroZero(void)
{
  GyroZX = 128;
  GyroZY = 128;
  GyroZZ = 128;
  waitcnt( CNT + 80000000/4 );
  
  for( int i=0; i<256; i++ )
  {
    GyroZX += Sensors_In(1);
    GyroZY += Sensors_In(2);
    GyroZZ += Sensors_In(3);

    waitcnt( CNT + 80000000/2000 );
  }

  GyroZX /= 256;
  GyroZY /= 256;
  GyroZZ /= 256;

  QuatIMU_SetGyroZero( GyroZX, GyroZY, GyroZZ );
}


void UpdateFlightLoop(void)
{
  int ThroOut, T1, T2, ThrustMul, AltiThrust, v, gr, gp, gy;

  UpdateFlightLEDColor();

  //Test for flight mode change-----------------------------------------------
  if( FlightEnabled == 0 )
  {
    //Are the sticks being pushed down and toward the center?

    if( (Thro < -750)  &&  (Elev < -750) )
    {
      if( (Rudd > 750)  &&  (Aile < -750) )
      {
        FlightEnableStep++;
        CompassConfigStep = 0;
        LEDModeColor = LED_Yellow & LED_Half;
                
        if( FlightEnableStep == 250 ) {   //Hold for 1 second
          ArmFlightMode();
        }          
      }
      else if( (Rudd > 750)  &&  (Aile > 750) )
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

    if( (Rudd < -750)  &&  (Aile > 750)  &&  (Thro < -750)  &&  (Elev < -750) )
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
    iRudd += Rudd;


    if( FlightMode == FlightMode_Manual )
    {
      RollDifference = Aile;
      PitchDifference = -Elev;
    }
    else
    {
      //Angular output from the IMU is +/- 65536 units, or 32768 = 90 degrees
      //Input range from the controls is +/- 1000 units.  Scale that up to about 22.5 degrees       

      DesiredRoll =  Aile << 3;
      DesiredPitch = -Elev << 3;

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
    DesiredYaw = iRudd >> 3;

     
    //Zero yaw target when throttle is off - makes for more stable liftoff

    if( Thro < -700 )
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
     
    RollOut = IntPID_Calculate_NoD2( &RollPID, RollDifference , GyroRoll , DoIntegrate );
    PitchOut = IntPID_Calculate_NoD2( &PitchPID, PitchDifference , GyroPitch , DoIntegrate );
    YawOut = IntPID_Calculate_ForceD_NoD2( &YawPID, DesiredYaw , Yaw , GyroYaw, DoIntegrate );


    ThroMix = (Thro + 1024) >> 2;                      // Approx 0 - 512
    ThroMix = clamp( ThroMix, 0, 64 );                // Above 1/8 throttle, clamp it to 64
     
    //add 12000 to all Output values to make them 'servo friendly' again   (12000 is our output center)
    ThroOut = (Thro << 2) + 12000;


    //-------------------------------------------
    if( FlightMode != FlightMode_Manual )
    {
      if( FlightMode == FlightMode_Automatic )
      {
        if( abs(Thro) > 100 )
        {
          DesiredAltitudeFractional += Thro >> 1;
          DesiredAltitude += (DesiredAltitudeFractional >> 8);
          DesiredAltitudeFractional -= (DesiredAltitudeFractional & 0xFFFFFF00);
        }

        T1 = (Aux2 + 1024) << 1;        //Left side control
        T2 = (Aux3 + 1024) << 8;        //Right side control 
         
        IntPID_SetPGain( &AscentPID, T1 );
        IntPID_SetDGain( &AscentPID, T2 );
      
        AltiThrust = IntPID_Calculate_NoD2( &AscentPID, DesiredAltitude , AltiEst , DoIntegrate );
        ThroOut = 12000 + Thro + AltiThrust;
      }
      else
      {
        //Accelerometer assist    
        if( abs(Aile) < 300 && abs(Elev) < 300 && ThroMix > 32) { //Above 1/8 throttle, add a little AccelZ into the mix if the user is trying to hover
          ThroOut -= (AccelZSmooth - Const_OneG) >> 1;
        }          
      }

      //Tilt compensated thrust assist      
      ThrustMul = clamp( QuatIMU_GetThrustFactor(), 256 , 384 );    //Limit the effect of the thrust modifier
      ThroOut = 8000 + (((ThroOut-8000) * ThrustMul) >> 8);
    }      
    //-------------------------------------------

     
    //X configuration
    Motor[OUT_FL] = ThroOut + ((+PitchOut + RollOut - YawOut) * ThroMix) >> 7;
    Motor[OUT_FR] = ThroOut + ((+PitchOut - RollOut + YawOut) * ThroMix) >> 7;
    Motor[OUT_BL] = ThroOut + ((-PitchOut + RollOut + YawOut) * ThroMix) >> 7;
    Motor[OUT_BR] = ThroOut + ((-PitchOut - RollOut - YawOut) * ThroMix) >> 7;


    //The low-throttle clamp prevents combined PID output from sending the ESCs below a minimum value
    //the ESCs appear to stall (go into "stop" mode) if the throttle gets too close to zero, even for a moment
     
    Motor[0] = clamp( Motor[0], 8500 , 16000);
    Motor[1] = clamp( Motor[1], 8500 , 16000);
    Motor[2] = clamp( Motor[2], 8500 , 16000);
    Motor[3] = clamp( Motor[3], 8500 , 16000);

    //Copy new Ouput array into servo values
    Servo32_Set( PIN_MOTOR_FL, Motor[0] );
    Servo32_Set( PIN_MOTOR_FR, Motor[1] );
    Servo32_Set( PIN_MOTOR_BR, Motor[2] );
    Servo32_Set( PIN_MOTOR_BL, Motor[3] );
  }
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
  int index = (counter >> 3) & 15;
  if( index == 0 ) {
    LEDModeColor = (LEDColorTable[FlightMode] & 0xFEFEFE) >> 1;
  }    
  else {
    LEDModeColor = (LEDArmDisarm[FlightEnabled & 1] & 0xFEFEFE) >> 1;
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

static void TxMode(void)
{
  Tx(0x77);
  Tx(0x77);
  Tx(Mode);
}  

void CheckDebugMode(void)
{
  int gsx, gsy, gsz, gox, goy, goz, aox, aoy, aoz, i, rollOfsSin, rollOfsCos, pitchOfsSin, pitchOfsCos;

  int c = fdserial_rxCheck(dbg);
  if( c < 0 ) return;

  if( c <= MODE_VibrationTest ) {
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
        gsx = GetDbgShort();
        gsy = GetDbgShort();
        gsz = GetDbgShort();
        gox = GetDbgShort();
        goy = GetDbgShort();
        goz = GetDbgShort();

        //Copy the values into the settings object
        memcpy( Settings_GetAddress(DriftScalePref), &gsx, 6*sizeof(int) );
        
        ApplySettings();                                                        //Apply the settings changes
        Settings_Save();
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
        aox = GetDbgShort();
        aoy = GetDbgShort();
        aoz = GetDbgShort();

        //Copy the values into the settings object
        memcpy( Settings_GetAddress(AccelOffsetPref), &aox, 3*sizeof(int) );
        
        ApplySettings();                                                           //Apply the settings changes
        Settings_Save();
        loopTimer = CNT;                                                        //Reset the loop counter in case we took too long 
      }
      else if( c == 0x17 )
      {
        //Write new accelerometer rotation settings - followed by 4 FLOAT values (Sin.Cos, Sin,Cos)
        for( i=0; i<=15; i++ ) {
          ((char *)&rollOfsSin)[i] = fdserial_rxChar(dbg);
        }

        //Copy the values into the settings object
        memcpy( Settings_GetAddress(RollCorrectPref), &rollOfsSin, 4*sizeof(int) );
        
        ApplySettings();                                                           //Apply the settings changes
        Settings_Save();
        loopTimer = CNT;                                                        //Reset the loop counter in case we took too long 
      }
    }      
  }  


  if( (c & 0xf8) == 0x18 )        //Modify a flag setting, like PWM/SBUS, Ping, etc
  {
    if( c == 0x18 )              //Receiver type (PWM / SBUS)
    {
      Settings_SetValue( UseSBUSPref , fdserial_rxChar(dbg) );
      Settings_Save();
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
  int phase = counter & 3;

  switch( Mode )
  {
  case MODE_RadioTest:
    if( phase == 0 )
    {
      TxMode();

      TxData[0] = Thro;        //Copy the values we're interested in into a WORD array, for faster transmission                        
      TxData[1] = Aile;
      TxData[2] = Elev;
      TxData[3] = Rudd;
      TxData[4] = Gear;
      TxData[5] = Aux1;
      TxData[6] = Aux2;
      TxData[7] = Aux3;

      TxBulk( &TxData, 16 );   //Send 16 bytes of data from @TxData onward (sends 8 words worth of data)                         
    }
    break;

   case MODE_SensorTest:
   
    if( phase == 0 )
    {
      TxMode();

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

      TxBulk( &TxData, 20 );   //Send 20 bytes of data from @TxData onward (sends 10 words worth of data)                         
    }          
    else if( phase == 2 )
    {
      TxBulk( &sens.Alt, 4 );       //Send 4 bytes of data for @Alt
      TxBulk( &sens.AltTemp, 4 );   //Send 4 bytes of data for @AltTemp
      TxBulk( &AltiEst, 4 );   //Send 4 bytes for altitude estimate 
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
      
  case MODE_IMUTest:

    if( phase == 0 )
    {
      //Quaternions are sensitive.  Copy it locally so we can send it in pieces without messing it up     
      memcpy( &Quat, QuatIMU_GetQuaternion(), 16);

      TxMode();
      TxBulk( &Quat , 16 );
    }
    else if( phase == 3 )
    {
      Tx( Pitch );
      Tx( Pitch >> 8 );
      Tx( Roll );
      Tx( Roll >> 8 );
      Tx( Yaw );
      Tx( Yaw >> 8 );
    }
    break;
    
  case MODE_IMUComp:

    if( (phase&1) == 0 )
    {
      TxMode();

      TxData[0] = sens.GyroX;          //Copy the values we're interested in into a WORD array, for faster transmission                        
      TxData[1] = sens.GyroY;
      TxData[2] = sens.GyroZ;
      TxData[3] = sens.AccelX;
      TxData[4] = sens.AccelY;
      TxData[5] = sens.AccelZ;

      TxBulk( &TxData, 12 );   //Send 12 bytes of data from @TxData onward (sends 6 words worth of data)                         
      TxBulk( &sens.Alt, 4 );
    }      
    break;

  case MODE_VibrationTest:
    TxMode();
     
    TxData[0] = sens.GyroX;
    TxData[1] = sens.GyroY;
    TxData[2] = sens.GyroZ;
     
    TxBulk( &TxData, 6 );   //Send 20 bytes of data from @TxData onward (sends 10 words worth of data)                         
    break;
  }    
}


void ApplySettings(void)
{
  // Placeholder
}  


void All_LED( int Color )
{
  for( int i=0; i<LED_COUNT; i++ )
    LEDValue[i] = Color;
}  
