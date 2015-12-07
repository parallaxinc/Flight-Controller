#ifndef __ELEV8_MAIN_H__
#define __ELEV8_MAIN_H__

void Initialize(void);
void InitReceiver(void);
void InitSerial(void);
void FindGyroZero(void);
void UpdateFlightLoop(void);
void UpdateFlightLEDColor(void);
void ArmFlightMode(void);
void DisarmFlightMode(void);
void StartCompassCalibrate(void);
void DoCompassCalibrate(void);
void CheckDebugInput(void);
void DoDebugModeOutput(void);
void InitializePrefs(void);
void ApplyPrefs(void);
void All_LED( int Color );


 // ESC output array indices for corresponding motors
#define   OUT_FL  0
#define   OUT_FR  1
#define   OUT_BR  2
#define   OUT_BL  3

#define LED_COUNT 2


enum MODE {
  MODE_None = 0,
  MODE_SensorTest = 2,
  MODE_MotorTest = 3,
};

enum FLIGHTMODE {
  FlightMode_Assisted = 0,
  FlightMode_Automatic = 1,
  FlightMode_Manual = 2,
  FlightMode_CalibrateCompass = 3,
};

// Structure to hold radio values to make sure they stay in order
struct RADIO {
  short Thro, Aile, Elev, Rudd, Gear, Aux1, Aux2, Aux3, Aux4;   // Aux4 is an additional raw channel for SBUS users only

  short & Channel(int i) { return (&Thro)[i]; }
};


//LED Brightness values - AND with color values to dim them
const int LED_Full    = 0xffffff;
const int LED_Half    = 0x7f7f7f;
const int LED_Quarter = 0x3f3f3f;
const int LED_Eighth  = 0x1f1f1f;
const int LED_Dim     = 0x0f0f0f;


//LED Color values
const int LED_Red   = 0x00FF00;
const int LED_Green = 0xFF0000;
const int LED_Blue  = 0x0000FF;
const int LED_White = 0xFFFFFF;
const int LED_Yellow = LED_Red | LED_Green;
const int LED_Violet = LED_Red | LED_Blue;
const int LED_Cyan =   LED_Blue | LED_Green;

const int LED_DimCyan = (LED_Blue | LED_Green) & LED_Half;
const int LED_DimWhite = LED_White & LED_Half;

#define COMMAND(a,b,c,d) (int)((a<<24) | (b<<16) | (c<<8) | (d<<0) )

#define Comm_Elv8       COMMAND('E','l','v','8')
#define Comm_Beat       COMMAND('B','E','A','T')
#define Comm_QueryPrefs COMMAND('Q','P','R','F')
#define Comm_SetPrefs   COMMAND('U','P','r','f')
#define Comm_Wipe       COMMAND('W','I','P','E')

#define Comm_ZeroGyro   COMMAND('Z','r','G','r')
#define Comm_ZeroAccel  COMMAND('Z','e','A','c')
#define Comm_ResetGyro  COMMAND('R','G','y','r')
#define Comm_ResetAccel COMMAND('R','A','c','l')
#define Comm_ResetRadio COMMAND('R','r','a','d')

#define Comm_Motor1     COMMAND('M','1','t','1')    // Motor test values are duplicated - very unlikely to get set this way by noise
#define Comm_Motor2     COMMAND('M','2','t','2')
#define Comm_Motor3     COMMAND('M','3','t','3')
#define Comm_Motor4     COMMAND('M','4','t','4')
#define Comm_Motor5     COMMAND('M','5','t','5')
#define Comm_Motor6     COMMAND('M','6','t','6')

#endif
