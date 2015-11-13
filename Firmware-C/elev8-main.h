#ifndef __ELEV8_MAIN_H__
#define __ELEV8_MAIN_H__

void Initialize(void);
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
  MODE_RadioTest = 1,
  MODE_SensorTest = 2,
  MODE_MotorTest = 3,
  MODE_IMUTest = 4,
  MODE_IMUComp = 5,
  MODE_VibrationTest = 6,
  MODE_Everything = 7,
};

enum FLIGHTMODE {
  FlightMode_Assisted = 0,
  FlightMode_Automatic = 1,
  FlightMode_Manual = 2,
  FlightMode_CalibrateCompass = 3,
};

// Structure to hold radio values to make sure they stay in order
struct RADIO {
  short Thro, Aile, Elev, Rudd, Gear, Aux1, Aux2, Aux3;

  short & Channel(int i) { return (&Thro)[i]; }
};


struct EVERYTHING_DATA
{
  short Thro, Aile, Elev, Rudd, Gear, Aux1, Aux2, Aux3;         // Radio values = 16 bytes
  short BatteryVolts;                                           // Battery Monitor = 2 bytes
  short padding;                                                // curently unused

  short Temp, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ, MagX, MagY, MagZ;  // IMU sensors = 20 bytes

  long  Alt, AltRate, Pressure;                                 // Altimeter = 12 bytes
  long  Pitch, Roll, Yaw;                                       // IMU = 12 bytes

  char Quaternion[16];                                          // Quaternion = 16 bytes
};                                                              // 80 bytes total


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



#endif
