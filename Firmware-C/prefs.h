//
// Preferences - User prefs storage for Elev8-FC
//

typedef struct {
  int   DriftScale[3];
  int   DriftOffset[3];
  int   AccelOffset[3];
  int   MagScaleOfs[6];

  float RollCorrect[2];
  float PitchCorrect[2];

  char  UseSBUS;
  char  UsePing;
  char  UseBattMon;
  char  unused;

  char  LowVoltageBuzzer;
  char  LowVoltageAscentLimit;
  char  unused2;
  char  unused3;

  short  ThrustCorrectionScale;  // 0 to 256  =  0 to 1
  short  AccelCorrectionFilter;  // 0 to 256  =  0 to 1

  short MaxRollPitch;
  short RollPitchSpeed;
  short YawSpeed;

  short VoltageOffset;    // Used to correct the difference between measured and actual voltage
  short LowVoltageAlarm;  // default is 1050 (10.50v)

  short ArmDelay;
  short DisarmDelay;

  short ThrottleTest;     // Typically the same as MinThrottleArmed, unless MinThrottleArmed is too low for movement

  short MinThrottle;      // Minimum motor output value
  short MaxThrottle;      // Maximum motor output value
  short CenterThrottle;   // Mid-point motor output value
  short MinThrottleArmed; // Minimum throttle output value when armed - MUST be equal or greater than MinThrottle

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
