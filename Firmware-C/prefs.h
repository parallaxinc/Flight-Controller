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

  int   UseSBUS;
  int   SBUSCenter;

  char  UsePing;
  char  UseBattMon;

  short MinThrottle;

  char  ThroChannel;
  char  AileChannel;
  char  ElevChannel;
  char  RuddChannel;
  char  GearChannel;
  char  Aux1Channel;
  char  Aux2Channel;
  char  Aux3Channel;

  int   Checksum;
} PREFS;


extern PREFS Prefs;


void Prefs_Load(void);
void Prefs_Save(void);
void Prefs_SetDefaults(void);

void Prefs_Test(void);

int Prefs_CalculateChecksum(void);
