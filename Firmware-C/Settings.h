//
// Settings - settings and user prefs storage for Elev8-FC
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
  int   UsePing;
  int   UseBattMon;
  int   Checksum;
} PREFS;


extern PREFS Prefs;

/*
#define DriftScalePref     0          //3 longs
#define DriftOffsetPref    3          //3 longs 

#define AccelOffsetPref    6          //3 longs
#define MagScaleOfsPref    9          //6 longs

#define RollCorrectPref   19          //2 longs (Sin,Cos)
#define PitchCorrectPref  21          //2 longs (Sin.Cos)

#define UseSBUSPref       15          //1 long
#define SBUSCenterPref    16          //1 long
#define UsePingPref       17          //1 long
#define UseBattMonPref    18          //1 long
*/


void Settings_Load(void);
void Settings_Save(void);
void Settings_SetDefaults(void);

void Settings_Test(void);

//long Settings_GetValue( int index );
//void Settings_SetValue( int index , int val );
//void Settings_SetValue( int index , float val );
//int * Settings_GetAddress( int index );

int Settings_CalculateChecksum(void);
