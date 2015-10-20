//
// Settings - settings and user prefs storage for Elev8-FC
//

typedef struct {
  long  DriftScale[3];
  long  DriftOffset[3];
  long  AccelOffset[3];
  long  MagScaleOfs[6];
  long  UseSBUS;
  long  SBUSCenter;
  long  UsePing;
  float RollCorrect[2];
  float PitchCorrect[2];
  long  Checksum;
} PREFS;
  

#define DriftScalePref     0           //3 longs
#define DriftOffsetPref    3          //3 longs 

#define AccelOffsetPref    6          //3 longs
#define MagScaleOfsPref    9          //6 longs

#define UseSBUSPref       15          //1 long                       
#define SBUSCenterPref    16          //1 long
#define UsePingPref       17          //1 long 

#define RollCorrectPref   18          //2 longs (Sin,Cos)
#define PitchCorrectPref  20          //2 longs (Sin.Cos)

#define PrefLen           22



void Settings_Load(void);
void Settings_Save(void);
void Settings_SetDefaults(void);

long Settings_GetValue( int index );
void Settings_SetValue( int index , long val );
long * Settings_GetAddress( int index );

long Settings_CalculateChecksum(void);
