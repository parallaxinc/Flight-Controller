//
// Settings - settings and user prefs storage for Elev8-FC
//

#include <string.h>
#include "eeprom.h"
#include "settings.h"


/*
PUB Main | testCheck
  ''This function exists only to test and validate the Load / Save / Checksum code
   
  {
  Dbg.Start( 31, 30, 0, 115200 )
  Dbg.rx

  eeprom.ToRam(@PrefStorage, @PrefStorage + constant(PrefLen*4 + 3), 32768 )    'Copy from EEPROM to DAT, address 32768

  testCheck := CalculateChecksum
  dbg.tx(0)
  dbg.hex( Checksum, 8 )
  dbg.tx(32)
  dbg.hex( testCheck, 8 )
  dbg.tx(13)


  SetDefaults   
  testCheck := CalculateChecksum
  dbg.hex( testCheck, 8 )
  dbg.tx(13)

  Save

  eeprom.ToRam(@PrefStorage, @PrefStorage + constant(PrefLen*4 + 3), 32768 )    'Copy from EEPROM to DAT, address 32768


  testCheck := CalculateChecksum
  dbg.tx(0)
  dbg.hex( Checksum, 8 )
  dbg.tx(32)
  dbg.hex( testCheck, 8 )
  dbg.tx(13)
  }
*/

static PREFS Prefs;


void Settings_Load(void)
{
  EEPROM::ToRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );    //Copy from EEPROM to DAT, address 32768

  int testChecksum = Settings_CalculateChecksum();
  if( testChecksum != Prefs.Checksum )
  {
    Settings_SetDefaults();
    Settings_Save();
  }
}

void Settings_Save(void)
{
  Prefs.Checksum = Settings_CalculateChecksum();
  EEPROM::FromRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );  //Copy from DAT to EEPROM, address 32768
}


void Settings_SetDefaults(void)
{
  memset( &Prefs, 0, sizeof(Prefs) );
  Prefs.SBUSCenter = 1000; 
  Prefs.RollCorrect[0] = 0.0f;                         //Sin of roll correction angle
  Prefs.RollCorrect[1] = 1.0f;                         //Cos of roll correction angle

  Prefs.PitchCorrect[0] = 0.0f;                        //Sin of pitch correction angle 
  Prefs.PitchCorrect[1] = 1.0f;                        //Cos of pitch correction angle 
}


long Settings_GetValue( int index )
{
  return ((int *)&Prefs)[index];
}

void Settings_SetValue( int index , int val )
{
  ((int*)&Prefs)[index] = val;
}  

void Settings_SetValue( int index , float val )
{
  ((float*)&Prefs)[index] = val;
}  

int * Settings_GetAddress( int index )
{
  return ((int*)&Prefs) + index;
}


int Settings_CalculateChecksum(void)
{
  unsigned int r = 0x55555555;            //Start with a strange, known value
  for( int i=0; i < (sizeof(Prefs)/4)-1; i++ )
  {
    r = (r << 7) | (r >> (32-7));
    r = r ^ ((unsigned int*)&Prefs)[i];     //Jumble the bits, XOR in the prefs value
  }    
  return (int)r;
}  
