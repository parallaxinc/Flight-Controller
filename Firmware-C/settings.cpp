//
// Settings - settings and user prefs storage for Elev8-FC
//

#include <string.h>   // for memset()
#include <fdserial.h>

#include "eeprom.h"
#include "settings.h"


PREFS Prefs;


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
  Prefs.UseBattMon = 1;

  Prefs.RollCorrect[0] = 0.0f;                         //Sin of roll correction angle
  Prefs.RollCorrect[1] = 1.0f;                         //Cos of roll correction angle

  Prefs.PitchCorrect[0] = 0.0f;                        //Sin of pitch correction angle 
  Prefs.PitchCorrect[1] = 1.0f;                        //Cos of pitch correction angle 

  // MagOffsetX=0, MagScaleX=1, MagOffsetY=2, MagScaleY=3, MagOffsetZ=4, MagScaleZ=5;
  Prefs.MagScaleOfs[1] = 1024;
  Prefs.MagScaleOfs[3] = 1024;
  Prefs.MagScaleOfs[5] = 1024;
}

/*
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
*/


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


extern fdserial * dbg;

static int tGetC( void ) {
  return fdserial_rxChar( dbg );
}

static void tPutC( char c ) {
  fdserial_txChar( dbg, c );
}

static void tPutHexNibble( int x ) {
  if( x <= 9 ) {
    tPutC( x + '0' );
  }
  else {
    tPutC( x - 10 + 'a' );
  }        
}  

static void tPutHex( int x, int len ) {
  for( int i=len-1; i>=0; i-- ) {
    tPutHexNibble( (x>>(4*i)) & 15 );
  }    
}


void Settings_Test( void )
{
  // This function exists only to test and validate the Load / Save / Checksum code
   
  tGetC();

  EEPROM::ToRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );    //Copy from EEPROM to DAT, address 32768

  int testCheck = Settings_CalculateChecksum();
  tPutC(0);
  tPutHex( Prefs.Checksum, 8 );
  tPutC(32);
  tPutHex( testCheck, 8 );
  tPutC(13);

  //EEPROM::FromRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );    //Copy from EEPROM to DAT, address 32768

  //tPutHex( Prefs.Checksum, 8 );
  //tPutC(32);
  //tPutHex( testCheck, 8 );
  //tPutC(13);
  //tPutC(13);


  Settings_SetDefaults();
  testCheck = Settings_CalculateChecksum();
  tPutHex( testCheck, 8 );
  tPutC(13);

  Settings_Save();

  EEPROM::ToRam( &Prefs, (char *)&Prefs + sizeof(Prefs)-1, 32768 );    //Copy from EEPROM to DAT, address 32768

  testCheck = Settings_CalculateChecksum();

  tPutHex( Prefs.Checksum, 8 );
  tPutC(32);
  tPutHex( testCheck, 8 );
  tPutC(13);

  tGetC();
}
