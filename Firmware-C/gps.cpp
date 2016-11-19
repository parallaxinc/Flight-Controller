/*

  gps.cpp - GPS support for the Elev8-V3

*/

#include <propeller.h>
#include "gps.h"
#ifdef GPS

#include "serial_4x.h"


char SatCount = 0;
int Latitude = 0;
int Longitude = 0;
int Dilution = 0;
int GpsAltitude = 0;

void GPSThread(void * par);


static u8 header[] =  { 0xB5, 0x62, 0x06 };
static u8 SetBaud[] = { 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08, 0x00, 0x00, 0x00, 0xe1,
                        0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd2, 0xe1 };

void SendGPSConfig( u8 * data, u8 count )
{
  S4_Put_Bytes(2, header, 3);
  S4_Put_Bytes(2, data, count);
}


void SetGPSBaud(void)
{
  SendGPSConfig( SetBaud, sizeof(SetBaud) );
}

#define GPS_STACK_SIZE (32 + 40)          // stack needs to accomodate thread control structure (40) plus room for functions (16)
int gps_stack[GPS_STACK_SIZE];


void StartGPSThread(void)
{
  cogstart( &GPSThread , 0, gps_stack, sizeof(gps_stack) );
}  


static u8 Checksums[6][2] = {
  0x00, 0x28, // for EnableGGA
  0x00, 0x2a,
  0x01, 0x31,
  0x02, 0x38,
  0x03, 0x3f,
  0x04, 0x46,
};

void SetGPSOutputConfig(void)
{
  // Enable  GGA: 0x01, 0x08, 0x00, 0xf0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,
  // Disable GLL: 0x01, 0x08, 0x00, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a,
  // Disable GSA: 0x01, 0x08, 0x00, 0xf0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31,
  // Disable GSV: 0x01, 0x08, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38,
  // Disable RMC: 0x01, 0x08, 0x00, 0xf0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3f,
  // Disable VTG: 0x01, 0x08, 0x00, 0xf0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46,

  // Set to 5Hz : 0x08, 0x06, 0x00, 0xc8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xde, 0x6a,
  // Set to 10Hz: 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7a, 0x12,

  static u8 command[] = { 0x01, 0x08, 0x00, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  waitcnt( CNT + 80000 * 200 );

  command[6] = 1; // Enable GGA, then disable everything else
  for( int i=0; i<=5; i++ )
  {
    S4_Put_Bytes(2, header, 3);
    command[4] = i;
    S4_Put_Bytes(2, command, sizeof(command) );
    S4_Put_Bytes(2, Checksums[i], 2);
    command[6] = 0;
    waitcnt( CNT + 80000 * 100 );
  }

  // 5Hz output rate
  static u8 rateCommand[] = { 0x08, 0x06, 0x00, 0xc8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xde, 0x6a };

  S4_Put_Bytes(2, header, 3);
  S4_Put_Bytes(2, rateCommand, sizeof(rateCommand) );
}

static char HasFix = 0;
static char Line[128];
static int Len, Pos;


inline char Get(void)
{
  //S4_Put(0, Line[Pos] );
  return Line[Pos++];
}

inline int GetI(void) {
  return Get() - '0';
}  

void SkipField(void) {
  while( Get() != ',' ) {
    ;
  }    
}

int ParseLatitude(void) // DDMM.mmmmm
{
  // peek to see if the next char is a comma - if so we haven't got sat-lock
  if( Line[Pos] == ',' ) {
    Pos+=2; // skip lat and N/S
    return 0;
  }

  int degs = (GetI() * 10) + GetI();
  int mins = (GetI() * 10) + GetI();

  degs *= 10000000;

  if( Get() != '.' ) {
    HasFix = false;
    return Latitude;
  }
  int i=5;
  char ch;
  while( i > 0 )
  {
    ch = Get();
    if( ch == ',' ) break;
    mins *= 10;
    mins += ch - '0';
    i--;
  }
  if( ch != ',' ) {
    Get(); // get the comma
  }

  // convert arc-minutes (60ths) to fractions of degrees (100ths)
  degs += mins * 10 / 6;

  ch = Get();
  if( ch == 'S' ) {
    degs = -degs;
  }

  SkipField();  // skip to the next field

  return degs;
}


int ParseLongitude(void) // DDDMM.mmmmm
{
  // peek to see if the next char is a comma - if so we haven't got sat-lock
  if( Line[Pos] == ',' ) {
    Pos+=2; // skip long and E/W
    return 0;
  }

  int degs = ((GetI() * 10) + GetI())*10 + GetI();
  int mins = (GetI() * 10) + GetI();

  degs *= 10000000;

  if( Get() != '.' ) {
    HasFix = false;
    return Longitude;
  }
  int i=5;
  char ch;
  while( i > 0 )
  {
    ch = Get();
    if( ch == ',' ) break;
    mins *= 10;
    mins += ch - '0';
    i--;
  }
  if( ch != ',' ) {
    Get(); // get the comma
  }

  // convert arc-minutes (60ths) to fractions of degrees (100ths)
  degs += mins * 10 / 6;

  ch = Get();
  if( ch == 'W' ) {
    degs = -degs;
  }

  SkipField();  // skip to the next field

  return degs;
}


int ParseFloat( void )  // used for Horizontal dilution and altitude
{
  int v = 0;    // Integer
  int lowDigits = 0;
  char ch;

  while(1) {
    ch = Get();
    if( ch < '0' || ch > '9' ) break;
    v *= 10;
    v += ch - '0';
  }

  if( ch == '.' ) { // Found the decimal, so parse the fractional part
    while(1) {
      ch = Get();
      if( ch < '0' || ch > '9' ) break;
      if( lowDigits < 5 )
      {
        v *= 10;
        v += ch - '0';
        lowDigits++;
      }        
    }

    while( lowDigits < 1 ) {
      v *= 10;
      lowDigits++;
    }
  }
  else {
    v *= 10;  // scale it up to the precision expected
  }    

  while( Get() != ',' )  // Skip the terminating ,
    ;

  return v;
}


// http://aprs.gids.nl/nmea/#rmc
void ParseGGA(void) // Global Positioning System Fix Data
{
  // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh

  Get();                     // Grab the initial comma

  //TimeStamp = ParseTime();  // Get the time value
  SkipField();                // Skip the time value

  int tempLat = ParseLatitude();     // Parse latitude
  int tempLong = ParseLongitude();   // Parse longitude

  char ch = Get();
  HasFix = (ch != '0' && ch != ',');
  if( ch != ',' ) {
    Get(); // Skip the comma
  }    

  if( HasFix ) {
    Latitude = tempLat;     // try to make sure they update at the same time
    Longitude = tempLong;
  }

  char temp = 0;
  while(1) {
    char ch = Get();
    if( ch < '0' || ch > '9' ) break;
    temp *= 10;
    temp += ch - '0';
  }
  if( temp != 0 ) {
    SatCount = temp;
  }
  else {
    SatCount = (char)-1;
  }

  temp = ParseFloat();
  if( temp != 0 ) {
    Dilution = temp;
  }

  temp = ParseFloat();
  if( temp != 0 ) {
    GpsAltitude = temp;
  }
}

void GPSThread(void *)
{
  SetGPSOutputConfig();

  while( 1 )
  {
    if( S4_Get(2) != '$' ) continue;
    Len = Pos = 0;
    char ch;
    do {
      ch = S4_Get(2);
      if( ch != 13 ) {
        Line[Len] = ch;
        if( Len < 128 ) Len++;
      }
    } while( ch != 13 );


    // Get the packet type (5 chars, but 1st is always 'G')
    ch = Get();
    if( ch != 'G' ) continue;

    int type = 0;
    for( int i=0; i<4; i++ ) {
      type <<= 8;
      type |= Get();
    }

    if( type == 'PGGA' ) {
      ParseGGA();
    }
  }
}  


#endif  // GPS
