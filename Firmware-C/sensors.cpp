/*
  Elev8 Flight Controller - V1.0

  This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-sa/4.0/

  Written by Jason Dorie
*/

#include <propeller.h>
#include "constants.h"
#include "sensors.h"

/*
  HUNDRED_nS  = _clkfreq / 10_000_000  'Number of clock cycles per 100 nanoseconds (8 @ 80MHz)                        
  ONE_uS      = HUNDRED_nS * 10 'Number of clock cycles per 1 microsecond (1000 nanoseconds)

'WS2812B Timings
  LED_0_HI    = (ONE_uS * 35)/100       
  LED_0_LO    = (ONE_uS * 90)/100       
  LED_1_HI    = (ONE_uS * 90)/100       
  LED_1_LO    = (ONE_uS * 35)/100       


'WS2812 Timings
'  LED_0_HI    = (ONE_uS * 35)/100       
'  LED_0_LO    = (ONE_uS * 80)/100       
'  LED_1_HI    = (ONE_uS * 70)/100       
'  LED_1_LO    = (ONE_uS * 60)/100       
*/


static struct DATA {
  int  ins[Sensors_ParamsCount];  //Temp, GX, GY, GZ, AX, AY, AZ, MX, MY, MZ, Alt, AltRate, AltTemp, Pressure, Timer
  int  DriftScale[3];
  int  DriftOffset[3];            //These values will be altered in the EEPROM by the Config Tool and Propeller Eeprom code                       
  int  AccelOffset[3];
  int  MagOffsetX, MagScaleX, MagOffsetY, MagScaleY, MagOffsetZ, MagScaleZ;
} data;

static int DriftBackup[6];
static int AccelBackup[3];
static int MagBackup[6];

static int cog;
extern long AltTable_000[];

void Sensors_Start( int ipin, int opin, int cpin, int sgpin, int smpin, int apin, int _LEDPin, int _LEDAddr, int _LEDCount )
{
// Start driver - starts a cog
// returns false if no cog available
// may be called again to change settings
//
//   ipin    = pin connected to DIN
//   opin    = pin connected to DOUT
//   cpin    = pin connected to CLK
//   sgpin   = pin connected to CS_AG
//   smpin   = pin connected to CS_M
//   apin    = pin connected to CS on altimeter
//   LEDPin  = pin connected to WS2812B LED array
//   LEDAddr = HUB address of RGB values for LED array (updated constantly)
//   LEDCount= Number of LED values to update  

	Sensors_Stop();

	data.ins[0] = ipin;
	data.ins[1] = opin;
	data.ins[2] = cpin;
	data.ins[3] = sgpin;
	data.ins[4] = smpin;
	data.ins[5] = apin;
	data.ins[6] = _LEDPin;
	data.ins[7] = _LEDAddr;
	data.ins[8] = _LEDCount;
	data.ins[9] = (long)&AltTable_000[0];       //Append the HUB address of the pressure to altitude table 


	data.DriftScale[0] = data.DriftScale[1] = data.DriftScale[2] = 0;
	data.DriftOffset[0] = data.DriftOffset[1] = data.DriftOffset[2] = 0;
	data.AccelOffset[0] = data.AccelOffset[1] = data.AccelOffset[2] = 0;
	data.MagOffsetX = data.MagOffsetY = data.MagOffsetZ = 0;
	data.MagScaleX = data.MagScaleY = data.MagScaleZ = 1024;

	// cog = cognew(@entry, @ins) + 1;
  use_cog_driver(sensors_driver);
  cog = load_cog_driver(sensors_driver, &data.ins[0]) + 1;
}


void Sensors_Stop(void)
{
// Stop driver - frees a cog
	if( cog ) {
		cogstop(cog - 1);
		cog = 0;
	}
}

int Sensors_In( int channel )
{
// Read the current value from a channel (0..ParamsSize-1)
  return data.ins[channel];
}


int * Sensors_Address(void)
{
  // Get the address of the sensor readings
  return &data.ins[0];
}

void Sensors_TempZeroDriftValues(void)
{
  //Temporarily back up the values so we can restore them with "ResetDriftValues"
  memcpy( &DriftBackup, &data.DriftScale[0], 6*sizeof(int) );
  memset( &data.DriftScale[0], 0, 6*sizeof(int));
}

void Sensors_ResetDriftValues(void)
{
  //Restore the values from our backup
  memcpy( &data.DriftScale[0], &DriftBackup, 6*sizeof(int) );
}


void Sensors_TempZeroAccelOffsetValues(void)
{
  //Temporarily back up the values so we can restore them with "ResetAccelOffsetValues"
  memcpy( &AccelBackup, &data.AccelOffset[0], 3*sizeof(int) );
  memset( &data.AccelOffset[0], 0, 3*sizeof(int) );
}

void Sensors_ResetAccelOffsetValues(void)
{
  memcpy( &data.AccelOffset[0], &AccelBackup, 3*sizeof(int) );
}


void Sensors_SetDriftValues( int * ScaleAndOffsetsAddr )
{
  memcpy( &data.DriftScale[0], ScaleAndOffsetsAddr, 6*sizeof(int) );
  memcpy( &DriftBackup, ScaleAndOffsetsAddr, 6*sizeof(int) );
}


void Sensors_SetAccelOffsetValues( int * OffsetsAddr )
{
  memcpy( &data.AccelOffset[0], OffsetsAddr, 3*sizeof(int) );
  memcpy( &AccelBackup, OffsetsAddr, 3*sizeof(int) );
}

void Sensors_ZeroMagnetometerScaleOffsets(void)
{
  memset( &data.MagOffsetX, 0, 6*sizeof(int) );
}

void Sensors_SetMagnetometerScaleOffsets( int * MagOffsetsAndScalesAddr )
{
  memcpy( &data.MagOffsetX, MagOffsetsAndScalesAddr, 6*sizeof(int) );
}


        //Table used to convert pressure to altitude.  The Pressure to Altitude conversion is complex,
        //and requires Log and Pow functions, which take a considerable length of CPU time.  A table lookup
        //is a suitable alternative.  I use linear interpolation, but the table has enough points to give an
        //absolute accuracy of +/- 6 inches, with an average accuracy of +/- 1 inch.

        //Table entries are altitude in mm, table index is (hPa - 260)/4, table range is 260 to 1260 hPa   

long AltTable_000[] = {
    10108515, 
    10008960,
    9910619,
    9813460,
    9717451,
    9622562,
    9528765,
    9436031,
    9344335,
    9253650,
    9163951,
    9075217,
    8987422,
    8900546,
    8814567,
    8729465,
    8645220,
    8561814,
    8479226,
    8397441,
    8316440,
    8236207,
    8156726,
    8077982,
    7999958,
    7922642,
    7846018,
    7770072,
    7694793,
    7620166,
    7546179,
    7472819,
    7400077,
    7327938,
    7256394,
    7185432,
    7115043,
    7045215,
    6975940,
    6907207,
    6839008,
    6771332,
    6704171,
    6637517,
    6571360,
    6505694,
    6440508,
    6375797,
    6311552,
    6247765,
    6184430,
    6121540,
    6059086,
    5997064,
    5935466,
    5874285,
    5813516,
    5753153,
    5693188,
    5633617,
    5574434,
    5515633,
    5457209,
    5399155,
    5341469,
    5284143,
    5227173,
    5170554,
    5114281,
    5058350,
    5002756,
    4947494,
    4892560,
    4837950,
    4783660,
    4729685,
    4676021,
    4622665,
    4569611,
    4516858,
    4464400,
    4412235,
    4360358,
    4308766,
    4257455,
    4206423,
    4155665,
    4105179,
    4054961,
    4005008,
    3955317,
    3905884,
    3856708,
    3807785,
    3759112,
    3710686,
    3662505,
    3614565,
    3566865,
    3519400,
    3472170,
    3425171,
    3378400,
    3331856,
    3285535,
    3239436,
    3193556,
    3147893,
    3102444,
    3057207,
    3012181,
    2967362,
    2922749,
    2878339,
    2834132,
    2790123,
    2746313,
    2702697,
    2659276,
    2616046,
    2573006,
    2530154,
    2487488,
    2445006,
    2402707,
    2360589,
    2318650,
    2276889,
    2235303,
    2193891,
    2152652,
    2111584,
    2070685,
    2029953,
    1989388,
    1948988,
    1908751,
    1868676,
    1828761,
    1789004,
    1749406,
    1709963,
    1670676,
    1631541,
    1592559,
    1553727,
    1515045,
    1476511,
    1438124,
    1399883,
    1361786,
    1323832,
    1286020,
    1248349,
    1210818,
    1173425,
    1136170,
    1099051,
    1062067,
    1025217,
    988500,
    951915,
    915461,
    879136,
    842941,
    806873,
    770932,
    735117,
    699426,
    663859,
    628415,
    593093,
    557893,
    522812,
    487850,
    453006,
    418280,
    383671,
    349177,
    314797,
    280532,
    246380,
    212339,
    178411,
    144593,
    110884,
    77285,
    43794,
    10410,
    -22865,
    -56037,
    -89102,
    -122064,
    -154922,
    -187676,
    -220329,
    -252880,
    -285330,
    -317680,
    -349931,
    -382083,
    -414136,
    -446093,
    -477952,
    -509716,
    -541383,
    -572957,
    -604435,
    -635821,
    -667113,
    -698313,
    -729422,
    -760439,
    -791365,
    -822202,
    -852949,
    -883608,
    -914178,
    -944661,
    -975057,
    -1005366,
    -1035589,
    -1065726,
    -1095779,
    -1125747,
    -1155632,
    -1185433,
    -1215151,
    -1244787,
    -1274341,
    -1303814,
    -1333206,
    -1362518,
    -1391750,
    -1420903,
    -1449977,
    -1478973,
    -1507890,
    -1536730,
    -1565494,
    -1594180,
    -1622791,
    -1651326,
    -1679786,
    -1708171,
    -1736482,
    -1764719,
    -1792882,
    -1820973,
    -1848991,
    -1876937,
    -1876937 };       //Last entry is duplicated so we don't have to check / clamp in the code 
