
#include "intpid.h"


static int clamp( int v, int min, int max ) {
  v = (v < min) ? min : v;
  v = (v > max) ? max : v;
  return v;
}

static int abs(int v) {
  v = (v<0) ? -v : v;
  return v;
}


void IntPID::Init( int PGain, int IGain, int DGain, short _SampleRate )
{
  SampleRate = _SampleRate;
  Kp = PGain;
  Ki = IGain / SampleRate;
  Kd = DGain / SampleRate;
  PMax = 0;
  PIMax = 0;
  DerivFilter = 0;

  DError = 0;
  LastPError = 0;
  IError = 0;
  MaxIntegral = 0x010000;
  MaxOutput = 1000;
  Precision = 16;
  RoundOffset = 1 << (Precision-1);
}

int IntPID::Calculate( int SetPoint , int Measured , char DoIntegrate )
{
  // Proportional error is Desired - Measured
  int PError = SetPoint - Measured;
  
  // Derivative error is the delta PError divided by time
  // If loop timing is const, you can skip the divide and just make the factor smaller
  if( DerivFilter == 0 ) {
    DError = PError - LastPError;
  }
  else {
    int RawDeriv = PError - LastPError;
    DError += ((RawDeriv - DError) * DerivFilter) >> 8;  //Filter the derivative error term so it's not completely nuts
  }

  LastPError = PError;

  int PClamped = PError;
  if( PMax > 0 ) {
    PClamped = clamp( PClamped, -PMax, PMax );
  }

  Output = (Kp * PClamped);
  if( Kd ) {
    Output += (Kd * DError);
  }

  if( Ki ) {
    Output += (Ki * IError);
  }

  Output = (Output + RoundOffset) >> Precision;
  
  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 

  if( abs(Output) > MaxOutput ) {
    //if( DoIntegrate && Ki ) {
    //  int Over = ((abs(Output) - MaxOutput) << Precision) / Ki;
    //  if( Output > 0 )
    //    IError -= Over;
    //  else
    //    IError += Over;
    //}

    Output = clamp( Output, -MaxOutput, MaxOutput );
  }    

  if( DoIntegrate && Ki != 0 )
  {
    PClamped = PError;
    if( PIMax > 0 ) {
      PClamped = clamp( PClamped, -PIMax, PIMax );
    }

    IError += PClamped;
    IError = clamp( IError, -MaxIntegral, MaxIntegral );
  }
     
  return Output;
}


/*
int IntPID::Calculate_ForceD( int SetPoint , int Measured , int Deriv , int DoIntegrate )
{
  // Proportional error is Desired - Measured
  PError = SetPoint - Measured;

  // Derivative error is the delta PError divided by time
  // If loop timing is const, you can skip the divide and just make the factor smaller
  DError = Deriv;
  D2Error = DError - LastDError;;

  LastDError = DError;
  LastPError = PError;

  Output = ((Kp * PError) + (Kd * DError) + (Kd2 * D2Error) + (Ki * IError) + RoundOffset) >> Precision;

  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 

  Output = clamp( Output, -MaxOutput , MaxOutput );

  if( DoIntegrate )
  {
    IError += PError;
    IError = clamp( IError, -MaxIntegral, MaxIntegral );
  }    

  return Output;
}


int IntPID::Calculate_PI( int SetPoint , Measured )

  // Proportional error is Desired - Measured
  PError = SetPoint - Measured
  

  PClamped = PError
  if( PMax > 0 )
    PClamped #>= -PMax
    PClamped <#= PMax

  Output = ((Kp * PClamped) + (Ki * IError) + RoundOffset) ~> Precision
  
  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 

  Output = -MaxOutput #> Output <# MaxOutput
     
  PClamped = PError
  if( PIMax > 0 )
    PClamped #>= -PIMax
    PClamped <#= PIMax
   
  IError += PClamped
  IError = -MaxIntegral #> IError <# MaxIntegral  
   
  return Output
*/

