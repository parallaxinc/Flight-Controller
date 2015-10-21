
#include "IntPID.h"


static int clamp( int v, int min, int max ) {
  v = (v < min) ? min : v;
  v = (v > max) ? max : v;
  return v;
}


void IntPID_Init( INTPID * pid, int PGain, int IGain, int DGain, int SampleRate )
{
  pid->SampleRate = SampleRate;
  pid->Kp = PGain;
  pid->Ki = IGain / SampleRate;
  pid->Kd = DGain / SampleRate;
  pid->Kd2 = 0;
  pid->PMax = 0;
  pid->PIMax = 0;
  pid->DerivFilter = 0;
  
  pid->LastPError = 0;
  pid->IError = 0;
  pid->MaxIntegral = 0x010000;
  pid->MaxOutput = 1000;
  pid->Precision = 16;
  pid->RoundOffset = 1 << (pid->Precision-1);
}


int IntPID_Calculate( INTPID * pid, int SetPoint , int Measured , int DoIntegrate )
{
  // Proportional error is Desired - Measured
  pid->PError = SetPoint - Measured;
  
  // Derivative error is the delta PError divided by time
  // If loop timing is const, you can skip the divide and just make the factor smaller
  if( pid->DerivFilter == 0 )
  {
    pid->DError = pid->PError - pid->LastPError;
  }
  else
  {
    int RawDeriv = pid->PError - pid->LastPError;
    pid->DError += ((RawDeriv - pid->DError) * pid->DerivFilter) >> 8;  //Filter the derivative error term so it's not completely nuts
  }

  pid->D2Error = pid->DError - pid->LastDError;

  pid->LastDError = pid->DError;
  pid->LastPError = pid->PError;

  int PClamped = pid->PError;
  if( pid->PMax > 0 ) {
    PClamped = clamp( PClamped, -pid->PMax, pid->PMax );
  }

  pid->Output = ((pid->Kp * PClamped) + (pid->Kd * pid->DError) + (pid->Kd2 * pid->D2Error) + (pid->Ki * pid->IError) + pid->RoundOffset) >> pid->Precision;
  
  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 

  pid->Output = clamp( pid->Output, -pid->MaxOutput, pid->MaxOutput );
     
  if( DoIntegrate )
  {
    PClamped = pid->PError;
    if( pid->PIMax > 0 )
    {
      PClamped = clamp( PClamped, -pid->PIMax, pid->PIMax );

      pid->IError += PClamped;
      pid->IError = clamp( pid->IError, -pid->MaxIntegral, pid->MaxIntegral );
    }
  }

  return pid->Output;
}


int IntPID_Calculate_NoD2( INTPID * pid, int SetPoint , int Measured , int DoIntegrate )
{
  // Proportional error is Desired - Measured
  pid->PError = SetPoint - Measured;
  
  // Derivative error is the delta PError divided by time
  // If loop timing is const, you can skip the divide and just make the factor smaller
  if( pid->DerivFilter == 0 ) {
    pid->DError = pid->PError - pid->LastPError;
  }
  else {
    int RawDeriv = pid->PError - pid->LastPError;
    pid->DError += ((RawDeriv - pid->DError) * pid->DerivFilter) >> 8;  //Filter the derivative error term so it's not completely nuts
  }

  pid->LastDError = pid->DError;
  pid->LastPError = pid->PError;

  int PClamped = pid->PError;
  if( pid->PMax > 0 ) {
    PClamped = clamp( PClamped, -pid->PMax, pid->PMax );
  }

  pid->Output = ((pid->Kp * PClamped) + (pid->Kd * pid->DError) + (pid->Ki * pid->IError) + pid->RoundOffset) >> pid->Precision;
  
  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 

  pid->Output = clamp( pid->Output, -pid->MaxOutput, pid->MaxOutput );
     
  if( DoIntegrate )
  {
    PClamped = pid->PError;
    if( pid->PIMax > 0 )
    {
      PClamped = clamp( PClamped, -pid->PIMax, pid->PIMax );
     
      pid->IError += PClamped;
      pid->IError = clamp( pid->IError, -pid->MaxIntegral, pid->MaxIntegral );
	 }
  }
     
  return pid->Output;
}


int IntPID_Calculate_ForceD( INTPID * pid, int SetPoint , int Measured , int Deriv , int DoIntegrate )
{
  // Proportional error is Desired - Measured
  pid->PError = SetPoint - Measured;

  // Derivative error is the delta PError divided by time
  // If loop timing is const, you can skip the divide and just make the factor smaller
  pid->DError = Deriv;
  pid->D2Error = pid->DError - pid->LastDError;;

  pid->LastDError = pid->DError;
  pid->LastPError = pid->PError;

  pid->Output = ((pid->Kp * pid->PError) + (pid->Kd * pid->DError) + (pid->Kd2 * pid->D2Error) + (pid->Ki * pid->IError) + pid->RoundOffset) >> pid->Precision;

  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 

  pid->Output = clamp( pid->Output, -pid->MaxOutput , pid->MaxOutput );

  if( DoIntegrate )
  {
    pid->IError += pid->PError;
    pid->IError = clamp( pid->IError, -pid->MaxIntegral, pid->MaxIntegral );
  }    

  return pid->Output;
}  


int IntPID_Calculate_ForceD_NoD2( INTPID * pid, int SetPoint , int Measured , int Deriv , int DoIntegrate )
{
  // Proportional error is Desired - Measured
  pid->PError = SetPoint - Measured;
  
  // Derivative error is the delta PError divided by time
  // If loop timing is const, you can skip the divide and just make the factor smaller
  pid->DError = Deriv;

  pid->LastDError = pid->DError;
  pid->LastPError = pid->PError;

  pid->Output = ((pid->Kp * pid->PError) + (pid->Kd * pid->DError) + (pid->Ki * pid->IError) + pid->RoundOffset) >> pid->Precision;

  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 
     
  pid->Output = clamp( pid->Output, -pid->MaxOutput , pid->MaxOutput);
     
  if( DoIntegrate )
  {
    pid->IError += pid->PError;
    pid->IError = clamp( pid->IError, -pid->MaxIntegral , pid->MaxIntegral );
  }     
  return pid->Output;
}  



/*
int IntPID_Calculate_PI( INTPID * pid, int SetPoint , Measured )

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

