
#include "IntPID.h"


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
  if( pid->PMax > 0 )
  {
    if( PClamped < -pid->PMax ) PClamped = -pid->PMax;
    else if( PClamped > pid->PMax ) PClamped = pid->PMax;
  }

  pid->Output = ((pid->Kp * PClamped) + (pid->Kd * pid->DError) + (pid->Kd2 * pid->D2Error) + (pid->Ki * pid->IError) + pid->RoundOffset) >> pid->Precision;
  
  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 

  if( pid->Output < -pid->MaxOutput) pid->Output = -pid->MaxOutput;
  else if( pid->Output > pid->MaxOutput) pid->Output =  pid->MaxOutput;
     
  if( DoIntegrate )
  {
    PClamped = pid->PError;
    if( pid->PIMax > 0 )
    {
      if( PClamped < -pid->PIMax ) PClamped = -pid->PIMax;
      else if( PClamped > pid->PIMax ) PClamped = pid->PIMax;

      pid->IError += pid->PClamped;
      if( pid->IError < -pid->MaxIntegral ) pid->IError = -pid->MaxIntegral;
      else if( pid->IError > pid->MaxIntegral ) pid->IError = pid->MaxIntegral;
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
  if( pid->PMax > 0 )
  {
    if( PClamped < -pid->PMax ) PClamped = -pid->PMax;
    else if( PClamped > pid->PMax ) PClamped = pid->PMax;
  }

  pid->Output = ((pid->Kp * PClamped) + (pid->Kd * pid->DError) + (pid->Ki * pid->IError) + pid->RoundOffset) >> pid->Precision;
  
  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 

  if( pid->Output < -pid->MaxOutput) pid->Output = -pid->MaxOutput;
  else if( pid->Output > pid->MaxOutput) pid->Output =  pid->MaxOutput;
     
  if( DoIntegrate )
  {
    PClamped = pid->PError;
    if( pid->PIMax > 0 )
    {
      if( PClamped < -pid->PIMax ) PClamped = -pid->PIMax;
      else if( PClamped > pid->PIMax ) PClamped = pid->PIMax;
     
      pid->IError += pid->PClamped;
	   if( pid->IError < -pid->MaxIntegral ) pid->IError = -pid->MaxIntegral;
	   else if( pid->IError > pid->MaxIntegral ) pid->IError = pid->MaxIntegral;
	 }
  }
     
  return pid->Output;
}


/*


int IntPID_Calculate_ForceD( INTPID * pid, int SetPoint , Measured , Deriv , DoIntegrate )

  // Proportional error is Desired - Measured
  PError = SetPoint - Measured
  
  // Derivative error is the delta PError divided by time
  // If loop timing is const, you can skip the divide and just make the factor smaller
  DError = Deriv
  D2Error = DError - LastDError  

  LastDError = DError
  LastPError = PError

  Output = ((Kp * PError) + (Kd * DError) + (Kd2 * D2Error) + (Ki * IError) + RoundOffset) ~> Precision
  
  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 
     
  Output = -MaxOutput #> Output <# MaxOutput
     
  if( DoIntegrate )
    IError += PError
    IError = -MaxIntegral #> IError <# MaxIntegral  
     
  return Output




int IntPID_Calculate_ForceD_NoD2( INTPID * pid, int SetPoint , Measured , Deriv , DoIntegrate )

  // Proportional error is Desired - Measured
  PError = SetPoint - Measured
  
  // Derivative error is the delta PError divided by time
  // If loop timing is const, you can skip the divide and just make the factor smaller
  DError = Deriv

  LastDError = DError
  LastPError = PError

  Output = ((Kp * PError) + (Kd * DError) + (Ki * IError) + RoundOffset) ~> Precision
  
  //Accumulate Integral error *or* Limit output. 
  //Stop accumulating when output saturates 
     
  Output = -MaxOutput #> Output <# MaxOutput
     
  if( DoIntegrate )
    IError += PError
    IError = -MaxIntegral #> IError <# MaxIntegral  
     
  return Output




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

