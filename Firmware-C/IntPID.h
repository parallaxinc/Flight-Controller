#ifndef __INTPID_H__
#define __INTPID_H__

class IntPID
{
public:
  void Init( int PGain, int IGain, int DGain, int SampleRate );

  void SetPrecision( int prec ) {
      Precision = prec;
      RoundOffset = 1 << (Precision-1);
  }

  void SetPGain( int Value )        { Kp = Value; }
  void SetIGain( int Value )        { Ki = Value / SampleRate; }
  void SetDGain( int Value )        { Kd = Value / SampleRate; }
  void SetD2Gain( int Value )       { Kd2 = Value / SampleRate; }
  void SetPMax( int Value )         { PMax = Value; }
  void SetPIMax( int Value )        { PIMax = Value; }
  void SetMaxIntegral( int Value )  { MaxIntegral = Value; }
  void SetMaxOutput( int Value )    { MaxOutput = Value; }


  //Derivative is normally used "raw", but if the set point or measurement change quickly
  //it can lead to "derivative kick".  The filter value is applied as a fraction over 256.
  //So a filter value of 128 allows 1/2 of the actual change in derivative to feed through
  //in each iteration.  Smaller numbers are a stronger filter.

  void SetDervativeFilter( int Filter ) { DerivFilter = Filter; }


  void ResetIntegralError(void) { IError = 0; }
  int GetIError(void)           { return IError; }


  int Calculate( int SetPoint , int Measured , int DoIntegrate );
  int Calculate_NoD2( int SetPoint , int Measured , int DoIntegrate );
  int Calculate_ForceD( int SetPoint , int Measured , int Deriv , int DoIntegrate );
  int Calculate_ForceD_NoD2( int SetPoint , int Measured , int Deriv , int DoIntegrate );
  int Calculate_PI( int SetPoint , int Measured );


public:

  long Kp;             //PID Gain
  long Ki;             //PID Gain
  long Kd;             //PID Gain
  long Kd2;            //PID Gain
  long PMax;           //Maximum P term error value
  long Precision;      //Number of fixed bits of precision assumed
  long RoundOffset;

  
  long Output;         //PID Output
  long PError, PClamped;
  long DError, D2Error;
  long IError;         //Accumulated integral error
  long LastPError;     //Previous Error
  long LastDError;     //Previous Error
  long MaxIntegral, PIMax;  
  long MaxOutput;
  long DerivFilter;
  long SampleRate;
};


#endif
