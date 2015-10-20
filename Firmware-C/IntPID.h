#ifndef __INTPID_H__
#define __INTPID_H__

typedef struct {
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
} INTPID;


void IntPID_Init( INTPID * pid, int PGain, int IGain, int DGain, int SampleRate );

inline void IntPID_SetPrecision( INTPID * pid, int prec )
{
  pid->Precision = prec;
  pid->RoundOffset = 1 << (pid->Precision-1);
}

inline void IntPID_SetPGain( INTPID * pid, int Value )        { pid->Kp = Value; }
inline void IntPID_SetIGain( INTPID * pid, int Value )        { pid->Ki = Value / pid->SampleRate; }
inline void IntPID_SetDGain( INTPID * pid, int Value )        { pid->Kd = Value / pid->SampleRate; }
inline void IntPID_SetD2Gain( INTPID * pid, int Value )       { pid->Kd2 = Value / pid->SampleRate; }
inline void IntPID_SetPMax( INTPID * pid, int Value )         { pid->PMax = Value; }
inline void IntPID_SetPIMax( INTPID * pid, int Value )        { pid->PIMax = Value; }
inline void IntPID_SetMaxIntegral( INTPID * pid, int Value )  { pid->MaxIntegral = Value; }
inline void IntPID_SetMaxOutput( INTPID * pid, int Value )    { pid->MaxOutput = Value; }


inline void IntPID_SetDervativeFilter( INTPID * pid, int Filter )
{
  //Derivative is normally used "raw", but if the set point or measurement change quickly
  //it can lead to "derivative kick".  The filter value is applied as a fraction over 256.
  //So a filter value of 128 allows 1/2 of the actual change in derivative to feed through
  //in each iteration.  Smaller numbers are a stronger filter.
  
  pid->DerivFilter = Filter;
}


inline void IntPID_ResetIntegralError(INTPID * pid) { pid->IError = 0; }

inline int IntPID_GetIError( INTPID * pid ) { return pid->IError; }


int IntPID_Calculate( INTPID * pid, int SetPoint , int Measured , int DoIntegrate );
int IntPID_Calculate_NoD2( INTPID * pid, int SetPoint , int Measured , int DoIntegrate );
int IntPID_Calculate_ForceD( INTPID * pid, int SetPoint , int Measured , int Deriv , int DoIntegrate );
int IntPID_Calculate_ForceD_NoD2( INTPID * pid, int SetPoint , int Measured , int Deriv , int DoIntegrate );
int IntPID_Calculate_PI( INTPID * pid, int SetPoint , int Measured );

#endif
