#ifndef __INTPID_H__
#define __INTPID_H__

/*
  This file is part of the ELEV-8 Flight Controller Firmware
  for Parallax part #80204, Revision A
  
  Copyright 2015 Parallax Incorporated

  ELEV-8 Flight Controller Firmware is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  ELEV-8 Flight Controller Firmware is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the ELEV-8 Flight Controller Firmware.  If not, see <http://www.gnu.org/licenses/>.
  
  Written by Jason Dorie
*/


class IntPID
{
public:
  void Init( int PGain, int IGain, int DGain, short SampleRate );

  void SetPrecision( unsigned char prec ) {
      Precision = prec;
      RoundOffset = 1 << ((int)Precision-1);
      ScaledMaxOutput = MaxOutput << Precision;
  }

  void SetPGain( int Value )        { Kp = Value; }
  void SetIGain( int Value )        { Ki = Value / (int)SampleRate; }
  void SetDGain( int Value )        { Kd = Value / (int)SampleRate; }
  void SetPMax( int Value )         { PMax = Value; }
  void SetPIMax( int Value )        { PIMax = Value; }
  void SetMaxIntegral( int Value )  { MaxIntegral = Value; }
  void SetMaxOutput( int Value )    { MaxOutput = Value; ScaledMaxOutput = MaxOutput << Precision; }


  //Derivative is normally used "raw", but if the set point or measurement change quickly
  //it can lead to "derivative kick".  The filter value is applied as a fraction over 256.
  //So a filter value of 128 allows 1/2 of the actual change in derivative to feed through
  //in each iteration.  Smaller numbers are a stronger filter.

  void SetDervativeFilter( unsigned char Filter ) { DerivFilter = Filter; }

  void ResetIntegralError(void) { IError = 0; }
  int GetIError(void)           { return IError; }


  int Calculate( int SetPoint , int Measured , char DoIntegrate );
  //int Calculate_ForceD( int SetPoint , int Measured , int Deriv , char DoIntegrate );
  //int Calculate_PI( int SetPoint , int Measured );


public:

  long Kp;             //PID Gain
  long Ki;             //PID Gain
  long Kd;             //PID Gain
  long PMax;           //Maximum P term error value

  unsigned char Precision;     //Number of fixed bits of precision assumed
  unsigned char DerivFilter;   //value from 0 to 255, where 0 is off, 1 to 255 are decreasing filter strength
  short SampleRate;            //updates per second

  long RoundOffset;
  long Output;
  long DError;         //Derivative error (kept for filtering)
  long IError;         //Accumulated integral error
  long LastPError;     //Previous Error
  long MaxIntegral, PIMax;  
  long MaxOutput;
  long ScaledMaxOutput; // MaxOutput << Precision
};


#endif
