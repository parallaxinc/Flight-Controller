CON

VAR
  long Kp             'PID Gain
  long Ki             'PID Gain
  long Kd             'PID Gain
  long Kd2            'PID Gain
  long PMax           'Maximum P term error value
  long Precision      'Number of fixed bits of precision assumed
  long RoundOffset

  
  long Output         'PID Output
  long PError, PClamped
  long DError, D2Error
  long IError         'Accumulated integral error
  long LastPError     'Previous Error
  long LastDError     'Previous Error
  long MaxIntegral, PIMax  
  long MaxOutput
  long DerivFilter
  long SampleRate

   
PUB Init( PGain, IGain, DGain, _SampleRate )
  SampleRate := _SampleRate
  Kp := PGain
  Ki := IGain / SampleRate
  Kd := DGain / SampleRate
  Kd2 := 0
  PMax := 0
  PIMax := 0
  DerivFilter := 0
  
  LastPError := 0
  IError := 0
  MaxIntegral := $01_0000
  MaxOutput := 1000
  Precision := 16
  RoundOffset := 1 << (Precision-1)


PUB SetPrecision( prec )
  Precision := prec
  RoundOffset := 1 << (Precision-1)

PUB SetPGain( Value )
  Kp := Value  

PUB SetIGain( Value )
  Ki := Value / SampleRate  

PUB SetDGain( Value )
  Kd := Value / SampleRate

PUB SetD2Gain( Value )
  Kd2 := Value / SampleRate
  

PUB SetPMax( Value )
  PMax := Value

PUB SetPIMax( Value )
  PIMax := Value

PUB SetMaxIntegral( Value )
  MaxIntegral := Value

PUB SetMaxOutput( Value )
  MaxOutput := Value



PUB SetDervativeFilter( Filter )

  {Derivative is normally used "raw", but if the set point or measurement change quickly
  it can lead to "derivative kick".  The filter value is applied as a fraction over 256.
  So a filter value of 128 allows 1/2 of the actual change in derivative to feed through
  in each iteration.  Smaller numbers are a stronger filter.}
  
  DerivFilter := Filter


PUB ResetIntegralError
  IError := 0


PUB GetVarAddress
  return @Output


PUB GetIError
  return IError
    

PUB Calculate( SetPoint , Measured , DoIntegrate ) | RawDeriv

  ' Proportional error is Desired - Measured
  PError := SetPoint - Measured
  
  ' Derivative error is the delta PError divided by time
  ' If loop timing is const, you can skip the divide and just make the factor smaller
  if( DerivFilter == 0 )
    DError := PError - LastPError 
  else
    RawDeriv := PError - LastPError
    DError += ((RawDeriv - DError) * DerivFilter) ~> 8  'Filter the derivative error term so it's not completely nuts

  D2Error := DError - LastDError  

  LastDError := DError
  LastPError := PError

  PClamped := PError
  if( PMax > 0 )
    PClamped #>= -PMax
    PClamped <#= PMax

  Output := ((Kp * PClamped) + (Kd * DError) + (Kd2 * D2Error) + (Ki * IError) + RoundOffset) ~> Precision
  
  'Accumulate Integral error *or* Limit output. 
  'Stop accumulating when output saturates 

  Output := -MaxOutput #> Output <# MaxOutput
     
  if( DoIntegrate == TRUE )
    PClamped := PError
    if( PIMax > 0 )
      PClamped := -PIMax #> PClamped <# PIMax
     
    IError += PClamped
    IError #>= -MaxIntegral
    IError <#= MaxIntegral  
     
  return Output


PUB Calculate_NoD2( SetPoint , Measured , DoIntegrate ) | RawDeriv

  ' Proportional error is Desired - Measured
  PError := SetPoint - Measured
  
  ' Derivative error is the delta PError divided by time
  ' If loop timing is const, you can skip the divide and just make the factor smaller
  if( DerivFilter == 0 )
    DError := PError - LastPError 
  else
    RawDeriv := PError - LastPError
    DError += ((RawDeriv - DError) * DerivFilter) ~> 8  'Filter the derivative error term so it's not completely nuts

  LastDError := DError
  LastPError := PError

  PClamped := PError
  if( PMax > 0 )
    PClamped #>= -PMax
    PClamped <#= PMax

  Output := ((Kp * PClamped) + (Kd * DError) + (Ki * IError) + RoundOffset) ~> Precision
  
  'Accumulate Integral error *or* Limit output. 
  'Stop accumulating when output saturates 

  Output := -MaxOutput #> Output <# MaxOutput
     
  if( DoIntegrate == TRUE )
    PClamped := PError
    if( PIMax > 0 )
      PClamped := -PIMax #> PClamped <# PIMax
     
    IError += PClamped
    IError #>= -MaxIntegral
    IError <#= MaxIntegral  
     
  return Output





PUB Calculate_ForceD( SetPoint , Measured , Deriv , DoIntegrate )

  ' Proportional error is Desired - Measured
  PError := SetPoint - Measured
  
  ' Derivative error is the delta PError divided by time
  ' If loop timing is const, you can skip the divide and just make the factor smaller
  DError := Deriv
  D2Error := DError - LastDError  

  LastDError := DError
  LastPError := PError

  Output := ((Kp * PError) + (Kd * DError) + (Kd2 * D2Error) + (Ki * IError) + RoundOffset) ~> Precision
  
  'Accumulate Integral error *or* Limit output. 
  'Stop accumulating when output saturates 
     
  Output := -MaxOutput #> Output <# MaxOutput
     
  if( DoIntegrate )
    IError += PError
    IError := -MaxIntegral #> IError <# MaxIntegral  
     
  return Output


PUB Calculate_ForceD_NoD2( SetPoint , Measured , Deriv , DoIntegrate )

  ' Proportional error is Desired - Measured
  PError := SetPoint - Measured
  
  ' Derivative error is the delta PError divided by time
  ' If loop timing is const, you can skip the divide and just make the factor smaller
  DError := Deriv

  LastDError := DError
  LastPError := PError

  Output := ((Kp * PError) + (Kd * DError) + (Ki * IError) + RoundOffset) ~> Precision
  
  'Accumulate Integral error *or* Limit output. 
  'Stop accumulating when output saturates 
     
  Output := -MaxOutput #> Output <# MaxOutput
     
  if( DoIntegrate )
    IError += PError
    IError := -MaxIntegral #> IError <# MaxIntegral  
     
  return Output


PUB Calculate_PI( SetPoint , Measured )

  ' Proportional error is Desired - Measured
  PError := SetPoint - Measured
  

  PClamped := PError
  if( PMax > 0 )
    PClamped #>= -PMax
    PClamped <#= PMax

  Output := ((Kp * PClamped) + (Ki * IError) + RoundOffset) ~> Precision
  
  'Accumulate Integral error *or* Limit output. 
  'Stop accumulating when output saturates 

  Output := -MaxOutput #> Output <# MaxOutput
     
  PClamped := PError
  if( PIMax > 0 )
    PClamped #>= -PIMax
    PClamped <#= PIMax
   
  IError += PClamped
  IError := -MaxIntegral #> IError <# MaxIntegral  
   
  return Output