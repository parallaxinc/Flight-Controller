
#include <stdio.h>
#include <cog.h>
#include <sys/driver.h>
#include <propeller.h>
#include "F32.h"


 
static volatile long  f32_cmd;
static char  cog;

static int* CommandAddr[8];
static int  TempCommand, StreamAddr;
static int* cmdCallTableAddr = 0;

int F32_Start(void)
{
//  Start start floating point engine in a new cog.
//  Returns:     True (non-zero) if cog started, or False (0) if no cog is available.

  F32_Stop();
  f32_cmd = 0;

  use_cog_driver(F32_driver);

  uint32_t * driverMem = get_cog_driver(F32_driver);
  int i=0;
  while( driverMem[i] != 0x12345678 )
    i++;

  cmdCallTableAddr = (int *)driverMem + i + 1;

  load_cog_driver(F32_driver, &f32_cmd);
  return cog;
}


void F32_Stop(void)
{
// Stop floating point engine and release the cog.

	if(cog) {
    cogstop(cog - 1);
    cog = 0;
	}
}


void F32_StartStream( int index, int* baseAddr )
{
  CommandAddr[index] = baseAddr;
}


void F32_AddCommand( int index, int fp_op, void* a_addr, void* b_addr, void* out_addr )
{
  int *addr = CommandAddr[index];
  addr[0] = (fp_op << 2) + (int)cmdCallTableAddr;  //This is a pointer into the JMPRET instruction table
  addr[1] = (int)a_addr;
  addr[2] = (int)b_addr;                           //Prop memory addresses are only 16 bit
  addr[3] = (int)out_addr;

  CommandAddr[index] = addr + 4;
}

void F32_EndStream( int index )
{
  int *addr = CommandAddr[index];
  addr[0] = 0;					         //Use zero to indicate end-of-stream 
  CommandAddr[index]++;

  //return CommandAddr[index];      //Allows the caller to figure out how much space this actually took
}


int* F32_GetCommandPtr( int fp_op )
{
  return cmdCallTableAddr + fp_op;
}
      

void F32_RunStream( int * a )
{
  //Can't use the stack for these, because they might be different by the time the COG gets to them
  TempCommand = cmdCallTableAddr[ F32_opRunStream ];
  StreamAddr = (int)a;
  f32_cmd = (int)&TempCommand;
}


void F32_WaitStream(void)
{
	while( f32_cmd )
		;
}

/*
void F32_Cmd_ptr(void)
{
//  return the Hub address of f32_Cmd, so other code can call F32 functions directly
	return &f32_cmd;
}


void F32_Call_ptr(void)
{
//  return the Hub address of the dispatch table, so other code can call F32 functions directly
	//return &cmdCallTable;
}
*/


#if 0
PUB FAdd(a, b)
/*
  Addition: result = a + b
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value
  Returns:   32-bit floating point value
*/
  result  := cmdFAdd
  f32_Cmd := @result
  repeat
  while f32_Cmd
          
PUB FSub(a, b)
/*
  Subtraction: result = a - b
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value
  Returns:   32-bit floating point value
*/
  result  := cmdFSub
  f32_Cmd := @result
  repeat
  while f32_Cmd
  
PUB FMul(a, b)
/*
  Multiplication: result = a * b
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value
  Returns:   32-bit floating point value
*/
  result  := cmdFMul
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB FDiv(a, b)
/*
  Division: result = a / b
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value
  Returns:   32-bit floating point value
*/
  result  := cmdFDiv
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB FFloat(n)
/*
  Convert integer to floating point.
  Parameters:
    n        32-bit integer value
  Returns:   32-bit floating point value
*/
  result  := cmdFFloat
  f32_Cmd := @result
  repeat
  while f32_Cmd

//PUB UintTrunc(a)
/*
  Convert floating point to unsigned integer (with truncation).
  Parameters:
    a        32-bit floating point value
  Returns:   32-bit unsigned integer value
  (negative values are clamped to 0)
*/

//  result  := cmdUintTrunc
//  f32_Cmd := @result
//  repeat
//  while f32_Cmd

PUB FTrunc(a) | b
/*
  Convert floating point to integer (with truncation).
  Parameters:
    a        32-bit floating point value
    b        flag: 0 signifies truncation
  Returns:   32-bit integer value
*/
  b       := %000
  result  := cmdFTruncRound
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB FRound(a) | b
/*
  Convert floating point to integer (with rounding).
  Parameters:
    a        32-bit floating point value
    b        flag: 1 signifies rounding to the nearest integer
  Returns:   32-bit integer value
*/
  b       := %001
  result  := cmdFTruncRound
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB FloatTrunc(a) | b
/*
  Convert floating point to whole number (floating point, with truncation).
  Parameters:
    a        32-bit floating point value
    b        flag: 2 signifies floating point truncation
  Returns:   32-bit floating point value
*/
  b       := %010
  result  := cmdFTruncRound
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB FloatRound(a) | b
/*
  Convert floating point to whole number (floating point, with rounding).
  Parameters:
    a        32-bit floating point value
    b        flag: 3 signifies floating point rounding to the nearest whole number
  Returns:   32-bit floating point value
*/
  b       := %011
  result  := cmdFTruncRound
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB FSqrt(a)
/*
  Square root.
  Parameters:
    a        32-bit floating point value
  Returns:   32-bit floating point value
*/
  result  := cmdFSqrt
  f32_Cmd := @result
  repeat
  while f32_Cmd


PUB FCmp(a, b)
/*
  Floating point comparison.
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value
  Returns:   32-bit integer value
             -1 if a < b
              0 if a == b
              1 if a > b
*/
  result  := cmdFCmp
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Sin(a)
/*
  Sine of an angle (radians).
  Parameters:
    a        32-bit floating point value (angle in radians)
  Returns:   32-bit floating point value
*/
  result  := cmdFSin
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Cos(a)
/*
  Cosine of an angle (radians).
  Parameters:
    a        32-bit floating point value (angle in radians)
  Returns:   32-bit floating point value
*/
  result  := cmdFCos
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Tan(a)
/*
  Tangent of an angle (radians).
  Parameters:
    a        32-bit floating point value (angle in radians)
  Returns:   32-bit floating point value
*/
  result  := cmdFTan
  f32_Cmd := @result
  repeat
  while f32_Cmd


PUB Log(a) | b
/*
  Logarithm, base e.
  Parameters:
    a        32-bit floating point value
    b        constant used to convert base 2 to base e
  Returns:   32-bit floating point value
*/
  b       := 1.442695041
  result  := cmdFLog2
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Log2(a) | b
/*
  Logarithm, base 2.
  Parameters:
    a        32-bit floating point value
    b        0 is a flag to skip the base conversion (skips a multiplication by 1.0)
  Returns:   32-bit floating point value
*/
  b       := 0
  result  := cmdFLog2
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Log10(a) | b
/*
  Logarithm, base 10.
  Parameters:
    a        32-bit floating point value
    b        constant used to convert base 2 to base 10
  Returns:   32-bit floating point value
*/
  b       := 3.321928095
  result  := cmdFLog2
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Exp(a) | b
/*
  Exponential (e raised to the power a).
  Parameters:
    a        32-bit floating point value
    b        constant used to convert base 2 to base e
  Returns:   32-bit floating point value
*/
  b       := 1.442695041
  result  := cmdFExp2
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Exp2(a) | b
/*
  Exponential (2 raised to the power a).
  Parameters:
    a        32-bit floating point value
    b        0 is a flag to skip the base conversion (skips a division by 1.0)
  Returns:   32-bit floating point value
*/
  b       := 0
  result  := cmdFExp2
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Exp10(a) | b
/*
  Exponential (10 raised to the power a).
  Parameters:
    a        32-bit floating point value
    b        constant used to convert base 2 to base 10
  Returns:   32-bit floating point value
*/
  b       := 3.321928095
  result  := cmdFExp2
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Pow(a, b)
/*
  Power (a to the power b).
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value  
  Returns:   32-bit floating point value
*/
  result  := cmdFPow
  f32_Cmd := @result
  repeat
  while f32_Cmd


{
PUB Frac(a)
/*
  Fraction (returns fractional part of a).
  Parameters:
    a        32-bit floating point value
  Returns:   32-bit floating point value
*/
  result  := cmdFFrac
  f32_Cmd := @result
  repeat
  while f32_Cmd
}

PUB FNeg(a)
/*
  Negate: result = -a.
  Parameters:
    a        32-bit floating point value
  Returns:   32-bit floating point value
*/
  return a ^ $8000_0000

PUB FAbs(a)
/*
  Absolute Value: result = |a|.
  Parameters:
    a        32-bit floating point value
  Returns:   32-bit floating point value
*/
  return a & $7FFF_FFFF
  
PUB Radians(a) | b
/*
  Convert degrees to radians
  Parameters:
    a        32-bit floating point value (angle in degrees)
    b        the conversion factor
  Returns:   32-bit floating point value (angle in radians)
*/
  b       := constant(pi / 180.0)
  result  := cmdFMul
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Degrees(a) | b
/*
  Convert radians to degrees
  Parameters:
    a        32-bit floating point value (angle in radians)
    b        the conversion factor
  Returns:   32-bit floating point value (angle in degrees)
*/
  b       := constant(180.0 / pi)
  result  := cmdFMul
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB FMin(a, b)
/*
  Minimum: result = the minimum value a or b.
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value  
  Returns:   32-bit floating point value
*/
  result  := cmdFCmp
  f32_Cmd := @result
  repeat
  while f32_Cmd
  if result < 0
    return a
  return b
  
PUB FMax(a, b)
/*
  Maximum: result = the maximum value a or b.
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value  
  Returns:   32-bit floating point value
*/
  result  := cmdFCmp
  f32_Cmd := @result
  repeat
  while f32_Cmd
  if result < 0
    return b
  return a

{
PUB FMod(a, b)
/*
  Floating point remainder: result = the remainder of a / b.
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value  
  Returns:   32-bit floating point value
*/
  result  := cmdFMod
  f32_Cmd := @result
  repeat
  while f32_Cmd
}

PUB ASin(a) | b
/*
  Arc Sine of a (in radians).
  Parameters:
    a        32-bit floating point value (|a| must be < 1)
    b        1 is a flag signifying return the sine component
  Returns:   32-bit floating point value (angle in radians)
*/
  b       := 1
  result  := cmdASinCos
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB ACos(a) | b
/*
  Arc Cosine of a (in radians).
  Parameters:
    a        32-bit floating point value (|a| must be < 1)
    b        0 is a flag signifying return the cosine component
  Returns:   32-bit floating point value (angle in radians)
             if |a| > 1, NaN is returned
*/
  b       := 0
  result  := cmdASinCos
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB ATan(a) | b
/*
  Arc Tangent of a.
  Parameters:
    a        32-bit floating point value
    b        atan(a) = atan2(a,1.0)
  Returns:   32-bit floating point value (angle in radians)
*/
  b       := 1.0
  result  := cmdATan2
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB ATan2(a, b)
/*
  Arc Tangent of vector a, b (in radians, no division is performed, so b==0 is legal).
  Parameters:
    a        32-bit floating point value
    b        32-bit floating point value
  Returns:   32-bit floating point value (angle in radians)
*/
  result  := cmdATan2
  f32_Cmd := @result
  repeat
  while f32_Cmd

{
PUB Floor(a)
/*
  Calculate the floating point value of the nearest integer <= a.
  Parameters:
    a        32-bit floating point value
  Returns:   32-bit floating point value
*/
  result  := cmdFloor
  f32_Cmd := @result
  repeat
  while f32_Cmd

PUB Ceil(a)
/*
  Calculate the floating point value of the nearest integer >= a.
  Parameters:
    a        32-bit floating point value
  Returns:   32-bit floating point value
*/
  result  := cmdCeil
  f32_Cmd := @result
  repeat
  while f32_Cmd
}

PUB FShift(a, b)
/*
  Float-Shift - multiply by POW(2,a), identical to a SHL or SHR in integer
  Parameters:
    a        32-bit floating point value
    b        32-bit integer value (shift amount)
  Returns:   32-bit floating point value
*/
  result  := cmdShift
  f32_Cmd := @result
  repeat
  while f32_Cmd

#endif


