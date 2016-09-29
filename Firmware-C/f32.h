#ifndef __F32_H__
#define __F32_H__


/*
  Elev8 Flight Controller

  F32 - Concise floating point code for the Propeller

  Copyright (c) 2011 Jonathan "lonesock" Dummer
  Ported to C++, modified for stream processing, and new instructions added by Jason Dorie

  C++ API Copyright 2015 Parallax Inc

  Released under the MIT License (see the end of f32_driver.spin for details)
*/


class F32
{
public:
  static int  Start(void);
  static void Stop(void);

  static void RunStream( unsigned char * a, float * b );
  static void WaitStream(void);

  static float FFloat( int n );
  static float FDiv( float a, float b );
};


/*
PUB FAdd(a, b)
PUB FSub(a, b)
PUB FMul(a, b)
PUB FDiv(a, b)
PUB FFloat(n)
//PUB UintTrunc(a)
PUB FTrunc(a)
PUB FRound(a)
PUB FloatTrunc(a)
PUB FloatRound(a)
PUB FSqrt(a)
PUB FCmp(a, b)
PUB Sin(a)
PUB Cos(a)
PUB Tan(a)
PUB Log(a) | b
PUB Log2(a) | b
PUB Log10(a) | b
PUB Exp(a) | b
PUB Exp2(a) | b
PUB Exp10(a) | b
PUB Pow(a, b)
PUB Frac(a)
PUB FNeg(a)
PUB FAbs(a)
PUB Radians(a) | b
PUB Degrees(a) | b
PUB FMin(a, b)
PUB FMax(a, b)
PUB FMod(a, b)
PUB ASin(a) | b
PUB ACos(a) | b
PUB ATan(a) | b
PUB ATan2(a, b)
PUB Floor(a)
PUB Ceil(a)
PUB FShift(a, b)
*/

#define F32_Add                  1    // result = a + b
#define F32_Sub                  2    // result = a - b
#define F32_Mul                  3    // result = a * b
#define F32_Div                  4    // result = a / b
#define F32_Float                5    // result = (float)a
#define F32_TruncRound           6    // if(b==0) result = (int)a, else result = (int)round(a)
#define F32_Sqrt                 7    // result = Sqrt(a)
#define F32_Cmp                  8    // if(a>b) result = 1;  if(a<b) result = -1; else result = 0;
#define F32_Sin                  9    // result = Sin(a)
#define F32_Cos                  10   // result = Cos(a)
#define F32_Tan                  11   // result = Tan(a)
#define F32_Log2                 12   // result = Log2(a)
#define F32_Exp2                 13   // result = Exp2(a)
#define F32_Pow                  14   // result = Pow(a,b)  (ie a to the power of b)
#define F32_ASinCos              15   // if(b==0) result = ACos(a) else result = ASin(a)
#define F32_ATan2                16   // result = ATan2(a,b)
#define F32_Shift                17   // result = a  x  pow(2, (float)b)  (works like a binary shift, but on floats)
#define F32_Neg                  18   // result = -a
#define F32_SinCos               19   // result = Sin(a),  b=Cos(a)   (faster than calling opSin(a) + opCos(a)
#define F32_FAbs                 20   // result = FAbs(a)
#define F32_FMin                 21   // if(a<b) result = a  else result = b
#define F32_Frac                 22   // result = fractional portion of a  (portion after the decimal point)
#define F32_CNeg                 23   // if(b<0)  a = -a  else  a = a
#define F32_Mov                  24   // result = a
#define F32_RunStream            25


//Instruction stream operands are the instruction indices shifted up 2 bits
#define F32_opAdd                  (F32_Add       << 2)  // result = a + b
#define F32_opSub                  (F32_Sub       << 2)  // result = a - b
#define F32_opMul                  (F32_Mul       << 2)  // result = a * b
#define F32_opDiv                  (F32_Div       << 2)  // result = a / b
#define F32_opFloat                (F32_Float     << 2)  // result = (float)a
#define F32_opTruncRound           (F32_TruncRound<< 2)  // if(b==0) result = (int)a, else result = (int)round(a)
#define F32_opSqrt                 (F32_Sqrt      << 2)  // result = Sqrt(a)
#define F32_opCmp                  (F32_Cmp       << 2)  // if(a>b) result = 1;  if(a<b) result = -1; else result = 0;
#define F32_opSin                  (F32_Sin       << 2)  // result = Sin(a)
#define F32_opCos                  (F32_Cos       << 2)  // result = Cos(a)
#define F32_opTan                  (F32_Tan       << 2)  // result = Tan(a)
#define F32_opLog2                 (F32_Log2      << 2)  // result = Log2(a)
#define F32_opExp2                 (F32_Exp2      << 2)  // result = Exp2(a)
#define F32_opPow                  (F32_Pow       << 2)  // result = Pow(a,b)  (ie a to the power of b)
#define F32_opASinCos              (F32_ASinCos   << 2)  // if(b==0) result = ACos(a) else result = ASin(a)
#define F32_opATan2                (F32_ATan2     << 2)  // result = ATan2(a,b)
#define F32_opShift                (F32_Shift     << 2)  // result = a  x  pow(2, (float)b)  (works like a binary shift, but on floats)
#define F32_opNeg                  (F32_Neg       << 2)  // result = -a
#define F32_opSinCos               (F32_SinCos    << 2)  // result = Sin(a),  b=Cos(a)   (faster than calling opSin(a) + opCos(a)
#define F32_opFAbs                 (F32_FAbs      << 2)  // result = FAbs(a)
#define F32_opFMin                 (F32_FMin      << 2)  // if(a<b) result = a  else result = b
#define F32_opFrac                 (F32_Frac      << 2)  // result = fractional portion of a  (portion after the decimal point)
#define F32_opCNeg                 (F32_CNeg      << 2)  // if(b<0)  a = -a  else  a = a
#define F32_opMov                  (F32_Mov       << 2)  // result = a
#define F32_opRunStream            (F32_RunStream << 2)



/*
+------------------------------------------------------------------------------------------------------------------------------+
|                                                   TERMS OF USE: MIT License                                                  |                                                            
+------------------------------------------------------------------------------------------------------------------------------+
|Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    | 
|files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    |
|modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software|
|is furnished to do so, subject to the following conditions:                                                                   |
|                                                                                                                              |
|The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.|
|                                                                                                                              |
|THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          |
|WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         |
|COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   |
|ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         |
+------------------------------------------------------------------------------------------------------------------------------+
*/

#endif
