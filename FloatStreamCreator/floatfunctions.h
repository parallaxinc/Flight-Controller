#ifndef FLOATFUNCTIONS_H
#define FLOATFUNCTIONS_H

/*
The f32 class in the Elev8 flight controller source supports the following list of operations:

#define F32_opAdd                  1    // result = a + b
#define F32_opSub                  2    // result = a - b
#define F32_opMul                  3    // result = a * b
#define F32_opDiv                  4    // result = a / b
#define F32_opFloat                5    // result = (float)a
#define F32_opTruncRound           6    // if(b==0) result = (int)a, else result = (int)round(a)
#define F32_opSqrt                 7    // result = Sqrt(a)
#define F32_opCmp                  8    // if(a>b) result = 1;  if(a<b) result = -1; else result = 0;
#define F32_opSin                  9    // result = Sin(a)
#define F32_opCos                  10   // result = Cos(a)
#define F32_opTan                  11   // result = Tan(a)
#define F32_opLog2                 12   // result = Log2(a)
#define F32_opExp2                 13   // result = Exp2(a)
#define F32_opPow                  14   // result = Pow(a,b)  (ie a to the power of b)
#define F32_opASinCos              15   // if(b==0) result = ACos(a) else result = ASin(a)
#define F32_opATan2                16   // result = ATan2(a,b)
#define F32_opShift                17   // result = a  x  pow(2, (float)b)  (works like a binary shift, but on floats)
#define F32_opNeg                  18   // result = -a
#define F32_opSinCos               19   // result = Sin(a),  b=Cos(a)   (faster than calling opSin(a) + opCos(a)
#define F32_opFAbs                 20   // result = FAbs(a)
#define F32_opFMin                 21   // if(a<b) result = a  else result = b
#define F32_opFrac                 22   // result = fractional portion of a  (portion after the decimal point)
#define F32_opCNeg                 23   // if(b<0)  a = -a  else  a = a
#define F32_opMov                  24   // result = a
*/

float Float(int i);
int   Trunc(float f);
int   Round(float f);
float Sqrt(float f);
int   Cmp(float a, float b);	// Check to see if result is float or int
float Sin(float a);
float Cos(float a);
float Tan(float a);
float Log2(float a);
float Exp2(float a);
float Pow(float a, float b);
float ASin(float a);
float ACos(float a);
float ATan2(float a, float b);
float Shift(float a, int b);
float Neg(float a);
float SinCos(float a, float & outSin);	// out = sin(a), result = cos(a)
float FAbs(float a);
float FMin(float a, float b);
float CNeg(float a, float b);


#endif // FLOATFUNCTIONS_H

