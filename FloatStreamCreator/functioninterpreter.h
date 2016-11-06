#ifndef FUNCTIONINTERPRETER_H
#define FUNCTIONINTERPRETER_H

#include <QtGlobal>

enum Operands {
	F32_End			,
	F32_opAdd       ,  // result = a + b
	F32_opSub       ,  // result = a - b
	F32_opMul       ,  // result = a * b
	F32_opDiv       ,  // result = a / b
	F32_opFloat     ,  // result = (float)a
	F32_opTruncRound,  // if(b==0) result = (int)a, else result = (int)round(a)
	F32_opSqrt      ,  // result = Sqrt(a)
	F32_opCmp       ,  // if(a>b) result = 1;  if(a<b) result = -1; else result = 0;
	F32_opSin       ,  // result = Sin(a)
	F32_opCos       ,  // result = Cos(a)
	F32_opTan       ,  // result = Tan(a)
	F32_opLog2      ,  // result = Log2(a)
	F32_opExp2      ,  // result = Exp2(a)
	F32_opPow       ,  // result = Pow(a,b)  (ie a to the power of b)
	F32_opASinCos   ,  // if(b==0) result = ACos(a) else result = ASin(a)
	F32_opATan2     ,  // result = ATan2(a,b)
	F32_opShift     ,  // result = a  x  pow(2, (float)b)  (works like a binary shift, but on floats)
	F32_opNeg       ,  // result = -a
	F32_opSinCos    ,  // result = Sin(a),  b=Cos(a)   (faster than calling opSin(a) + opCos(a)
	F32_opFAbs      ,  // result = FAbs(a)
	F32_opFMin      ,  // if(a<b) result = a  else result = b
	F32_opCNeg      ,  // if(b<0)  a = -a  else  a = a
	F32_opCMov      ,  // if(b==0) a = c  else  a = b
	F32_opMov       ,  // result = a
};

class FunctionInterpreter
{
public:
	static void Run( quint8 * stream , float * floats );
};

#endif // FUNCTIONINTERPRETER_H
