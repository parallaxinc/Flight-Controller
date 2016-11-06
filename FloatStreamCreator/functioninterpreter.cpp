#include "functioninterpreter.h"
#include "floatfunctions.h"
#include <QDebug>

void FunctionInterpreter::Run(quint8 *stream, float * floats)
{
	int * ints = (int *)floats;

	for( ; stream[0] != 0; stream += 4 )
	{
		int aIndex = stream[1];
		int bIndex = stream[2];
		int resIndex = stream[3];

		switch( (Operands)stream[0] )
		{
		case F32_opAdd:
			floats[resIndex] = floats[aIndex] + floats[bIndex];
			break;

		case F32_opSub:
			floats[resIndex] = floats[aIndex] - floats[bIndex];
			break;

		case F32_opMul:
			floats[resIndex] = floats[aIndex] * floats[bIndex];
			break;

		case F32_opDiv:
			floats[resIndex] = floats[aIndex] / floats[bIndex];
			break;

		case F32_opFloat:
			floats[resIndex] = Float(ints[aIndex]);
			break;

		case F32_opTruncRound:
			switch( ints[bIndex] )
			{
			case 0:
				ints[resIndex] = Trunc(floats[aIndex]);
				break;

			case 1:
				ints[resIndex] = Round(floats[aIndex]);
				break;

			case 2:
				floats[resIndex] = FloatTrunc(floats[aIndex]);
				break;

			case 3:
				floats[resIndex] = FloatRound(floats[aIndex]);
				break;

			default:
				qDebug() << "Error: TruncRound invalid B argument";
				break;
			}
			break;

		case F32_opSqrt:
			floats[resIndex] = Sqrt(ints[aIndex]);
			break;

		case F32_opCmp:
			ints[resIndex] = Cmp(floats[aIndex], floats[bIndex]);
			break;

		case F32_opSin:
			floats[resIndex] = Sin(floats[aIndex]);
			break;

		case F32_opCos:
			floats[resIndex] = Cos(floats[aIndex]);
			break;

		case F32_opTan:
			floats[resIndex] = Tan(floats[aIndex]);
			break;

		case F32_opLog2:
			floats[resIndex] = Log2(floats[aIndex]);
			break;

		case F32_opExp2:
			floats[resIndex] = Exp2(floats[aIndex]);
			break;

		case F32_opPow:
			floats[resIndex] = Pow(floats[aIndex], floats[bIndex]);
			break;

		case F32_opASinCos:
			if( ints[bIndex] == 1 ) {
				floats[resIndex] = ASin(floats[aIndex]);
			}
			else if( ints[bIndex] == 0 ){
				floats[resIndex] = ACos(floats[aIndex]);
			}
			else {
				qDebug() << "Error: ASincCos invalid B argument";
			}
			break;

		case F32_opATan2:
			floats[resIndex] = ATan2(floats[aIndex], floats[bIndex]);
			break;

		case F32_opShift:
			floats[resIndex] = Shift(floats[aIndex], ints[bIndex]);
			break;

		case F32_opNeg:
			floats[resIndex] = Neg(floats[aIndex]);
			break;

		case F32_opSinCos:
			floats[resIndex] = SinCos(floats[aIndex], floats[bIndex]);
			break;

		case F32_opFAbs:
			floats[resIndex] = FAbs(floats[aIndex]);
			break;

		case F32_opFMin:
			floats[resIndex] = FMin(floats[aIndex], floats[bIndex]);
			break;

		case F32_opCNeg:
			floats[resIndex] = CNeg(floats[aIndex], floats[bIndex]);
			break;

		case F32_opCMov:
			floats[resIndex] = CMov(floats[aIndex], floats[bIndex]);
			break;

		case F32_opMov:
			floats[resIndex] = floats[aIndex];
			break;

		default:
			qDebug() << "Error: Invalid operand";
			break;
		}
	}
}
