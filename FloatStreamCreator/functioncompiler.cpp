#include "functioncompiler.h"
#include "expressionparser.h"
#include <QFile>
#include <QDebug>

FunctionCompiler::FunctionCompiler()
{
}

FunctionCompiler::~FunctionCompiler()
{
}

bool FunctionCompiler::Compile(QByteArray & Source, QByteArray & InputOutputList, QString & OutputPathPrefix )
{
	outPath = OutputPathPrefix;
	ExpressionParser parser;

	// First, prime all the registers
	QList<QByteArray> lines = InputOutputList.split('\n');
	for( int i=0; i<lines.length(); i++)
	{
		QByteArray & line = lines[i];
		if( line.startsWith("INPUT ") ) line.remove(0,5);
		else if( line.startsWith("OUTPUT ")) line.remove(0,6);
		else if( line.startsWith("PERSIST ")) line.remove(0,7);

		parser.Parse(line.data());
	}


	char * src = Source.data();
	int len = Source.length();

	bool FoundStart = false;
	bool FoundEnd = false;
	QString FuncName;

	int pos = 0;
	while( pos < len )
	{
		if( FoundStart == false )
		{
			// Find FUNCTION:<name>
			do {
				if( strncmp(src+pos , "FUNCTION:", 9) == 0 ) {
					FoundStart = true;
					pos += 9;	// Skip FUNCTION:

					// Get the name of the function
					int nameStart = pos;

					// Build a new function entry to hold this (all new expressions go into it)

					// Skip to the next line
					while( pos < len && src[pos] != '\n' ) pos++;
					src[pos] = 0;
					FuncName = src+nameStart;
					pos++;
					FuncName = FuncName.trimmed();	// remove leading / trailing whitespace

					parser.StartFunction( FuncName );

					while( pos < len && (src[pos] == '\n' || src[pos] == '\r')) pos++;
					break;
				}
				else {
					if( (len-pos) > 8 ) {
						pos++;
					}
					else {
						pos = len;
						break;
					}
				}
			} while( !FoundStart && pos < len );
		}
		else
		{
			// Build the current line by scanning for newline
			int start = pos;
			while( pos < len && src[pos] != '\n' ) {
				pos++;
			}

			src[pos] = 0;

			// Check to see if the current line contains // ENDFUNCTION
			for( int i=0; i<=pos-start-11; i++ ) {
				if(strncmp(src+start+i, "ENDFUNCTION", 11) == 0 ) {
					FoundEnd = true;
				}
			}

			if( FoundEnd ) {
				FoundEnd = false;		// NEXT!
				FoundStart = false;
			}
			else {
				// If not, pass it to the parser

				parser.Parse( src + start );

				pos++;	// skip the semicolon
			}
		}
	}

	GenerateOutputCode( parser );

	return true;
}

bool FunctionCompiler::GenerateOutputCode( ExpressionParser & parser )
{
	bool result = GenerateTokens( parser );

	// have to do this second because we generate some temps during function calls
	if( result) result = GenerateVariables( parser );
	return result;
}

bool FunctionCompiler::GenerateVariables( ExpressionParser & parser )
{
	QString filename = outPath + "_vars.inc";

	QByteArray out;
	QTextStream stream( &out );

	for( QMap<QString,EVar*>::iterator iter = parser.varList.begin(); iter != parser.varList.end(); iter++ )
	{
		EVar * var = iter.value();
		stream << var->constName << ",";

		if( var->isConst ) {
			stream << "\t\t // == " << var->valString;
		}

		stream << "\t\t // refcount = " << var->refCount;

		stream << "\n";
	}
	stream.flush();

	QFile f(filename);
	f.open( QFile::WriteOnly );
	f.write(out);
	f.close();

	return true;
}

bool FunctionCompiler::GenerateTokens( ExpressionParser & parser )
{
	for( int i=0; i<parser.funcList.length(); i++ )
	{
		ExprFunc * func = parser.funcList[i];
		QString filename = outPath + '_' + func->name + ".inc";

		QByteArray out;
		QTextStream stream( &out );

		for( int j=0; j<func->exprList.length(); j++ )
		{
			Expression * expr = func->exprList[j];

			if( expr->op == T_Comment ) {
				stream << "\t" << expr->FuncName << "\n";	// line comments just get output as is
			}
			else {
				if( OutputExpressionTokens( parser, stream, expr ) == false ) {
					//return false;
				}
			}
		}

		stream.flush();
		QFile f(filename);
		f.open( QFile::WriteOnly );
		f.write(out);
		f.close();
	}

	return true;
}

bool FunctionCompiler::OutputExpressionTokens( ExpressionParser & parser , QTextStream & stream , Expression * expr )
{
	// Temp vars are used to hold intermediate results during sub-expression parsing
	QString nameLeft = "Temp_lhs";
	QString nameRight = "Temp_rhs";
	pLeftTemp = parser.MakeVariable( T_Float, nameLeft );
	pRightTemp = parser.MakeVariable( T_Float, nameRight );

	EVar * outVar;
	bool result = ComputeSubExpressions( stream, expr, pRightTemp, &outVar );

	return result;
}

bool FunctionCompiler::ComputeSubExpressions( QTextStream & stream , Expression * expr , EVar *pUseTemp , EVar ** outVar )
{
	if( expr == NULL ) {
		*outVar = NULL;		// for unaries, functions with one argument, etc
		return true;
	}

	if( expr->Value != NULL ) {
		*outVar = expr->Value;
		return true;
	}

	bool result = true;
	EVar *leftArg = NULL, *rightArg = NULL;

	if( expr->Left != NULL ) {
		result &= ComputeSubExpressions( stream, expr->Left, pLeftTemp, &leftArg );
	}

	if( expr->Right != NULL )
	{
		// Assign ops are handled by the right sub-expression
		if( expr->op == T_Assign ) {
			result &= ComputeSubExpressions( stream, expr->Right, leftArg, &rightArg );
		}
		else {
			result &= ComputeSubExpressions( stream, expr->Right, pRightTemp, &rightArg );
		}
	}


	// if this op is Assign
	if( expr->op == T_Assign )
	{
		// no GenerateInstruction required for this expr - assign is handled by the sub
		// UNLESS the right sub is a single value, IE the source code was simply A = B
		// In that case, "B" isn't an operation, so we need to generate an opMov instruction

		if( expr->Right->op == T_Label ) {	// Is this a literal or variable?
			result &= GenerateInstruction( stream, expr, rightArg, 0, leftArg );
		}
	}
	else
	{
		if( IsAssign( expr->op ) ) {
			// Build the instruction:
			result &= GenerateInstruction( stream, expr, leftArg, rightArg, leftArg );
			*outVar = leftArg;
		}
		else {
			// Build the instruction:
			result &= GenerateInstruction( stream, expr, leftArg, rightArg, pUseTemp );
			*outVar = pUseTemp;
		}
	}

	return result;
}


static bool OpToFunction( Expression * op, QString & out, QString & outArg )
{
	switch( op->op )
	{
	case T_Float:		out = "opFloat";	break;
	case T_Int:			out = "opTrunc";	break;
	case T_Assign:		out = "opMov";		break;

	case T_Add:
	case T_AddAssign:	out = "opAdd";		break;
	case T_Sub:
	case T_SubAssign:	out = "opSub";		break;
	case T_Mul:
	case T_MulAssign:	out = "opMul";		break;
	case T_Div:
	case T_DivAssign:	out = "opDiv";		break;
	case T_Negative:	out = "opNeg";		break;

	case T_Function:
		if( op->FuncName == "Float" )		out = "opFloat";
		else if( op->FuncName == "Trunc" ) {
			out = "opTruncRound";
			outArg = "0";
		}
		else if( op->FuncName == "Sin" )	out = "opSin";
		else if( op->FuncName == "Cos" )	out = "opCos";
		else if( op->FuncName == "Tan" )	out = "opTan";
		else if( op->FuncName == "Log2" )	out = "opLog2";
		else if( op->FuncName == "SinCos")	out = "opSinCos";
		else if( op->FuncName == "Sqrt")	out = "opSqrt";
		else if( op->FuncName == "FMin")	out = "opFMin";
		else if( op->FuncName == "FAbs")	out = "opFAbs";
		else if( op->FuncName == "Cmp")		out = "opFCmp";
		else if( op->FuncName == "CNeg")	out = "opCNeg";
		else if( op->FuncName == "Shift")	out = "opShift";
		else if( op->FuncName == "ASin") {
			out = "opASinCos";
			outArg = "1";
		}
		else if( op->FuncName == "ACos") {
			out = "opASinCos";
			outArg = "0";
		}
		else if( op->FuncName == "ATan2")	out = "opATan2";
		else {
			qDebug() << "Err: Unknown function: " << op->FuncName;
			return false;
		}
		break;

	default:
		qDebug() << "Err: Unknown operand: " << TokenNames[op->op];
		return false;
	}

	return true;
}

bool FunctionCompiler::GenerateInstruction( QTextStream & stream , Expression * op, EVar * pLeft, EVar * pRight, EVar * pOut )
{
	QString funcName;
	QString arg2('0');

	if( pRight != NULL ) {
		arg2 = pRight->constName;
		pRight->refCount++;
	}

	if( OpToFunction( op , funcName, arg2 ) == false ) {
		stream << "-- Problem line --" << "\n";
		return false;
	}

	stream << "\t" << ("F32_" + funcName + ",").leftJustified(15, ' ');
	if( pLeft != NULL ) {
		pLeft->refCount++;
		stream << pLeft->constName << ", ";
	}
	else {
		stream << "0, ";
	}

	stream << arg2 << ", ";

	if( pOut == NULL ) return false;
	stream << pOut->constName << ",\n";
	pOut->refCount++;

	return true;
}
