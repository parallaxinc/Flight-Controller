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

	// Mark variables with in/out/persist as "alwaysValid"
	// Other variables should be ranged, per function
		// First assign = start
		// last non-assign = end
		// Next assign = new start, etc.
		// Assign is '=' ONLY.  *=, +=, etc are usage, not overwrite

	// Then any variables with non-overlapping ranges can share space
		// Last use / first assign (single line overlap) could be allowed

	// ALL IO variables are considered persistent
	parser.VarsPersist = true;

	// First, prime all the registers
	QList<QByteArray> lines = InputOutputList.split('\n');
	for( int i=0; i<lines.length(); i++)
	{
		QByteArray & line = lines[i];
		if( line.startsWith("INPUT ") ) {
			line.remove(0,5);
		}
		else if( line.startsWith("OUTPUT ")) {
			line.remove(0,6);
		}
		else if( line.startsWith("PERSIST ")) {
			line.remove(0,7);
		}

		parser.Parse(line.data());
	}

	parser.VarsPersist = false;


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

	GenerateOutputCode();
	qDebug() << "Done!";

	return true;
}

bool FunctionCompiler::GenerateOutputCode( void )
{
	bool result = GenerateTokens();

	AssignVariableEnumIndices();

	// have to do this second because we generate some temps during function calls
	if( result) result = GenerateVariables();
	return result;
}

bool FunctionCompiler::GenerateVariables(void)
{
	QString fileVars = outPath + "_vars.inc";
	QString fileInits = outPath + "_var_init.inc";

	QByteArray outVars, outInits;
	QTextStream streamVars( &outVars );
	QTextStream streamInits( &outInits );

	int maxEnum = 0;

	for( QList<EVar*>::iterator iter = parser.orderedVarList.begin(); iter != parser.orderedVarList.end(); iter++ )
	{
		EVar * var = *iter;
		if( var->isConst && var->refCount == 0 ) continue;	// don't output these - they're unused
		if( var->enumValue == -2 ) {
			continue;	// Unused persist variable?
		}

		streamVars << var->constName << " = " << var->enumValue << " ,";
		if( var->enumValue > maxEnum ) maxEnum = var->enumValue;

		if( var->isConst ) {
			streamVars << "\t\t // = " << var->valString;

			if( var->type == T_Int ) {
				streamInits << "	INT_VARS[";
			}
			else if( var->type == T_Float ) {
				streamInits << "	IMU_VARS[";
			}
			streamInits << var->constName << "] = " << var->valString << ";\n";
		}
		streamVars << "\t\t // refcount = " << var->refCount << "\n";
	}

	// Output the final count
	streamVars << "MAX_VAR_INDEX = " << maxEnum << " ,";

	streamVars.flush();
	streamInits.flush();

	QFile f(fileVars);
	f.open( QFile::WriteOnly );
	f.write(outVars);
	f.close();

	QFile f2(fileInits);
	f2.open( QFile::WriteOnly );
	f2.write(outInits);
	f2.close();

	return true;
}

bool FunctionCompiler::GenerateTokens( void )
{
	for( int i=0; i<parser.funcList.length(); i++ )
	{
		funcIndex = i;
		instrIndex = 0;	// reset the generated output instruction index for each function

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
				parser.Optimize(expr);
				if( OutputExpressionTokens( stream, expr ) == false ) {
					//return false;
				}
			}
		}

		stream << "\t0,0,0,0\n";	// Function stream terminator

		stream.flush();
		QFile f(filename);
		f.open( QFile::WriteOnly );
		f.write(out);
		f.close();
	}

	return true;
}

bool FunctionCompiler::OutputExpressionTokens( QTextStream & stream , Expression * expr )
{
	// Temp vars are used to hold intermediate results during sub-expression parsing
	QString nameLeft = "Temp_lhs";
	QString nameRight = "Temp_rhs";

	parser.VarsPersist = true;	// temp vars are persistent
	pLeftTemp = parser.MakeVariable( T_Float, nameLeft );
	pRightTemp = parser.MakeVariable( T_Float, nameRight );
	parser.VarsPersist = false;

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


static bool OpToFunction( ExpressionParser & parser, Expression * op, QString & out, EVar ** outArg )
{
	switch( op->op )
	{
	case T_Float:		out = "opFloat";	break;
	case T_Int:			{
			out = "opTrunc";
			*outArg = parser.MakeVariable(T_Int, QString("0"));
		}
		break;
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
		if( op->FuncName == "Float" ) {
			out = "opFloat";
		}
		else if( op->FuncName == "Trunc" ) {	// trunc with integer output
			out = "opTruncRound";
			*outArg = parser.MakeVariable(T_Int, QString("0"));
		}
		else if( op->FuncName == "Round" ) {	// round with integer output
			out = "opTruncRound";
			*outArg = parser.MakeVariable(T_Int, QString("1"));
		}
		else if( op->FuncName == "FloatTrunc" ) {	// trunc with float output
			out = "opTruncRound";
			*outArg = parser.MakeVariable(T_Int, QString("2"));
		}
		else if( op->FuncName == "FloatRound" ) {	// round with float output
			out = "opTruncRound";
			*outArg = parser.MakeVariable(T_Int, QString("3"));
		}
		else if( op->FuncName == "Sin" )	out = "opSin";
		else if( op->FuncName == "Cos" )	out = "opCos";
		else if( op->FuncName == "Tan" )	out = "opTan";
		else if( op->FuncName == "Log2" )	out = "opLog2";
		else if( op->FuncName == "SinCos")	out = "opSinCos";
		else if( op->FuncName == "Sqrt")	out = "opSqrt";
		else if( op->FuncName == "FMin")	out = "opFMin";
		else if( op->FuncName == "FAbs")	out = "opFAbs";
		else if( op->FuncName == "Cmp")		out = "opCmp";
		else if( op->FuncName == "CNeg")	out = "opCNeg";
		else if( op->FuncName == "CMov")	out = "opCMov";
		else if( op->FuncName == "Shift")	out = "opShift";
		else if( op->FuncName == "ASin") {
			out = "opASinCos";
			*outArg = parser.MakeVariable(T_Int, QString("1"));
		}
		else if( op->FuncName == "ACos") {
			out = "opASinCos";
			*outArg = parser.MakeVariable(T_Int, QString("0"));
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

	if( OpToFunction( parser, op , funcName, &pRight ) == false ) {
		stream << "-- Problem line --" << "\n";
		return false;
	}
	if( pRight != NULL ) {
		arg2 = pRight->constName;
		pRight->refCount++;
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

	UpdateVarScope( pLeft, false, funcIndex, instrIndex );

	if( op->op == T_Function && op->FuncName == "SinCos" ) {
		UpdateVarScope( pRight, true, funcIndex, instrIndex );	// second arg of SinCos is an output / assignment
	}
	else {
		UpdateVarScope( pRight, false, funcIndex, instrIndex );
	}
	UpdateVarScope( pOut, true, funcIndex, instrIndex );
	instrIndex++;

	return true;
}


// This is recursive
void FunctionCompiler::UpdateVarScope( EVar * var , bool isAssign, int iFunc, int iInstr )
{
	if( var == NULL || var->isPersistent || var->isConst ) return;

	if( isAssign )
	{
		// If a variable is assigned here (and isn't persist / const), start a new range

		if( var->ranges.count() > 0 && var->ranges.back().Func == iFunc &&
				(var->ranges.back().Last == iInstr || var->ranges.back().Last == iInstr-1 ) )
		{
			return;	// This is a re-assign, like qw=qw*x  - it's simply another usage, so just extend the range
		}

		ERange range;
		range.Func = iFunc;
		range.First = iInstr;
		range.Last = 0x7fff;	// not terminated at present

		var->ranges.append( range );
	}
	else // Not assigment
	{
		// If a variable is used here (and isn't persist / const), update the end of the last range
		if( var->ranges.count() == 0 ) {
			qDebug() << "Err: Variable used before assignment: " << var->constName;
		}
		else {
			var->ranges.back().Last = iInstr;
		}
	}
}


void FunctionCompiler::AssignVariableEnumIndices(void)
{
	// Assign non-const / persist variables an EnumIndex.  Others will remain -1, as EVar constructor sets them to
	int NextVarIndex = 1;
	int numVars = parser.orderedVarList.count();
	for( int i=0; i<numVars; i++ )
	{
		EVar * var1 = parser.orderedVarList[i];
		if( var1->enumValue >= 0 ) continue;	// already assigned, like const_I0

		if( var1->isConst || var1->isPersistent ) {
			if( var1->refCount != 0 ) {
				var1->enumValue = NextVarIndex++;
			}
			else {
				var1->enumValue = -2;	// Tag these as special - const / persist, but unused
			}
		}
	}

	// For each variable
	for( int i=0; i<numVars; i++ )
	{
		EVar * var1 = parser.orderedVarList[i];

		// If the variable is unassigned, assign it the next available index
		if( var1->enumValue == -1 ) {
			var1->enumValue = NextVarIndex;

			RangeList activeRange = var1->ranges;

			// For all additional variables
			for( int j=i+1; j<numVars; j++ )
			{
				EVar * var2 = parser.orderedVarList[j];

				// If the 2nd is unassigned, and does not overlap the first
				if( var2->enumValue == -1 && RangesOverlap(activeRange, var2->ranges) == false )
				{
					// assign it the same variable index
					var2->enumValue = NextVarIndex;
					MergeRanges( activeRange, var2->ranges );
				}
			}

			NextVarIndex++;
		}
	}
}

bool FunctionCompiler::RangesOverlap( RangeList & set1, RangeList & set2 )
{
	// For each range in var1
	for( QVector<ERange>::iterator r1 = set1.begin(); r1 != set1.end(); r1++ )
	{
		// For each range in var2
		for( QVector<ERange>::iterator r2 = set2.begin(); r2 != set2.end(); r2++ )
		{
			// if range1 overlaps range2
			if( r1->Func == r2->Func )
			{
				if( r1->First >= r2->First && r1->First <= r2->Last ) {
					return true;
				}
				if( r2->First >= r1->First && r2->First <= r1->Last ) {
					return true;
				}
			}
		}
	}
	return false;
}

// THIS ASSUMES THAT THE RANGES DO NOT OVERLAP - they shouldn't, so no
// optimization should be required
void FunctionCompiler::MergeRanges(RangeList &set1, RangeList &set2)
{
	// For each range in set2
	for( QVector<ERange>::iterator r2 = set2.begin(); r2 != set2.end(); r2++ ) {
		set1.append( *r2 );  // add the range to set1 (unsorted)
	}
}
