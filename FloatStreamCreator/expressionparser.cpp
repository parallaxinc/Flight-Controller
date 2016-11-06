#include "expressionparser.h"
#include "expressiontokenizer.h"

#include <QDebug>
#include <math.h>

static bool DumpOutput = false;

ExpressionParser::ExpressionParser( )
{
	varType = T_None;
	isConst = false;
	exprList = NULL;
	SourceLine = 0;


	//The interpreter needs zero, one, two, three to be the first entries in the const table, used by TruncRound function
	QString zeroConstName("0.0");
	EVar * fZero = MakeVariable( T_Float, zeroConstName );
	fZero->enumValue = 0;	// Float zero is the same as int zero, so put them in the same location

	QString constName("0");
	for( int i=0; i<4; i++ ) {
		constName[0] = '0' + i;
		EVar * var = MakeVariable( T_Int, constName );
		if( i == 0 ) {	// only assign "0" a set / fixed index
			var->enumValue = i;
		}
	}
}

ExpressionParser::~ExpressionParser()
{
	// need to dealloc vars and expressions
}

void ExpressionParser::StartFunction( QString & FuncName )
{
	ExprFunc * pFunc = new ExprFunc();
	pFunc->name = FuncName;
	exprList = &pFunc->exprList;

	funcList.append( pFunc );

	SourceLine = 0;
}


bool ExpressionParser::Parse( const char * pSrc )
{
	varType = T_None;

	if( DumpOutput ) {
		qDebug() << "Ln:" << SourceLine << " " << pSrc;
	}

	if( exprList != NULL )
	{
		// Build "Comment" expressions per line, and append them to the list here so the output is commented
		Expression * lineComment = new Expression();
		lineComment->op = T_Comment;
		QString temp = QString(pSrc).trimmed();
		if( temp.startsWith("//") == false ) {
			temp = "// " + temp;
		}
		lineComment->FuncName = temp;
		exprList->append(lineComment);
	}
	SourceLine++;

	tok.SetSource( pSrc );
	tok.Advance(); // prime it - first call only populates "Next" token

	while( tok.Advance() ) {
		TOKEN t = tok.token;
		TOKEN last = tok.last;
		QString s = tok.value;

		switch( t )
		{
		case T_Const:
			if( last == T_None || last == T_Break || last == T_Comment ) {
				isConst = true;
			}
			else {
				qDebug() << "Err: got a type modifier mid-expression";
				qDebug() << "Ln:" << SourceLine << " " << pSrc;
				return false;	// parse error
			}
			break;

		case T_Float:
		case T_Int:
			if( varType == T_None && (last == T_None || last == T_Break || last == T_Const || last == T_Comment) ) {
				varType = t;
			}
			else {
				qDebug() << "Err: got a type declaration mid-expression";
				qDebug() << "Ln:" << SourceLine << " " << pSrc;
				return false;	// parse error
			}
			break;

		case T_Label:
			labelName = s;
			if( IsType(last) || last == T_Break || last == T_None || last == T_Comment )
			{
				// this is a variable definition
				Expression * expr = MakeVariableExpression(varType, s);
				exprStack.push( expr );
			}
			break;

		case T_Assign:
		case T_AddAssign:
		case T_SubAssign:
		case T_MulAssign:
		case T_DivAssign:
			{
				if( exprStack.empty() ) return false;	// Nothing on the stack?  That's an error

				// should be a T_Label expression on the stack (the val being assigned)
				// it should not be const
				Expression * lhs = exprStack.pop();
				if( lhs->op != T_Label || (lhs->Value->isConst && !isConst) ) {
					exprStack.push(lhs);
					qDebug() << "Err: parsing assignment, but stack-top is not a variable we can assign to";
					qDebug() << "Ln:" << SourceLine << " " << pSrc;
					return false;
				}

				Expression * rhs = GetExpression();
				if( rhs == NULL ) {
					qDebug() << "Err: Atom() function returned null extracting right-hand-side for assignment";
					qDebug() << "Ln:" << SourceLine << " " << pSrc;
					return false;	// parse error?
				}

				if( isConst && lhs->Value->isConst ) // actually assigning a const initializer here
				{
					if( rhs->Value == NULL || rhs->Value->isConst != true )
					{
						qDebug() << "Err: Cannot assign a non-numeric value to a const";
						qDebug() << "Ln:" << SourceLine << " " << pSrc;
						return false;
					}

					TOKEN type = varType;
					if( lhs->Value->valString.length() == 0 ) {
						AssignValueToVariable( lhs->Value, type, rhs->Value->valString );
					}
					else {
						if( lhs->Value->valString != rhs->Value->valString ) {
							qDebug() << "Err: Attempting to re-assign a const to a different value";
							qDebug() << "Ln:" << SourceLine << " " << pSrc;
						}
					}

					// don't need these any more
					delete rhs;
					delete lhs;

					if( tok.next == T_Break || tok.next == T_End ) {
						tok.Advance();
						if( tok.token == T_End ) {
							varType = T_None;	// clear this when encountering a semicolon
							isConst = false;
						}
					}
				}
				else {
					Expression * expr = new Expression();
					expr->line = SourceLine-1;
					expr->op = t;
					expr->Left = lhs;
					expr->Right = rhs;
					exprStack.push(expr);
				}
			}
			break;

		case T_Break:	// Comma delimiter
		case T_End:		// semi-colon delimiter
			{
				// Current expression is done, another may follow
				Expression * expr = exprStack.pop();
				if( expr->op == T_Label ) {
					delete expr;
				}
				else {
					if( exprList != NULL ) {
						exprList->append( expr );
					}
					else {
						qDebug() << "Err: No expression list to store expressions to!  Call StartFunction() first";
						qDebug() << "Ln:" << SourceLine << " " << pSrc;
					}
					if( DumpOutput ) {
						expr->Dump();
					}
				}

				if( t == T_End ) {
					varType = T_None;	// clear this when encountering a semicolon
					isConst = false;
				}
			}
			break;

		case T_Comment:
			break;

		default:
			qDebug() << "Warn: Hit default case in parse";
			qDebug() << "Ln:" << SourceLine << " " << pSrc;
			break;
		}
	}

	return true;
}


Expression * ExpressionParser::GetExpression( void )
{
	return SubExpression( Atom(), 0 );
}


Expression * ExpressionParser::Atom( void )
{
	Expression * expr = NULL;

	tok.Advance();
	switch( tok.token )
	{
	case T_Digit:
		return MakeVariableExpression( tok.token, tok.value );

	case T_Positive:
	case T_Negative:	// Unary positive / negative
		{
			TOKEN t = tok.token;	// remember it

			Expression * lhs = Atom();
			if( lhs == NULL ) return NULL;

			Expression * expr = new Expression();
			expr->op = t;
			expr->Left = lhs;
			return expr;
		}
		break;

	case T_Label:
		if( tok.next == T_OpenParen ) {
			// this is a function call
			Expression * expr = new Expression();
			expr->op = T_Function;
			expr->FuncName = tok.value;	// name of the function

			if( ParseArguments(expr) ) {
				return expr;
			}
			else {
				qDebug() << "Err: Function argument parsing failed";
				delete expr;
				return NULL;
			}
		}
		else {
			return MakeVariableExpression( varType, tok.value );
		}

	case T_OpenParen:
		expr = GetExpression();
		if( tok.next == T_CloseParen ) {
			tok.Advance();	// eat the closeParen
		}
		else {
			qDebug() << "Err: mismatched close paren?";
		}
		break;

	default:
		break;
	}

	return expr;
}

void ExpressionParser::AssignValueToVariable( EVar * var , TOKEN & type, const QString & s )
{
	var->valString = s;	// just using the actual digits of the const as the name
	var->type = type;
	var->isConst = isConst;

	bool ok;
	float asFloat = s.toFloat(&ok);
	if( ok ) {
		if( type == T_Digit ) {
			if( s.contains('.') ) {
				var->type = type = T_Float;
			}
			else {
				var->type = type = T_Int;
			}
		}

		QString prefix = "F";
		if( type == T_Float ) {
			var->Val.f = asFloat;
			prefix = "F";
		}
		else {
			var->Val.i = (int)asFloat;
			prefix = "I";
		}
		QString pretty = s;
		pretty.replace("-", "Neg_").replace(".","_");

		var->isConst = true;
		// Only assign a name if it doesn't have one already
		if( var->constName.length() == 0 ) {
			var->constName = "const_" + prefix + pretty;
		}
	}
	else {
		var->constName = var->valString;
	}
}

EVar * ExpressionParser::MakeVariable( TOKEN type, const QString & s )
{
	EVar * var;
	if( varList.contains(s) ) {
		var = varList[s];	// look up an existing const by digits (might have dupes if people type "0.00", "0", "0.0"...

		//if( var->type != type || var->isConst != isConst ) {
		//	qDebug() << "Err: variable declaration with different types encountered";
		//}
	}
	else {
		if( type != T_Int && type != T_Float && type != T_Digit ) {
			qDebug() << "Err: declaring a variable with no type";
			return NULL;
		}

		var = new EVar();
		var->isPersistent = VarsPersist;
		var->refCount = 0;
		var->varIndex = orderedVarList.length();

		AssignValueToVariable( var, type, s );

		varList.insert( var->valString, var );
		orderedVarList.append( var );	// Track original order so we can output them in a guaranteed order

		if( DumpOutput ) {
			qDebug() << "Declared " << (isConst ? "const " : "") << TokenNames[type] << " : " << s;
		}
	}
	return var;
}


Expression * ExpressionParser::MakeVariableExpression( TOKEN type, const QString & s )
{
	Expression * expr = new Expression();
	expr->Value = MakeVariable( type, s );
	expr->op = T_Label;

	return expr;
}

static int Precedence( TOKEN t )
{
	switch( t )
	{
	case T_Assign:
	case T_AddAssign:
	case T_SubAssign:
	case T_MulAssign:
	case T_DivAssign:	return 50;

	case T_Positive:
	case T_Negative:	return 20;

	case T_Mul:
	case T_Div:			return 10;

	case T_Add:
	case T_Sub:			return 5;

	case T_Digit:
	case T_Label:		return 0;

	default:			return -1;	// not an operation : finish
	}
}


Expression * ExpressionParser::SubExpression( Expression * lhs, int MinPrec )
{
	while( true ) {
		TOKEN op = tok.next;

		int prec = Precedence(op);
		if( prec < MinPrec ) break;

		tok.Advance();

		Expression * rhs = Atom();
		int nextPrec;

		while( true ) {
			TOKEN nextOp = tok.next;
			nextPrec = Precedence(nextOp);
			if( nextPrec <= prec ) {
				break;
			}

			rhs = SubExpression(rhs, nextPrec );
		}

		Expression * expr = new Expression();
		expr->op = op;
		expr->Left = lhs;
		expr->Right = rhs;

		lhs = expr;
		if( nextPrec < 0 ) {
			break;
		}
	}

	return lhs;
}


struct FUNCINFO {
	const char * Name;
	int			 ArgCount;
} FuncInfo[] = {
	"Float",	1,
	"Int",		1,
	"Sin",		1,
	"Cos",		1,
	"ASin",		1,
	"ACos",		1,
	"Sqrt",		1,
	"SinCos",	2,
	"ATan2",	2,
	"FMin",		2,
	"FAbs",		1,
	"Trunc",	1,
	"Round",	1,
	"FloatTrunc",1,
	"FloatRound",1,
	"Cmp",		2,
	"CNeg",     2,
	"CMov",     2,
	"Shift",    2,
};



bool ExpressionParser::ParseArguments( Expression * funcCall )
{
	tok.Advance();	// Eat the OpenParen

	int Count = sizeof(FuncInfo) / sizeof(FUNCINFO);
	int FuncIndex = -1;
	for( int i=0; i<Count; i++ )
	{
		if( funcCall->FuncName == FuncInfo[i].Name ) {
			FuncIndex = i;
			break;
		}
	}

	if( FuncIndex == -1 ) {
		qDebug() << "Err: Function not found: " << funcCall->FuncName;
		return false;
	}

	int ArgCount = FuncInfo[FuncIndex].ArgCount;
	int ArgsFound = 0;

	while( ArgsFound < ArgCount )
	{
		Expression * expr = GetExpression();
		if( expr == NULL ) {
			return false;
		}

		if( ArgsFound == 0 ) {
			funcCall->Left = expr;
		}
		else {
			funcCall->Right = expr;
		}
		ArgsFound++;

		if( tok.next != T_Break && tok.next != T_CloseParen ) {
			return false;
		}

		tok.Advance();	// eat the comma or CloseParen
	}

	return true;
}

void ExpressionParser::Optimize( Expression * expr )
{
	if( expr->op == T_Mul || expr->op == T_Div ) {
		// if one of the terms is const, and a power of two, change this to a shift

		if( expr->op == T_Mul && expr->Left->Value != NULL && expr->Left->Value->type == T_Float && expr->Left->Value->Val.i != 0 )
		{
			// Note that we can ONLY optimize Val x Val on both sides - Mul is commutative, Div isn't
			// IE, 2 / Val isn't the same as Val / 2, but 2 x Val *IS* the same as Val x 2

			float lg = log2(expr->Left->Value->Val.f);
			//qDebug() << expr->Left->Value->Val.f << " = " << lg;
			if( ((float)(int)lg) == lg ) {
				QString s = QString::asprintf( "%d", (int)lg );
				EVar * shiftVar = MakeVariable(T_Int, s);
				expr->op = T_Function;
				expr->FuncName = "Shift";
				expr->SwapArguments();
				expr->Right->Value = shiftVar;
				return;
			}
		}
		else if( expr->Right->Value != NULL && expr->Right->Value->type == T_Float && expr->Right->Value->Val.i != 0)
		{
			// Convert multiply or divide by powers of two to Shift operations instead
			float lg = log2(expr->Right->Value->Val.f);
			//qDebug() << expr->Right->Value->Val.f << " = " << lg;
			if( ((float)(int)lg) == lg ) {
				if( expr->op == T_Div ) lg = -lg;
				QString s = QString::asprintf( "%d", (int)lg );
				EVar * shiftVar = MakeVariable(T_Int, s);
				expr->op = T_Function;
				expr->FuncName = "Shift";
				expr->Right->Value = shiftVar;
				return;
			}
		}
	}

	if( expr->op == T_Add && expr->Left->op == T_Negative )
	{
		// Convert "-A + B" into "B - A"

		Expression * pTemp = expr->Left;
		expr->Left = pTemp->Left;	// Negation only has one argument, we're making it OUR left argument
		pTemp->Left = NULL;			// unhook this so we don't delete the sub-tree
		delete pTemp;
		expr->SwapArguments();
		expr->op = expr->op == T_Add ? T_Sub : T_Add;	// Flip the operation we were doing
	}
	else if( (expr->op == T_Sub || expr->op == T_Add) && expr->Right->op == T_Negative )
	{
		// Convert "A + -B" into "A - B"
		//    or   "A - -B" into "A + B"

		Expression * pTemp = expr->Right;
		expr->Right = pTemp->Left;	// Negation only has one argument, we're making it OUR right argument
		pTemp->Left = NULL;			// unhook this so we don't delete the sub-tree
		delete pTemp;
		expr->op = expr->op == T_Add ? T_Sub : T_Add;	// Flip the operation we were doing
	}

	if(expr->Left != NULL)  Optimize( expr->Left );
	if(expr->Right != NULL) Optimize( expr->Right );
}
