#include "expressiontokenizer.h"
#include <QDebug>

ExpressionTokenizer::ExpressionTokenizer()
{
	last = token = next = T_None;
}

static bool DoDump = false;


void ExpressionTokenizer::SetSource( const char * Source )
{
	pSrc = Source;
	last = token = next = T_None;
	valueLast.clear();
	value.clear();
	valueNext.clear();
}


static bool HasArgument( TOKEN t ) {
	return t == T_Label || t == T_Digit || t == T_Comment;
}

const char * TokenNames[] = {
	"T_Float",		"T_Int",	"T_Const",
	"T_Label",		"T_Digit",
	"T_OpenParen",	"T_CloseParen",
	"T_End",		"T_Break",

	"T_Assign",
	"T_Add",		"T_Sub",
	"T_Mul",		"T_Div",
	"T_AddAssign",	"T_SubAssign",
	"T_MulAssign",	"T_DivAssign",
	"T_Positive",	"T_Negative",
	"T_Comment",	"T_None",
};


static void DumpToken(TOKEN t, QString & s)
{
	if( DoDump == false ) return;

	if( t == T_None ) return;	// ignore these
	QString out = TokenNames[t];
	if( HasArgument(t)) {
		out.append( " " );
		out.append( s );
	}
	qDebug().noquote() << out;
}


// CURRENT token is always "token"
// last and next are previous and upcoming tokens, available for context when parsing

bool ExpressionTokenizer::Advance(void)
{
	valueLast = value;
	value = valueNext;
	valueNext.clear();

	last = token;
	token = next;
	next = T_DoneTokenizing;	// we'll replace this if not finished yet...

	DumpToken( token, value );

	while( *pSrc != 0 )
	{
		if( isspace(*pSrc) ) {
			// eat whitespace
			pSrc++;
			continue;
		}

		if( pSrc[0] == '/' && pSrc[1] == '/' ) {
			// entire line is a comment
			while( *pSrc && *pSrc != '\n' && *pSrc != '\r' ) {
				valueNext += *pSrc++;
			}
			next = T_Comment;
			return true;
		}

		if( isalpha(*pSrc) || *pSrc == '_' ) {
			// Build a function, token, or label
			while( isalnum(*pSrc) || *pSrc == '_' ) {
				valueNext += *pSrc++;
			}

			if( valueNext == "float" ) {
				next = T_Float;
			}
			else if( valueNext == "int" ) {
				next = T_Int;
			}
			else if( valueNext == "const" ) {
				next = T_Const;
			}
			else {
				next = T_Label;
			}
			return true;
		}

		if( isdigit(*pSrc) ) {
			while( isdigit(*pSrc) || *pSrc == '.' ) {
				valueNext += *pSrc++;
			}
			if( *pSrc == 'f' ) {	// eat the 'f' suffix on float numbers
				pSrc++;
			}
			next = T_Digit;
			return true;
		}

		if( *pSrc == '+' || *pSrc == '-' ) {
			if( token == T_Label || token == T_Digit || token == T_None ||
				token == T_CloseParen ) {

				// this is an operator, not a sign
				if( pSrc[0] == '+' && pSrc[1] == '=' ) {
					next = T_AddAssign;
					pSrc++;
				}
				else if( pSrc[0] == '-' && pSrc[1] == '=' ) {
					next = T_SubAssign;
					pSrc++;
				}
				else if( *pSrc == '+' ) {
					next = T_Add;
				}
				else {
					next = T_Sub;
				}
				pSrc++;
				return true;
			}
			else if( IsOperator(token) || IsAssign(token) || token == T_OpenParen || token == T_Break || token == T_None )
			{
				// This is a unary positive/negative

				if( *pSrc == '+' ) {
					next = T_Positive;
				}
				else {
					next = T_Negative;
				}
				pSrc++;
				return true;
			}
		}

		if( *pSrc == '*' ) {
			if( pSrc[1] == '=' ) {
				next = T_MulAssign;
				pSrc++;
			}
			else {
				next = T_Mul;
			}
			pSrc++;
			return true;
		}

		if( *pSrc == '/' ) {
			if( pSrc[1] == '=' ) {
				next = T_DivAssign;
				pSrc++;
			}
			else {
				next = T_Div;
			}
			pSrc++;
			return true;
		}


		if( *pSrc == '(' ) {
			next = T_OpenParen;
			pSrc++;
			return true;
		}

		if( *pSrc == ')' ) {
			next = T_CloseParen;
			pSrc++;
			return true;
		}

		if( *pSrc == '=' ) {
			next = T_Assign;
			pSrc++;
			return true;
		}

		if( *pSrc == ';' ) {
			next = T_End;
			pSrc++;
			return true;
		}

		if( *pSrc == ',' ) {
			next = T_Break;
			pSrc++;
			return true;
		}

		pSrc++;	// ? This may mean bad source
	}

	return token != T_DoneTokenizing;
}

