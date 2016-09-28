#ifndef EXPRESSIONTOKENIZER_H
#define EXPRESSIONTOKENIZER_H

#include <QByteArray>
#include <QString>


enum TOKEN
{
	T_Float,
	T_Int,
	T_Const,

	T_Label,
	T_Digit,

	T_OpenParen,
	T_CloseParen,

	T_End,		// ;
	T_Break,	// , - delimiter for sub-expressions, or func args

	T_Assign,

	T_Add,
	T_Sub,
	T_Mul,
	T_Div,

	T_AddAssign,
	T_SubAssign,
	T_MulAssign,
	T_DivAssign,

	T_Positive,
	T_Negative,

	T_Comment,
	T_None,

	T_DoneTokenizing,
	T_Function,		// Not used during tokenizing, just parsing
};

extern const char * TokenNames[];

inline static bool IsType( TOKEN t ) {
	return t == T_Float || t == T_Int;
}

inline static bool IsAssign( TOKEN t ) {
	return t == T_Assign || t == T_AddAssign || t == T_SubAssign || t == T_MulAssign || t == T_DivAssign;
}

inline static bool IsUnary( TOKEN t ) {
	return t == T_Positive || t == T_Negative;
}

inline static bool IsOperator( TOKEN t ) {
	return t == T_Add || t == T_Sub || t == T_Mul || t == T_Div;
}


class ExpressionTokenizer
{
public:
	ExpressionTokenizer();

	void SetSource( const char * Source );
	bool Advance(void);


	QString valueLast;
	QString value;
	QString valueNext;

	TOKEN	last;
	TOKEN   token;
	TOKEN	next;

	const char * pSrc;
};

#endif // EXPRESSIONTOKENIZER_H


// A + B  (label ADD label)
// A+B    (label ADD label)
// A+ -B   (label ADD (neg)label)
// A + +B (label ADD label)
