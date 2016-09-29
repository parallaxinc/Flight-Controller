#ifndef EXPRESSION_H
#define EXPRESSION_H

#include "expressiontokenizer.h"


struct EVar
{
	QString	valString;
	QString constName;

	TOKEN	type;
	bool	isConst;
	bool	isPersistent;
	union {
		float	f;
		int		i;
	} Val;

	int		refCount;
	int     varIndex;	// Index into the orderedVarList array
};


struct Expression		// This can be a function call, with one or two arguments
{
	int line;
	TOKEN op;
	Expression * Right;
	Expression * Left;

	EVar * Value;
	QString FuncName;	// Only used if this is a function call

	Expression() : line(-1), Right(NULL), Left(NULL), Value(NULL) {;}
	~Expression();

	void SwapArguments(void);

	void Dump(void);
};


#endif
