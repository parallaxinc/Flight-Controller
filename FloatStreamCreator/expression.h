#ifndef EXPRESSION_H
#define EXPRESSION_H

#include "expressiontokenizer.h"
#include <QVector>


struct ERange {
	short	Func;			// which function the range belongs to (index)
	short	First, Last;	// instruction indices (inclusive) where a variable is valid
};

typedef QVector<ERange> RangeList;


struct EVar
{
	EVar();

	QString	valString;
	QString constName;

	TOKEN	type;
	bool	isConst;
	bool	isPersistent;
	bool	isTemp;
	union {
		float	f;
		int		i;
	} Val;

	int		refCount;
	int     varIndex;			// Index into the orderedVarList array
	int		enumValue;			// Index in the final enum list (may share space with other values if their usage doesn't overlap)

	bool	alwaysValid;		// Keep this variable "in range" forever
	RangeList ranges;			// ranges where a variable is valid
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
