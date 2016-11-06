#ifndef FUNCTIONCOMPILER_H
#define FUNCTIONCOMPILER_H

#include "expressionparser.h"


// Need:

// an expression output generator
//	- convert operators, functions to tokens in proper order
//	- all token values can be bytes, since runtime requires it


#include <QString>
#include <QTextStream>


class FunctionCompiler
{
public:
	FunctionCompiler();
	~FunctionCompiler();

	bool Compile( QByteArray& Source , QByteArray& InputOutputList , QString & OutputPathPrefix );

private:
	QString outPath;
	int funcIndex, instrIndex;

	ExpressionParser parser;

	QVector<EVar*> FreeTemp, UsedTemp;

	EVar *GetTempVar(void);
	void FreeIfTempVar(EVar *pTemp);

	bool Parse(void);

	bool GenerateOutputCode( void );

	bool GenerateVariables( void );
	bool GenerateTokens( void );

	bool OutputExpressionTokens( QTextStream & stream , Expression * expr );
	bool ComputeSubExpressions( QTextStream & stream , Expression * expr , EVar *pAssignTo, EVar ** outVar );

	bool GenerateInstruction( QTextStream & stream , Expression * op, EVar * pLeft, EVar * pRight, EVar * pOut );

	void UpdateVarScope( EVar * var , bool isAssign, int iFunc, int iInstr );
	void AssignVariableEnumIndices( void);

	bool RangesOverlap( RangeList & set1, RangeList & set2 );
	void MergeRanges( RangeList & set1, RangeList & set2 );
};


#endif // FUNCTIONCOMPILER_H
