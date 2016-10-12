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

	EVar *pLeftTemp, *pRightTemp;

	bool Parse(void);

	bool GenerateOutputCode( ExpressionParser & parser );

	bool GenerateVariables( ExpressionParser & parser );
	bool GenerateTokens( ExpressionParser & parser );

	bool OutputExpressionTokens( ExpressionParser & parser , QTextStream & stream , Expression * expr );
	bool ComputeSubExpressions( QTextStream & stream , Expression * expr , EVar *pUseTemp, EVar ** outVar );

	bool GenerateInstruction( QTextStream & stream , Expression * op, EVar * pLeft, EVar * pRight, EVar * pOut );

	void UpdateVarScope( EVar * var , bool isAssign, int iFunc, int iInstr );
	void AssignVariableEnumIndices( ExpressionParser & parser );

	bool RangesOverlap( RangeList & set1, RangeList & set2 );
	void MergeRanges( RangeList & set1, RangeList & set2 );
};


#endif // FUNCTIONCOMPILER_H
