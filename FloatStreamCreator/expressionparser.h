#ifndef EXPRESSIONPARSER_H
#define EXPRESSIONPARSER_H

#include "expressiontokenizer.h"
#include "expression.h"
#include <QString>
#include <QMap>
#include <QStack>

struct ExprFunc {
	~ExprFunc();	// destroy expression list contents

	QString name;
	QVector<Expression *> exprList;		// completed expressions go here
};


class ExpressionParser
{
public:
	ExpressionParser();
	~ExpressionParser();

	ExpressionTokenizer tok;

	bool VarsPersist;

	TOKEN varType;
	bool isConst;

	QString labelName;
	int SourceLine;

	void StartFunction( QString & FuncName );
	bool Parse( const char * pSrc );

	Expression * Atom( void );
	EVar *       MakeVariable( TOKEN t, QString & s );
	Expression * MakeVariableExpression( TOKEN t, QString & s );
	Expression * GetExpression( void );
	Expression * SubExpression( Expression * lhs, int MinPrec );
	bool ParseArguments( Expression * funcCall );

	QMap<QString, EVar*> varList;
	QStack<Expression *> exprStack;		// expressions being built go here
	QVector<Expression *> * exprList;	// completed expressions go here

	QVector<ExprFunc *> funcList;
};

#endif // EXPRESSIONPARSER_H
