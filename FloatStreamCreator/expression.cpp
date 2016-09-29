
#include "expression.h"
#include <QDebug>

Expression::~Expression()
{
	if( Left != NULL ) delete Left;
	if( Right != NULL ) delete Right;
}


const char * TokenOperator[] = {
	NULL, //T_Float,
	NULL, //T_Int,
	NULL, //T_Const,

	NULL, //T_Label,
	NULL, //T_Digit,

	NULL, //T_OpenParen,
	NULL, //T_CloseParen,

	NULL, //T_End,		// ;
	NULL, //T_Break,	// , - delimiter for sub-expressions, or func args

	"=", //T_Assign,

	"+", //T_Add,
	"-", //T_Sub,
	"*", //T_Mul,
	"/", //T_Div,

	"+=", //T_AddAssign,
	"-=", //T_SubAssign,
	"*=", //T_MulAssign,
	"/=", //T_DivAssign,

	"Pos", //T_Positive,
	"Neg", //T_Negative,

	NULL, //T_Comment,
	NULL, //T_None,

	NULL, //T_DoneTokenizing,
	NULL, //T_Function,		// Not used during tokenizing, just parsing
};



int _print_t(Expression *tree, int is_left, int offset, int depth, int & maxDepth, char s[20][255])
{
	char b[50];
	int width = 3;

	if (!tree) return 0;
	if( depth > maxDepth ) maxDepth = depth;

	if( tree->Value != NULL ) {
		QByteArray s = tree->Value->valString.toLatin1();
		sprintf(b, "(%s)", s.data());
	}
	else if( tree->op == T_Function ) {
		QByteArray s = tree->FuncName.toLatin1();
		sprintf(b, "(%s)", s.data());
	}
	else if( TokenOperator[tree->op] != NULL ) {
		sprintf(b, "(%s)", TokenOperator[tree->op] );
	}
	else {
		b[0] = 0;
	}

	width = strlen(b);

	int left  = _print_t(tree->Left,  1, offset,                depth + 1, maxDepth, s);
	int right = _print_t(tree->Right, 0, offset + left + width, depth + 1, maxDepth, s);

#ifdef COMPACT
	for (int i = 0; i < width; i++)
		s[depth][offset + left + i] = b[i];

	if (depth && is_left) {

		for (int i = 0; i < width + right; i++)
			s[depth - 1][offset + left + width/2 + i] = '-';

		s[depth - 1][offset + left + width/2] = '.';

	} else if (depth && !is_left) {

		for (int i = 0; i < left + width; i++)
			s[depth - 1][offset - width/2 + i] = '-';

		s[depth - 1][offset + left + width/2] = '.';
	}
#else
	for (int i = 0; i < width; i++)
		s[2 * depth][offset + left + i] = b[i];

	if (depth && is_left) {

		for (int i = 0; i < width + right; i++)
			s[2 * depth - 1][offset + left + width/2 + i] = '-';

		s[2 * depth - 1][offset + left + width/2] = '+';
		s[2 * depth - 1][offset + left + width + right + width/2] = '+';

	} else if (depth && !is_left) {

		for (int i = 0; i < left + width; i++)
			s[2 * depth - 1][offset - width/2 + i] = '-';

		s[2 * depth - 1][offset + left + width/2] = '+';
		s[2 * depth - 1][offset - width/2 - 1] = '+';
	}
#endif

	return left + width + right;
}

void print_t(Expression *tree)
{
	char s[20][255];
	for (int i = 0; i < 20; i++)
		sprintf(s[i], "%180s", " ");

	int maxDepth = 0;
	_print_t(tree, 0, 0, 0, maxDepth, s);

	for (int i = 0; i <= maxDepth*2; i++) {
		qDebug() << s[i];
	}
	qDebug() << "";
}

static bool DoDump = false;
void Expression::Dump(void)
{
	if( DoDump == false ) return;
	print_t( this );
}

void Expression::SwapArguments(void)
{
	// Exchange the left / right arguments - used in some optimizations
	Expression * pTemp = Left;
	Left = Right;
	Right = pTemp;
}
