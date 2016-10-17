#include "floatfunctions.h"
#include <math.h>

float Float(int i) {
	return (float)i;
}

int Trunc(float f) {
	return (int)f;
}

int Round(float f) {
	return (int)roundf(f);
}

float FloatTrunc(float f) {
	return (float)(int)f;
}

float FloatRound(float f) {
	return roundf(f);
}


float Sqrt(float f) {
	return sqrtf(f);
}

int   Cmp(float a, float b) {
	if( a < b ) return -1;
	if( a > b ) return 1;
	return 0;
}

float Sin(float a) {
	return sinf(a);
}

float Cos(float a) {
	return cosf(a);
}

float Tan(float a) {
	return tanf(a);
}

float Log2(float a) {
	return log2f(a);
}

float Exp2(float a) {
	return exp2f(a);
}

float Pow(float a, float b) {
	return powf(a,b);
}

float ASin(float a) {
	if(a > 0.707106f) a = 0.707106f;
	if(a < -0.707106f) a = -0.707106f;
	return asinf(a);
}

float ACos(float a) {
	if(a > 0.707106f) a = 0.707106f;
	if(a < -0.707106f) a = -0.707106f;
	return acosf(a);
}

float ATan2(float a, float b) {
	return atan2f(a,b);
}

float Shift(float a, int b) {
	float factor = powf(2.0f, (float)b);
	return a * factor;
}


float Neg(float a) {
	return -a;
}

float SinCos(float a, float & outSin) {
	outSin = sinf(a);
	return cosf(a);
}

float FAbs(float a) {
	if( a < 0.0f ) return -a;
	return a;
}

float FMin(float a, float b) {
	if( a < b ) return a;
	return b;
}

float CMov(float a, float b) {
	if( a != 0.0f ) return a;
	else return b;
}

float CNeg(float a, float b) {
	if( b < 0 ) a = -a;
	return a;
}
