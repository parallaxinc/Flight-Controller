	// Xn  /= compassPointCount;
	F32_opDiv,     Xn, compassPointCount, Xn,
	// Xn2 /= compassPointCount;
	F32_opDiv,     Xn2, compassPointCount, Xn2,
	// Xn3 /= compassPointCount;
	F32_opDiv,     Xn3, compassPointCount, Xn3,
	// Yn  /= compassPointCount;
	F32_opDiv,     Yn, compassPointCount, Yn,
	// Yn2 /= compassPointCount;
	F32_opDiv,     Yn2, compassPointCount, Yn2,
	// Yn3 /= compassPointCount;
	F32_opDiv,     Yn3, compassPointCount, Yn3,
	// Zn  /= compassPointCount;
	F32_opDiv,     Zn, compassPointCount, Zn,
	// Zn2 /= compassPointCount;
	F32_opDiv,     Zn2, compassPointCount, Zn2,
	// Zn3 /= compassPointCount;
	F32_opDiv,     Zn3, compassPointCount, Zn3,
	// 
	// XY  /= compassPointCount;
	F32_opDiv,     XY, compassPointCount, XY,
	// XZ  /= compassPointCount;
	F32_opDiv,     XZ, compassPointCount, XZ,
	// YZ  /= compassPointCount;
	F32_opDiv,     YZ, compassPointCount, YZ,
	// X2Y /= compassPointCount;
	F32_opDiv,     X2Y, compassPointCount, X2Y,
	// X2Z /= compassPointCount;
	F32_opDiv,     X2Z, compassPointCount, X2Z,
	// Y2X /= compassPointCount;
	F32_opDiv,     Y2X, compassPointCount, Y2X,
	// Y2Z /= compassPointCount;
	F32_opDiv,     Y2Z, compassPointCount, Y2Z,
	// Z2X /= compassPointCount;
	F32_opDiv,     Z2X, compassPointCount, Z2X,
	// Z2Y /= compassPointCount;
	F32_opDiv,     Z2Y, compassPointCount, Z2Y,
	// 
	//Reduction of multiplications
	// F0 = Xn2 + Yn2 + Zn2;
	F32_opAdd,     Xn2, Yn2, Temp_lhs,
	F32_opAdd,     Temp_lhs, Zn2, F0,
	// F1 = 0.5f * F0;
	F32_opShift,   F0, const_INeg_1, F1,
	// F2 = -(Xn3 + Y2X + Z2X) * 8.0f;
	F32_opAdd,     Xn3, Y2X, Temp_lhs,
	F32_opAdd,     Temp_lhs, Z2X, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I3, F2,
	// F3 = -(X2Y + Yn3 + Z2Y) * 8.0f;
	F32_opAdd,     X2Y, Yn3, Temp_lhs,
	F32_opAdd,     Temp_lhs, Z2Y, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I3, F3,
	// F4 = -(X2Z + Y2Z + Zn3) * 8.0f;
	F32_opAdd,     X2Z, Y2Z, Temp_lhs,
	F32_opAdd,     Temp_lhs, Zn3, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I3, F4,
	// 
	//Set initial conditions:
	// A = Xn;
	F32_opMov,     Xn, 0, A,
	// B = Yn;
	F32_opMov,     Yn, 0, B,
	// C = Zn;
	F32_opMov,     Zn, 0, C,
	// 
	//First iteration computation:
	// A2 = A*A;
	F32_opMul,     A, A, A2,
	// B2 = B*B;
	F32_opMul,     B, B, B2,
	// C2 = C*C;
	F32_opMul,     C, C, C2,
	// QS = A2 + B2 + C2;
	F32_opAdd,     A2, B2, Temp_lhs,
	F32_opAdd,     Temp_lhs, C2, QS,
	// QB = -(A*Xn + B*Yn + C*Zn) * 2.0f;
	F32_opMul,     A, Xn, Temp_lhs,
	F32_opMul,     B, Yn, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     C, Zn, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, QB,
	// 
	//Set initial conditions:
	// Rsq = F0 + QB + QS;
	F32_opAdd,     F0, QB, Temp_lhs,
	F32_opAdd,     Temp_lhs, QS, Rsq,
	// 
	//First iteration computation:
	// Q0 = 0.5f * (QS - Rsq);
	F32_opSub,     QS, Rsq, Temp_lhs,
	F32_opShift,   Temp_lhs, const_INeg_1, Q0,
	// Q1 = F1 + Q0;
	F32_opAdd,     F1, Q0, Q1,
	// Q2 = 8.0f * ( QS - Rsq + QB + F0 );
	F32_opSub,     QS, Rsq, Temp_lhs,
	F32_opAdd,     Temp_lhs, QB, Temp_lhs,
	F32_opAdd,     Temp_lhs, F0, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I3, Q2,
	// 
	0,0,0,0
