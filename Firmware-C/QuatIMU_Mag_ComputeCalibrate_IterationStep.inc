	//Setup
	// float A2 = A*A;
	F32_opMul,     A, A, A2,
	// float B2 = B*B;
	F32_opMul,     B, B, B2,
	// float C2 = C*C;
	F32_opMul,     C, C, C2,
	// float QS = A2 + B2 + C2;
	F32_opAdd,     A2, B2, Temp_0,
	F32_opAdd,     Temp_0, C2, QS,
	// float QB = -(A*Xn + B*Yn + C*Zn) * 2.0f;
	F32_opMul,     A, Xn, Temp_0,
	F32_opMul,     B, Yn, Temp_1,
	F32_opAdd,     Temp_0, Temp_1, Temp_0,
	F32_opMul,     C, Zn, Temp_1,
	F32_opAdd,     Temp_0, Temp_1, Temp_0,
	F32_opNeg,     Temp_0, 0, Temp_0,
	F32_opShift,   Temp_0, const_I1, QB,
	// float Rsq = F0 + QB + QS;
	F32_opAdd,     F0, QB, Temp_0,
	F32_opAdd,     Temp_0, QS, Rsq,
	// float Q0 = 0.5f * (QS - Rsq);
	F32_opSub,     QS, Rsq, Temp_0,
	F32_opShift,   Temp_0, const_INeg_1, Q0,
	// float Q1 = F1 + Q0;
	F32_opAdd,     F1, Q0, Q1,
	// float Q2 = 8.0f * (QS - Rsq + QB + F0);
	F32_opSub,     QS, Rsq, Temp_0,
	F32_opAdd,     Temp_0, QB, Temp_0,
	F32_opAdd,     Temp_0, F0, Temp_0,
	F32_opShift,   Temp_0, const_I3, Q2,
	// 
	//Compute denominator:
	// float aA = Q2 + 16.0f * (A2 - A*Xn*2.0f + Xn2);
	F32_opMul,     A, Xn, Temp_0,
	F32_opMul,     Temp_0, const_F2_0, Temp_0,
	F32_opSub,     A2, Temp_0, Temp_0,
	F32_opAdd,     Temp_0, Xn2, Temp_0,
	F32_opShift,   Temp_0, const_I4, Temp_0,
	F32_opAdd,     Q2, Temp_0, aA,
	// float aB = Q2 + 16.0f * (B2 - B*Yn*2.0f + Yn2);
	F32_opMul,     B, Yn, Temp_0,
	F32_opMul,     Temp_0, const_F2_0, Temp_0,
	F32_opSub,     B2, Temp_0, Temp_0,
	F32_opAdd,     Temp_0, Yn2, Temp_0,
	F32_opShift,   Temp_0, const_I4, Temp_0,
	F32_opAdd,     Q2, Temp_0, aB,
	// float aC = Q2 + 16.0f * (C2 - C*Zn*2.0f + Zn2);
	F32_opMul,     C, Zn, Temp_0,
	F32_opMul,     Temp_0, const_F2_0, Temp_0,
	F32_opSub,     C2, Temp_0, Temp_0,
	F32_opAdd,     Temp_0, Zn2, Temp_0,
	F32_opShift,   Temp_0, const_I4, Temp_0,
	F32_opAdd,     Q2, Temp_0, aC,
	// 
	//aA = CMov(aA, 1.0f);	// aA = (aA == 0) ? 1.0f : aA;
	//aB = CMov(aB, 1.0f);	// aB = (aB == 0) ? 1.0f : aB;
	//aC = CMov(aC, 1.0f);	// aC = (aC == 0) ? 1.0f : aC;
	// 
	//Compute next iteration
	// float nA = A - ((F2 + 16.0f*( B*XY + C*XZ - Xn*(A2 + Q0) + A*(Xn2 + Q1 - C*Zn - B*Yn)))/aA);
	F32_opMul,     B, XY, Temp_0,
	F32_opMul,     C, XZ, Temp_1,
	F32_opAdd,     Temp_0, Temp_1, Temp_0,
	F32_opAdd,     A2, Q0, Temp_1,
	F32_opMul,     Xn, Temp_1, Temp_1,
	F32_opSub,     Temp_0, Temp_1, Temp_0,
	F32_opAdd,     Xn2, Q1, Temp_1,
	F32_opMul,     C, Zn, Temp_2,
	F32_opSub,     Temp_1, Temp_2, Temp_1,
	F32_opMul,     B, Yn, Temp_2,
	F32_opSub,     Temp_1, Temp_2, Temp_1,
	F32_opMul,     A, Temp_1, Temp_1,
	F32_opAdd,     Temp_0, Temp_1, Temp_0,
	F32_opShift,   Temp_0, const_I4, Temp_0,
	F32_opAdd,     F2, Temp_0, Temp_0,
	F32_opDiv,     Temp_0, aA, Temp_0,
	F32_opSub,     A, Temp_0, nA,
	// float nB = B - ((F3 + 16.0f*( A*XY + C*YZ - Yn*(B2 + Q0) + B*(Yn2 + Q1 - A*Xn - C*Zn)))/aB);
	F32_opMul,     A, XY, Temp_0,
	F32_opMul,     C, YZ, Temp_1,
	F32_opAdd,     Temp_0, Temp_1, Temp_0,
	F32_opAdd,     B2, Q0, Temp_1,
	F32_opMul,     Yn, Temp_1, Temp_1,
	F32_opSub,     Temp_0, Temp_1, Temp_0,
	F32_opAdd,     Yn2, Q1, Temp_1,
	F32_opMul,     A, Xn, Temp_2,
	F32_opSub,     Temp_1, Temp_2, Temp_1,
	F32_opMul,     C, Zn, Temp_2,
	F32_opSub,     Temp_1, Temp_2, Temp_1,
	F32_opMul,     B, Temp_1, Temp_1,
	F32_opAdd,     Temp_0, Temp_1, Temp_0,
	F32_opShift,   Temp_0, const_I4, Temp_0,
	F32_opAdd,     F3, Temp_0, Temp_0,
	F32_opDiv,     Temp_0, aB, Temp_0,
	F32_opSub,     B, Temp_0, nB,
	// float nC = C - ((F4 + 16.0f*( A*XZ + B*YZ - Zn*(C2 + Q0) + C*(Zn2 + Q1 - A*Xn - B*Yn)))/aC);
	F32_opMul,     A, XZ, Temp_0,
	F32_opMul,     B, YZ, Temp_1,
	F32_opAdd,     Temp_0, Temp_1, Temp_0,
	F32_opAdd,     C2, Q0, Temp_1,
	F32_opMul,     Zn, Temp_1, Temp_1,
	F32_opSub,     Temp_0, Temp_1, Temp_0,
	F32_opAdd,     Zn2, Q1, Temp_1,
	F32_opMul,     A, Xn, Temp_2,
	F32_opSub,     Temp_1, Temp_2, Temp_1,
	F32_opMul,     B, Yn, Temp_2,
	F32_opSub,     Temp_1, Temp_2, Temp_1,
	F32_opMul,     C, Temp_1, Temp_1,
	F32_opAdd,     Temp_0, Temp_1, Temp_0,
	F32_opShift,   Temp_0, const_I4, Temp_0,
	F32_opAdd,     F4, Temp_0, Temp_0,
	F32_opDiv,     Temp_0, aC, Temp_0,
	F32_opSub,     C, Temp_0, nC,
	// 
	//Set next iteration's start values
	// A = nA;
	F32_opMov,     nA, 0, A,
	// B = nB;
	F32_opMov,     nB, 0, B,
	// C = nC;
	F32_opMov,     nC, 0, C,
	// 
	// mx = Round(A);
	F32_opTruncRound,A, const_I1, mx,
	// my = Round(B);	// These are the output results
	F32_opTruncRound,B, const_I1, my,
	// mz = Round(C);
	F32_opTruncRound,C, const_I1, mz,
	// 
	0,0,0,0
