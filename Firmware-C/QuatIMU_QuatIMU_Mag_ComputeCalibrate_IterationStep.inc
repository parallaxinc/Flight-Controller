	//Compute denominator:
	// float aA = Q2 + 16.0f * (A2 - 2.0f*A*Xn + Xn2);
	F32_opMul,     const_F2_0, A, Temp_lhs,
	F32_opMul,     Temp_lhs, Xn, Temp_rhs,
	F32_opSub,     A2, Temp_rhs, Temp_lhs,
	F32_opAdd,     Temp_lhs, Xn2, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I4, Temp_rhs,
	F32_opAdd,     Q2, Temp_rhs, aA,
	// float aB = Q2 + 16.0f * (B2 - 2.0f*B*Yn + Yn2);
	F32_opMul,     const_F2_0, B, Temp_lhs,
	F32_opMul,     Temp_lhs, Yn, Temp_rhs,
	F32_opSub,     B2, Temp_rhs, Temp_lhs,
	F32_opAdd,     Temp_lhs, Yn2, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I4, Temp_rhs,
	F32_opAdd,     Q2, Temp_rhs, aB,
	// float aC = Q2 + 16.0f * (C2 - 2.0f*C*Zn + Zn2);
	F32_opMul,     const_F2_0, C, Temp_lhs,
	F32_opMul,     Temp_lhs, Zn, Temp_rhs,
	F32_opSub,     C2, Temp_rhs, Temp_lhs,
	F32_opAdd,     Temp_lhs, Zn2, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I4, Temp_rhs,
	F32_opAdd,     Q2, Temp_rhs, aC,
	// 
	// aA = CMov(aA, 1.0f);	// aA = (aA == 0) ? 1.0f : aA;
	F32_opCMov,    aA, const_F1_0, aA,
	// aB = CMov(aB, 1.0f);	// aB = (aB == 0) ? 1.0f : aB;
	F32_opCMov,    aB, const_F1_0, aB,
	// aC = CMov(aC, 1.0f);	// aC = (aC == 0) ? 1.0f : aC;
	F32_opCMov,    aC, const_F1_0, aC,
	// 
	//Compute next iteration
	// float nA = A - ((F2 + 16.0f*( B*XY + C*XZ + Xn*(-A2 - Q0) + A*(Xn2 + Q1 - C*Zn - B*Yn) ) )/aA);
	F32_opMul,     B, XY, Temp_lhs,
	F32_opMul,     C, XZ, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opNeg,     A2, 0, Temp_lhs,
	F32_opSub,     Temp_lhs, Q0, Temp_rhs,
	F32_opMul,     Xn, Temp_rhs, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opAdd,     Xn2, Q1, Temp_lhs,
	F32_opMul,     C, Zn, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     B, Yn, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_rhs,
	F32_opMul,     A, Temp_rhs, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I4, Temp_rhs,
	F32_opAdd,     F2, Temp_rhs, Temp_lhs,
	F32_opDiv,     Temp_lhs, aA, Temp_rhs,
	F32_opSub,     A, Temp_rhs, nA,
	// float nB = B - ((F3 + 16.0f*( A*XY + C*YZ + Yn*(-B2 - Q0) + B*(Yn2 + Q1 - A*Xn - C*Zn) ) )/aB);
	F32_opMul,     A, XY, Temp_lhs,
	F32_opMul,     C, YZ, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opNeg,     B2, 0, Temp_lhs,
	F32_opSub,     Temp_lhs, Q0, Temp_rhs,
	F32_opMul,     Yn, Temp_rhs, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opAdd,     Yn2, Q1, Temp_lhs,
	F32_opMul,     A, Xn, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     C, Zn, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_rhs,
	F32_opMul,     B, Temp_rhs, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I4, Temp_rhs,
	F32_opAdd,     F3, Temp_rhs, Temp_lhs,
	F32_opDiv,     Temp_lhs, aB, Temp_rhs,
	F32_opSub,     B, Temp_rhs, nB,
	// float nC = C - ((F4 + 16.0f*( A*XZ + B*YZ + Zn*(-C2 - Q0) + C*(Zn2 + Q1 - A*Xn - B*Yn) ) )/aC);
	F32_opMul,     A, XZ, Temp_lhs,
	F32_opMul,     B, YZ, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opNeg,     C2, 0, Temp_lhs,
	F32_opSub,     Temp_lhs, Q0, Temp_rhs,
	F32_opMul,     Zn, Temp_rhs, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opAdd,     Zn2, Q1, Temp_lhs,
	F32_opMul,     A, Xn, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     B, Yn, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_rhs,
	F32_opMul,     C, Temp_rhs, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I4, Temp_rhs,
	F32_opAdd,     F4, Temp_rhs, Temp_lhs,
	F32_opDiv,     Temp_lhs, aC, Temp_rhs,
	F32_opSub,     C, Temp_rhs, nC,
	// 
	//Check for stop condition
	//dA = (nA - A);
	//dB = (nB - B);
	//dC = (nC - C);
	// 
	//if( (dA*dA + dB*dB + dC*dC) <= Nstop ){
	//	break;
	//}
	// 
	//Compute next iteration's values
	// A = nA;
	F32_opMov,     nA, 0, A,
	// B = nB;
	F32_opMov,     nB, 0, B,
	// C = nC;
	F32_opMov,     nC, 0, C,
	// A2 = A*A;
	F32_opMul,     A, A, A2,
	// B2 = B*B;
	F32_opMul,     B, B, B2,
	// C2 = C*C;
	F32_opMul,     C, C, C2,
	// QS = A2 + B2 + C2;
	F32_opAdd,     A2, B2, Temp_lhs,
	F32_opAdd,     Temp_lhs, C2, QS,
	// QB = - 2.0f * (A*Xn + B*Yn + C*Zn);
	F32_opNeg,     const_F2_0, 0, Temp_lhs,
	F32_opMul,     A, Xn, Temp_lhs,
	F32_opMul,     B, Yn, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     C, Zn, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_rhs,
	F32_opMul,     Temp_lhs, Temp_rhs, QB,
	// Rsq = F0 + QB + QS;
	F32_opAdd,     F0, QB, Temp_lhs,
	F32_opAdd,     Temp_lhs, QS, Rsq,
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
