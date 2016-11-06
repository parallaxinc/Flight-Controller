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
	F32_opAdd,     Xn2, Yn2, Temp_0,
	F32_opAdd,     Temp_0, Zn2, F0,
	// F1 = 0.5f * F0;
	F32_opShift,   F0, const_INeg_1, F1,
	// F2 = -(Xn3 + Y2X + Z2X) * 8.0f;
	F32_opAdd,     Xn3, Y2X, Temp_0,
	F32_opAdd,     Temp_0, Z2X, Temp_0,
	F32_opNeg,     Temp_0, 0, Temp_0,
	F32_opShift,   Temp_0, const_I3, F2,
	// F3 = -(X2Y + Yn3 + Z2Y) * 8.0f;
	F32_opAdd,     X2Y, Yn3, Temp_0,
	F32_opAdd,     Temp_0, Z2Y, Temp_0,
	F32_opNeg,     Temp_0, 0, Temp_0,
	F32_opShift,   Temp_0, const_I3, F3,
	// F4 = -(X2Z + Y2Z + Zn3) * 8.0f;
	F32_opAdd,     X2Z, Y2Z, Temp_0,
	F32_opAdd,     Temp_0, Zn3, Temp_0,
	F32_opNeg,     Temp_0, 0, Temp_0,
	F32_opShift,   Temp_0, const_I3, F4,
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
	//A2 = A*A;
	//B2 = B*B;
	//C2 = C*C;
	//QS = A2 + B2 + C2;
	//QB = -(A*Xn + B*Yn + C*Zn) * 2.0f;
	// 
	//Set initial conditions:
	//Rsq = F0 + QB + QS;
	// 
	//First iteration computation:
	//Q0 = 0.5f * (QS - Rsq);
	//Q1 = F1 + Q0;
	//Q2 = 8.0f * ( QS - Rsq + QB + F0 );
	// 
	0,0,0,0
