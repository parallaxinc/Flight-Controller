	// float fmx = Float(mx);
	F32_opFloat,   mx, 0, fmx,
	// float fmy = Float(my);
	F32_opFloat,   my, 0, fmy,
	// float fmz = Float(mz);
	F32_opFloat,   mz, 0, fmz,
	// 
	// XY    += fmx * fmy;
	F32_opMul,     fmx, fmy, Temp_0,
	F32_opAdd,     XY, Temp_0, XY,
	// XZ    += fmx * fmz;
	F32_opMul,     fmx, fmz, Temp_0,
	F32_opAdd,     XZ, Temp_0, XZ,
	// YZ    += fmy * fmz;
	F32_opMul,     fmy, fmz, Temp_0,
	F32_opAdd,     YZ, Temp_0, YZ,
	// 
	// float fmx2 = fmx * fmx;
	F32_opMul,     fmx, fmx, fmx2,
	// Xn    += fmx;
	F32_opAdd,     Xn, fmx, Xn,
	// Xn2   += fmx2;
	F32_opAdd,     Xn2, fmx2, Xn2,
	// Xn3   += fmx2 * fmx;
	F32_opMul,     fmx2, fmx, Temp_0,
	F32_opAdd,     Xn3, Temp_0, Xn3,
	// X2Y   += fmx2 * fmy;
	F32_opMul,     fmx2, fmy, Temp_0,
	F32_opAdd,     X2Y, Temp_0, X2Y,
	// X2Z   += fmx2 * fmz;
	F32_opMul,     fmx2, fmz, Temp_0,
	F32_opAdd,     X2Z, Temp_0, X2Z,
	// 
	// float fmy2 = fmy * fmy;
	F32_opMul,     fmy, fmy, fmy2,
	// Yn    += fmy;
	F32_opAdd,     Yn, fmy, Yn,
	// Yn2   += fmy2;
	F32_opAdd,     Yn2, fmy2, Yn2,
	// Yn3   += fmy2 * fmy;
	F32_opMul,     fmy2, fmy, Temp_0,
	F32_opAdd,     Yn3, Temp_0, Yn3,
	// Y2X   += fmy2 * fmx;
	F32_opMul,     fmy2, fmx, Temp_0,
	F32_opAdd,     Y2X, Temp_0, Y2X,
	// Y2Z   += fmy2 * fmz;
	F32_opMul,     fmy2, fmz, Temp_0,
	F32_opAdd,     Y2Z, Temp_0, Y2Z,
	// 
	// float fmz2 = fmz * fmz;
	F32_opMul,     fmz, fmz, fmz2,
	// Zn    += fmz;
	F32_opAdd,     Zn, fmz, Zn,
	// Zn2   += fmz2;
	F32_opAdd,     Zn2, fmz2, Zn2,
	// Zn3   += fmz2 * fmz;
	F32_opMul,     fmz2, fmz, Temp_0,
	F32_opAdd,     Zn3, Temp_0, Zn3,
	// Z2X   += fmz2 * fmx;
	F32_opMul,     fmz2, fmx, Temp_0,
	F32_opAdd,     Z2X, Temp_0, Z2X,
	// Z2Y   += fmz2 * fmy;
	F32_opMul,     fmz2, fmy, Temp_0,
	F32_opAdd,     Z2Y, Temp_0, Z2Y,
	// 
	// compassPointCount += 1.0f;
	F32_opAdd,     compassPointCount, const_F1_0, compassPointCount,
	0,0,0,0
