	// Convert radio inputs to float, scale them to get them into the range we want
	// float rx = Float(In_Elev) * const_AutoBankScale;
	F32_opFloat,   In_Elev, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_AutoBankScale, rx,
	// float rz = Float(In_Aile) * -const_AutoBankScale;
	F32_opFloat,   In_Aile, 0, Temp_lhs,
	F32_opNeg,     const_AutoBankScale, 0, Temp_rhs,
	F32_opMul,     Temp_lhs, Temp_rhs, rz,
	// float ry = Float(In_Rudd) * const_YawRateScale;
	F32_opFloat,   In_Rudd, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_YawRateScale, ry,
	// 
	// Add scaled rudd to desired Heading
	// Heading += ry;
	F32_opAdd,     Heading, ry, Heading,
	// 
	// Keep Heading in the range of 0 to TwoPI
	// Heading -= FloatTrunc(Heading / const_TwoPI) * const_TwoPI;
	F32_opDiv,     Heading, const_TwoPI, Temp_lhs,
	F32_opTruncRound,Temp_lhs, const_I2, Temp_lhs,
	F32_opMul,     Temp_lhs, const_TwoPI, Temp_rhs,
	F32_opSub,     Heading, Temp_rhs, Heading,
	// 
	// Compute sines and cosines of scaled control input values
	// 
	// float csx, csy, csz, snx, sny, snz;
	// csx = SinCos(rx, snx);
	F32_opSinCos,  rx, snx, csx,
	// csy = SinCos(Heading, sny);
	F32_opSinCos,  Heading, sny, csy,
	// csz = SinCos(rz, snz);
	F32_opSinCos,  rz, snz, csz,
	// 
	// Pre-compute some re-used terms to save computation time
	// float snycsx = sny * csx;
	F32_opMul,     sny, csx, snycsx,
	// float snysnx = sny * snx;
	F32_opMul,     sny, snx, snysnx,
	// float csycsz = csy * csz;
	F32_opMul,     csy, csz, csycsz,
	// float csysnz = csy * snz;
	F32_opMul,     csy, snz, csysnz,
	// 
	// Compute the quaternion that represents our new desired orientation  ((CQ = Control Quaternion))
	// cqx = snycsx * snz + csycsz * snx;
	F32_opMul,     snycsx, snz, Temp_lhs,
	F32_opMul,     csycsz, snx, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, cqx,
	// cqy = snycsx * csz + csysnz * snx;
	F32_opMul,     snycsx, csz, Temp_lhs,
	F32_opMul,     csysnz, snx, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, cqy,
	// cqz = csysnz * csx - snysnx * csz;
	F32_opMul,     csysnz, csx, Temp_lhs,
	F32_opMul,     snysnx, csz, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, cqz,
	// cqw = csycsz * csx - snysnx * snz;
	F32_opMul,     csycsz, csx, Temp_lhs,
	F32_opMul,     snysnx, snz, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, cqw,
	// 
	0,0,0,0
