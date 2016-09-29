	// float rx = Float(In_Elev) * const_ManualBankScale;
	F32_opFloat,   In_Elev, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_ManualBankScale, rx,
	// float rz = Float(In_Aile) * -const_ManualBankScale;
	F32_opFloat,   In_Aile, 0, Temp_lhs,
	F32_opNeg,     const_ManualBankScale, 0, Temp_rhs,
	F32_opMul,     Temp_lhs, Temp_rhs, rz,
	// float ry = Float(In_Rudd) * const_ManualYawScale;
	F32_opFloat,   In_Rudd, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_ManualYawScale, ry,
	// 
	// QR = CQ * Quaternion(0,rx,ry,rz)
	// 
	// Expands to ( * qw zero terms removed):
	// float qrx =               (cqy * rz) - (cqz * ry) + (cqw * rx);
	F32_opMul,     cqy, rz, Temp_lhs,
	F32_opMul,     cqz, ry, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqw, rx, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qrx,
	// float qry = -(cqx * rz)              + (cqz * rx) + (cqw * ry);
	F32_opMul,     cqz, rx, Temp_lhs,
	F32_opMul,     cqx, rz, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqw, ry, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qry,
	// float qrz =  (cqx * ry) - (cqy * rx)              + (cqw * rz);
	F32_opMul,     cqx, ry, Temp_lhs,
	F32_opMul,     cqy, rx, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqw, rz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qrz,
	// float qrw = -(cqx * rx) - (cqy * ry) - (cqz * rz);
	F32_opMul,     cqx, rx, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, Temp_lhs,
	F32_opMul,     cqy, ry, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqz, rz, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, qrw,
	// 
	// 
	// CQ = CQ + QR;
	// cqw += qrw;
	F32_opAdd,     cqw, qrw, cqw,
	// cqx += qrx;
	F32_opAdd,     cqx, qrx, cqx,
	// cqy += qry;
	F32_opAdd,     cqy, qry, cqy,
	// cqz += qrz;
	F32_opAdd,     cqz, qrz, cqz,
	// 
	// 
	// CQ.Normalize();
	// float rmag = Sqrt(cqx*cqx + cqy*cqy + cqz*cqz + cqw*cqw + const_epsilon);
	F32_opMul,     cqx, cqx, Temp_lhs,
	F32_opMul,     cqy, cqy, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqz, cqz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqw, cqw, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opAdd,     Temp_lhs, const_epsilon, Temp_lhs,
	F32_opSqrt,    Temp_lhs, 0, rmag,
	// 
	//cq /= rmag
	// cqw /= rmag;
	F32_opDiv,     cqw, rmag, cqw,
	// cqx /= rmag;
	F32_opDiv,     cqx, rmag, cqx,
	// cqy /= rmag;
	F32_opDiv,     cqy, rmag, cqy,
	// cqz /= rmag;
	F32_opDiv,     cqz, rmag, cqz,
	// 
	0,0,0,0
