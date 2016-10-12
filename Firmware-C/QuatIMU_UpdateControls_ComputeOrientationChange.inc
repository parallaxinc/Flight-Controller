	//---------------------------------------------------------------------------
	// Compute the quaternion which is the rotation from our current orientation (Q)
	// to our desired one (CQ)
	//
	// IE, QR = rotation from (Q) to (CQ)
	// Computation is QR = CQ.Conjugate() * Q,  where Conjugate is just (w, -x, -y, -z)
	//---------------------------------------------------------------------------
	// 
	// With all the appropriate sign flips, the formula becomes:
	// 
	// float qrx = -(cqx * qw) - (cqy * qz) + (cqz * qy) + (cqw * qx);
	F32_opMul,     cqx, qw, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, Temp_lhs,
	F32_opMul,     cqy, qz, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqz, qy, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqw, qx, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qrx,
	// float qry =  (cqx * qz) - (cqy * qw) - (cqz * qx) + (cqw * qy);
	F32_opMul,     cqx, qz, Temp_lhs,
	F32_opMul,     cqy, qw, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqz, qx, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqw, qy, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qry,
	// float qrz = -(cqx * qy) + (cqy * qx) - (cqz * qw) + (cqw * qz);
	F32_opMul,     cqy, qx, Temp_lhs,
	F32_opMul,     cqx, qy, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqz, qw, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqw, qz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qrz,
	// float qrw =  (cqx * qx) + (cqy * qy) + (cqz * qz) + (cqw * qw);
	F32_opMul,     cqx, qx, Temp_lhs,
	F32_opMul,     cqy, qy, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqz, qz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     cqw, qw, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qrw,
	// 
	// qrw < 0?
	// float temp = Cmp(qrw, 0);
	F32_opCmp,     qrw, const_I0, temp,
	// 
	// Conditionally negate QR if QR.w < 0
	// qrw = CNeg(qrw, temp);
	F32_opCNeg,    qrw, temp, qrw,
	// qrx = CNeg(qrx, temp);
	F32_opCNeg,    qrx, temp, qrx,
	// qry = CNeg(qry, temp);
	F32_opCNeg,    qry, temp, qry,
	// qrz = CNeg(qrz, temp);
	F32_opCNeg,    qrz, temp, qrz,
	// 
	// 
	// float diffAngle = qrot.ToAngleAxis( out DiffAxis );
	// 
	// Converts to:
	// 
	// float diffAngle = 2.0f * Acos(qrw);
	// float rmag = Sqrt( 1.0f - qrw*qrw );	  // assuming quaternion normalised then w is less than 1, so term always positive.
	// rmag = min( rmag, 0.001 )
	// DiffAxis.x = qrx / rmag; // normalise axis
	// DiffAxis.y = qry / rmag;
	// DiffAxis.z = qrz / rmag;
	// 
	// PitchDiff = DiffAxis.x * diffAngle
	// RollDiff =  DiffAxis.z * diffAngle
	// YawDiff =   DiffAxis.y * diffAngle
	// 
	// 
	// float diffAngle = 2.0f * Acos(qrw);
	// 
	// clamp qrw to -1.0 to +1.0 range
	// qrw = -FMin(qrw, 1.0f);
	F32_opFMin,    qrw, const_F1_0, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, qrw,
	// qrw = -FMin(qrw, 1.0f);	// Yes, this executes twice, it looks funny
	F32_opFMin,    qrw, const_F1_0, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, qrw,
	// 
	// diffAngle = ACos(qrw) * 2.0
	// float diffAngle = ACos(qrw) * 2.0f;
	F32_opASinCos, qrw, 0, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, diffAngle,
	// 
	// DebugFloat = diffAngle;
	F32_opMov,     diffAngle, 0, DebugFloat,
	// 
	// 
	// float rmag = Sqrt( 1.0f - qrw*qrw );	  // assuming quaternion normalised then w is less than 1, so term always positive.
	// temp = 1.0f - qrw*qrw;
	F32_opMul,     qrw, qrw, Temp_rhs,
	F32_opSub,     const_F1_0, Temp_rhs, temp,
	// 
	// make sure temp is >= 0.0 (don't have FMax, so negate, use FMin, negate again)
	// temp = -FMin( 1.0f, -temp );
	F32_opNeg,     temp, 0, Temp_rhs,
	F32_opFMin,    const_F1_0, Temp_rhs, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, temp,
	// float rmag = Sqrt(temp) + const_epsilon;
	F32_opSqrt,    temp, 0, Temp_lhs,
	F32_opAdd,     Temp_lhs, const_epsilon, rmag,
	// 
	// 
	// rmag = (1.0/rmag * diffAngle) * 4096  equivalent to rmag = (diffAngle / rmag)
	// rmag = diffAngle / rmag * 4096.0f;
	F32_opDiv,     diffAngle, rmag, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I12, rmag,
	// 
	// Simplified this a little by changing  X / rmag * diffAngle into X * (1.0/rmag * diffAngle)
	// PitchDiff = qrx / rmag * diffAngle
	// RollDiff =  qry / rmag * diffAngle
	// YawDiff =   qrz / rmag * diffAngle
	// 
	// PitchDiff = Trunc(qrx * rmag);
	F32_opMul,     qrx, rmag, Temp_lhs,
	F32_opTruncRound,Temp_lhs, 0, PitchDiff,
	// RollDiff =  Trunc(qrz * rmag);
	F32_opMul,     qrz, rmag, Temp_lhs,
	F32_opTruncRound,Temp_lhs, 0, RollDiff,
	// YawDiff =   Trunc(qry * rmag);
	F32_opMul,     qry, rmag, Temp_lhs,
	F32_opTruncRound,Temp_lhs, 0, YawDiff,
	// 
	0,0,0,0
