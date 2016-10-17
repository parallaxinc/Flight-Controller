	//--------------------------------------------------------------
	// Convert gyro rates to radians, add in the previous cycle error corrections
	//--------------------------------------------------------------
	// float rx = Float(gx) * const_GyroScale    + errCorrX;
	F32_opFloat,   gx, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_GyroScale, Temp_lhs,
	F32_opAdd,     Temp_lhs, errCorrX, rx,
	// float ry = Float(gz) * const_NegGyroScale + errCorrY;
	F32_opFloat,   gz, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_NegGyroScale, Temp_lhs,
	F32_opAdd,     Temp_lhs, errCorrY, ry,
	// float rz = Float(gy) * const_NegGyroScale + errCorrZ;
	F32_opFloat,   gy, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_NegGyroScale, Temp_lhs,
	F32_opAdd,     Temp_lhs, errCorrZ, rz,
	// 
	//--------------------------------------------------------------
	// Update the orientation quaternion
	//--------------------------------------------------------------
	// float rmag = Sqrt(rx*rx + ry*ry + rz*rz + const_epsilon) * 0.5f;
	F32_opMul,     rx, rx, Temp_lhs,
	F32_opMul,     ry, ry, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     rz, rz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opAdd,     Temp_lhs, const_epsilon, Temp_lhs,
	F32_opSqrt,    Temp_lhs, 0, Temp_lhs,
	F32_opShift,   Temp_lhs, const_INeg_1, rmag,
	// 
	// float sinr, cosr;
	// cosr = SinCos(rmag, sinr);
	F32_opSinCos,  rmag, sinr, cosr,
	// sinr /= rmag;
	F32_opDiv,     sinr, rmag, sinr,
	// 
	// float qdw = -(rx*qx + ry*qy + rz*qz) * 0.5f;
	F32_opMul,     rx, qx, Temp_lhs,
	F32_opMul,     ry, qy, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     rz, qz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, Temp_lhs,
	F32_opShift,   Temp_lhs, const_INeg_1, qdw,
	// float qdx =  (rx*qw + rz*qy - ry*qz) * 0.5f;
	F32_opMul,     rx, qw, Temp_lhs,
	F32_opMul,     rz, qy, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     ry, qz, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opShift,   Temp_lhs, const_INeg_1, qdx,
	// float qdy =  (ry*qw - rz*qx + rx*qz) * 0.5f;
	F32_opMul,     ry, qw, Temp_lhs,
	F32_opMul,     rz, qx, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     rx, qz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opShift,   Temp_lhs, const_INeg_1, qdy,
	// float qdz =  (rz*qw + ry*qx - rx*qy) * 0.5f;
	F32_opMul,     rz, qw, Temp_lhs,
	F32_opMul,     ry, qx, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     rx, qy, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opShift,   Temp_lhs, const_INeg_1, qdz,
	// 
	// qw = cosr * qw + sinr * qdw;
	F32_opMul,     cosr, qw, Temp_lhs,
	F32_opMul,     sinr, qdw, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qw,
	// qx = cosr * qx + sinr * qdx;
	F32_opMul,     cosr, qx, Temp_lhs,
	F32_opMul,     sinr, qdx, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qx,
	// qy = cosr * qy + sinr * qdy;
	F32_opMul,     cosr, qy, Temp_lhs,
	F32_opMul,     sinr, qdy, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qy,
	// qz = cosr * qz + sinr * qdz;
	F32_opMul,     cosr, qz, Temp_lhs,
	F32_opMul,     sinr, qdz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, qz,
	// 
	// rmag = Sqrt(qx*qx + qy*qy + qz*qz + qw*qw + const_epsilon);
	F32_opMul,     qx, qx, Temp_lhs,
	F32_opMul,     qy, qy, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     qz, qz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     qw, qw, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opAdd,     Temp_lhs, const_epsilon, Temp_lhs,
	F32_opSqrt,    Temp_lhs, 0, rmag,
	// qw /= rmag;
	F32_opDiv,     qw, rmag, qw,
	// qx /= rmag;
	F32_opDiv,     qx, rmag, qx,
	// qy /= rmag;
	F32_opDiv,     qy, rmag, qy,
	// qz /= rmag;
	F32_opDiv,     qz, rmag, qz,
	// 
	//--------------------------------------------------------------
	// Convert the updated quaternion to a rotation matrix
	//--------------------------------------------------------------
	// float fx2 = qx*qx;
	F32_opMul,     qx, qx, fx2,
	// float fy2 = qy*qy;
	F32_opMul,     qy, qy, fy2,
	// float fz2 = qz*qz;
	F32_opMul,     qz, qz, fz2,
	// 
	// float fwx = qw*qx;
	F32_opMul,     qw, qx, fwx,
	// float fwy = qw*qy;
	F32_opMul,     qw, qy, fwy,
	// float fwz = qw*qz;
	F32_opMul,     qw, qz, fwz,
	// 
	// float fxy = qx*qy;
	F32_opMul,     qx, qy, fxy,
	// float fxz = qx*qz;
	F32_opMul,     qx, qz, fxz,
	// float fyz = qy*qz;
	F32_opMul,     qy, qz, fyz,
	// 
	// m00 = 1.0f - (fy2 + fz2) * 2.0f;
	F32_opAdd,     fy2, fz2, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, Temp_rhs,
	F32_opSub,     const_F1_0, Temp_rhs, m00,
	// m01 =        (fxy - fwz) * 2.0f;
	F32_opSub,     fxy, fwz, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, m01,
	// m02 =        (fxz + fwy) * 2.0f;
	F32_opAdd,     fxz, fwy, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, m02,
	// 
	// m10 =        (fxy + fwz) * 2.0f;
	F32_opAdd,     fxy, fwz, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, m10,
	// m11 = 1.0f - (fx2 + fz2) * 2.0f;
	F32_opAdd,     fx2, fz2, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, Temp_rhs,
	F32_opSub,     const_F1_0, Temp_rhs, m11,
	// m12 =        (fyz - fwx) * 2.0f;
	F32_opSub,     fyz, fwx, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, m12,
	// 
	// m20 =        (fxz - fwy) * 2.0f;
	F32_opSub,     fxz, fwy, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, m20,
	// m21 =        (fyz + fwx) * 2.0f;
	F32_opAdd,     fyz, fwx, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, m21,
	// m22 = 1.0f - (fx2 + fy2) * 2.0f;
	F32_opAdd,     fx2, fy2, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, Temp_rhs,
	F32_opSub,     const_F1_0, Temp_rhs, m22,
	// 
	// 
	//--------------------------------------------------------------
	// Grab the accelerometer values as floats
	//--------------------------------------------------------------
	// float fax = -Float(ax);
	F32_opFloat,   ax, 0, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, fax,
	// float fay = Float(az);
	F32_opFloat,   az, 0, fay,
	// float faz = Float(ay);
	F32_opFloat,   ay, 0, faz,
	// 
	//--------------------------------------------------------------
	// Rotate accelerometer vector by the level correction angles
	//--------------------------------------------------------------
	// float axRot = (fax * accRollCorrCos) - (fay * accRollCorrSin);
	F32_opMul,     fax, accRollCorrCos, Temp_lhs,
	F32_opMul,     fay, accRollCorrSin, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, axRot,
	// float ayRot = (fax * accRollCorrSin) + (fay * accRollCorrCos);
	F32_opMul,     fax, accRollCorrSin, Temp_lhs,
	F32_opMul,     fay, accRollCorrCos, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, ayRot,
	// fax = axRot;
	F32_opMov,     axRot, 0, fax,
	// fay = ayRot;
	F32_opMov,     ayRot, 0, fay,
	// 
	// axRot = (faz * accPitchCorrCos) - (fay * accPitchCorrSin);
	F32_opMul,     faz, accPitchCorrCos, Temp_lhs,
	F32_opMul,     fay, accPitchCorrSin, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, axRot,
	// ayRot = (faz * accPitchCorrSin) + (fay * accPitchCorrCos);
	F32_opMul,     faz, accPitchCorrSin, Temp_lhs,
	F32_opMul,     fay, accPitchCorrCos, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, ayRot,
	// faz = axRot;
	F32_opMov,     axRot, 0, faz,
	// fay = ayRot;
	F32_opMov,     ayRot, 0, fay,
	// 
	//--------------------------------------------------------------
	// Compute length of the accelerometer vector and normalize it.
	// Use the computed length to decide weighting, IE how likely is
	// it a good reading to use to correct our rotation estimate.
	// If it's too long/short, weight it less.
	//--------------------------------------------------------------
	// 
	// rmag = Sqrt( fax*fax + fay*fay + faz*faz + const_epsilon );
	F32_opMul,     fax, fax, Temp_lhs,
	F32_opMul,     fay, fay, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     faz, faz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opAdd,     Temp_lhs, const_epsilon, Temp_lhs,
	F32_opSqrt,    Temp_lhs, 0, rmag,
	// float faxn = fax / rmag;
	F32_opDiv,     fax, rmag, faxn,
	// float fayn = fay / rmag;
	F32_opDiv,     fay, rmag, fayn,
	// float fazn = faz / rmag;
	F32_opDiv,     faz, rmag, fazn,
	// 
	// TODO: investigate better weighting here, like (4.0 - vecLen * 4.0)
	// float accWeight = 1.0f - FMin( FAbs( 2.0f - const_AccScale * rmag * 2.0f), 1.0f );
	F32_opMul,     const_AccScale, rmag, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I1, Temp_rhs,
	F32_opSub,     const_F2_0, Temp_rhs, Temp_lhs,
	F32_opFAbs,    Temp_lhs, 0, Temp_lhs,
	F32_opFMin,    Temp_lhs, const_F1_0, Temp_rhs,
	F32_opSub,     const_F1_0, Temp_rhs, accWeight,
	// 
	// 
	//--------------------------------------------------------------
	// Compute the cross product of our normalized accelerometer vector
	// and our current orientation "up" vector.  If they're identical,
	// the cross product will be zeros.  Any difference produces an
	// axis of rotation between the two vectors, and the magnitude of
	// the vector is the amount to rotate to align them.
	//--------------------------------------------------------------
	// 
	// float errDiffX = fayn * m12 - fazn * m11;
	F32_opMul,     fayn, m12, Temp_lhs,
	F32_opMul,     fazn, m11, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, errDiffX,
	// float errDiffY = fazn * m10 - faxn * m12;
	F32_opMul,     fazn, m10, Temp_lhs,
	F32_opMul,     faxn, m12, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, errDiffY,
	// float errDiffZ = faxn * m11 - fayn * m10;
	F32_opMul,     faxn, m11, Temp_lhs,
	F32_opMul,     fayn, m10, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, errDiffZ,
	// 
	// accWeight *= const_AccErrScale;
	F32_opMul,     accWeight, const_AccErrScale, accWeight,
	// 
	//--------------------------------------------------------------
	// Scale the resulting difference by the weighting factor.  This
	// gets mixed in with the gyro values on the next update to pull
	// the "up" part of our rotation back into alignment with gravity
	// over time.
	//--------------------------------------------------------------
	// 
	// errCorrX = errDiffX * accWeight;
	F32_opMul,     errDiffX, accWeight, errCorrX,
	// errCorrY = errDiffY * accWeight;
	F32_opMul,     errDiffY, accWeight, errCorrY,
	// errCorrZ = errDiffZ * accWeight;
	F32_opMul,     errDiffZ, accWeight, errCorrZ,
	// 
	// compute heading using Atan2 and the Z vector of the orientation matrix
	// FloatYaw = -ATan2(m20, m22);
	F32_opATan2,   m20, m22, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, FloatYaw,
	// 
	// 
	// 
	// TESTING compass hold
	// 
	// float fmx = Float(mx);
	F32_opFloat,   mx, 0, fmx,
	// float fmy = Float(my);
	F32_opFloat,   my, 0, fmy,
	// float fmz = Float(mz);
	F32_opFloat,   mz, 0, fmz,
	// 
	// float cosPitch, sinPitch;
	// float cosRoll, sinRoll;
	// 
	// TODO: Need to handle >45 degree orientation, where ASin falls down, or cosPitch == 0, resulting in NaN for roll
	// 
	// float fpitch = ASin(CNeg(m10,m11));		// negate M10 if M11 is negative (ie, if we're inverted)
	F32_opCNeg,    m10, m11, Temp_lhs,
	F32_opASinCos, Temp_lhs, const_I1, fpitch,
	// cosPitch = SinCos(-fpitch, sinPitch);
	F32_opNeg,     fpitch, 0, Temp_lhs,
	F32_opSinCos,  Temp_lhs, sinPitch, cosPitch,
	// cosPitch = CMov(cosPitch, const_epsilon);	// avoid divide by 0
	F32_opCMov,    cosPitch, const_epsilon, cosPitch,
	// 
	// float froll = ASin(CNeg(m10,m11)/cosPitch);	// negate M12 if M11 is negative (ie, if we're inverted)
	F32_opCNeg,    m10, m11, Temp_lhs,
	F32_opDiv,     Temp_lhs, cosPitch, Temp_lhs,
	F32_opASinCos, Temp_lhs, const_I1, froll,
	// cosRoll = SinCos(froll, sinRoll);
	F32_opSinCos,  froll, sinRoll, cosRoll,
	// 
	// float fmxSinPitch = fmx * sinPitch;
	F32_opMul,     fmx, sinPitch, fmxSinPitch,
	// float fmzCosPitch = fmz * cosPitch;
	F32_opMul,     fmz, cosPitch, fmzCosPitch,
	// 
	// xh =  fmx * cosPitch + fmz * sinPitch;
	F32_opMul,     fmx, cosPitch, Temp_lhs,
	F32_opMul,     fmz, sinPitch, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, xh,
	// zh =  fmxSinPitch * sinRoll + fmy * cosRoll - fmzCosPitch * sinRoll;
	F32_opMul,     fmxSinPitch, sinRoll, Temp_lhs,
	F32_opMul,     fmy, cosRoll, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     fmzCosPitch, sinRoll, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, zh,
	// 
	// rmag = Sqrt( xh*xh + zh*zh + const_epsilon );
	F32_opMul,     xh, xh, Temp_lhs,
	F32_opMul,     zh, zh, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opAdd,     Temp_lhs, const_epsilon, Temp_lhs,
	F32_opSqrt,    Temp_lhs, 0, rmag,
	// 
	// This is the North vector in world space
	// xh /= rmag;
	F32_opDiv,     xh, rmag, xh,
	// zh /= rmag;
	F32_opDiv,     zh, rmag, zh,
	// 
	// If pitch/roll are outsite "reasonable" limits, zero magErrScale ?
	// 
	// float magErrScale = const_AccErrScale * 0.5f;		// Use the AccScale for now
	F32_opShift,   const_AccErrScale, const_INeg_1, magErrScale,
	// 
	// 
	// float temp = -FAbs(m11);	// Grab the vertical component of the vertical axis - If it's < 45 deg of horizontal we don't want mag correction
	F32_opFAbs,    m11, 0, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, temp,
	// temp = -FMin(temp, -0.6f);	// Clamp to 0.6 .. 1.0
	F32_opNeg,     const_F0_6, 0, Temp_rhs,
	F32_opFMin,    temp, Temp_rhs, Temp_lhs,
	F32_opNeg,     Temp_lhs, 0, temp,
	// temp -= 0.707f;				// Shift the range down
	F32_opSub,     temp, const_F0_707, temp,
	// temp *= 10.0f;				// 0.707f to 0.807f is now 0.0f to 1.0f  (0.707y is 45 deg from vertical.  < 0.8 we scale off - translates to ~37deg to 45deg)
	F32_opMul,     temp, const_F10_0, temp,
	// temp = FMin(temp, 1.0f);	// Clamp to 0 to 1
	F32_opFMin,    temp, const_F1_0, temp,
	// 
	// magErrScale *= temp;		// Scale the magErr correction by this
	F32_opMul,     magErrScale, temp, magErrScale,
	// 
	// Cross the mag heading vector with our orientation Z vector (zero mult terms removed)
	// 
	// errCorrX -=            (zh * m21) * magErrScale;
	F32_opMul,     zh, m21, Temp_lhs,
	F32_opMul,     Temp_lhs, magErrScale, Temp_rhs,
	F32_opSub,     errCorrX, Temp_rhs, errCorrX,
	// errCorrY += (zh * m20 - xh * m22) * magErrScale;
	F32_opMul,     zh, m20, Temp_lhs,
	F32_opMul,     xh, m22, Temp_rhs,
	F32_opSub,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     Temp_lhs, magErrScale, Temp_rhs,
	F32_opAdd,     errCorrY, Temp_rhs, errCorrY,
	// errCorrZ += (xh * m21)            * magErrScale;
	F32_opMul,     xh, m21, Temp_lhs,
	F32_opMul,     Temp_lhs, magErrScale, Temp_rhs,
	F32_opAdd,     errCorrZ, Temp_rhs, errCorrZ,
	// 
	// TESTING
	// 
	// 
	// 
	// When switching between manual and auto, or just lifting off, I need to
	// know the half-angle of the craft so I can use it as my initial Heading value
	// to be fed into the quaternion construction code.  This HalfYaw value serves that purpose
	// 
	// HalfYaw = FloatYaw * 0.5f;
	F32_opShift,   FloatYaw, const_INeg_1, HalfYaw,
	// 
	// 
	// Compute pitch and roll in integer form, used by compass calibration, possible user code
	// 
	// Pitch = Trunc( ASin(m12) * const_outAngleScale );
	F32_opASinCos, m12, const_I1, Temp_lhs,
	F32_opMul,     Temp_lhs, const_outAngleScale, Temp_lhs,
	F32_opTruncRound,Temp_lhs, const_I0, Pitch,
	// Roll = Trunc( fpitch * const_outNegAngleScale );
	F32_opMul,     fpitch, const_outNegAngleScale, Temp_lhs,
	F32_opTruncRound,Temp_lhs, const_I0, Roll,
	// 
	// 1.0/m11 = scale factor for thrust - this will be infinite if perpendicular to ground
	// ThrustFactor = Trunc( (1.0f / m11) * 256.0f );
	F32_opDiv,     const_F1_0, m11, Temp_lhs,
	F32_opShift,   Temp_lhs, const_I8, Temp_lhs,
	F32_opTruncRound,Temp_lhs, const_I0, ThrustFactor,
	// 
	// 
	//--------------------------------------------------------------
	// Compute the running height estimate - this is a fusion of the
	// height computed directly from barometric pressure, and and
	// running estimate of vertical velocity computed from the
	// accelerometer, integrated to produce a height estimate.
	// The two different values are used to correct each other.
	//--------------------------------------------------------------
	// 
	//Orient force vector into world frame & subtract gravity (1G)
	// forceWY = (m01*fax + m11*fay + m21*faz) / 4096.0f - 1.0f;
	F32_opMul,     m01, fax, Temp_lhs,
	F32_opMul,     m11, fay, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opMul,     m21, faz, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, Temp_lhs,
	F32_opShift,   Temp_lhs, const_INeg_12, Temp_lhs,
	F32_opSub,     Temp_lhs, const_F1_0, forceWY,
	// 
	//Convert G to mm/sec^2
	// forceWY *= const_G_mm_PerSec;
	F32_opMul,     forceWY, const_G_mm_PerSec, forceWY,
	// 
	// velocityEstimate += forceWY * const_UpdateScale;
	F32_opMul,     forceWY, const_UpdateScale, Temp_rhs,
	F32_opAdd,     velocityEstimate, Temp_rhs, velocityEstimate,
	// 
	// velocityEstimate = (velocityEstimate * const_velAccScale) + (Float(altRate) * const_velAltiScale);
	F32_opMul,     velocityEstimate, const_velAccScale, Temp_lhs,
	F32_opFloat,   altRate, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_velAltiScale, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, velocityEstimate,
	// altitudeEstimate += velocityEstimate * const_UpdateScale;
	F32_opMul,     velocityEstimate, const_UpdateScale, Temp_rhs,
	F32_opAdd,     altitudeEstimate, Temp_rhs, altitudeEstimate,
	// 
	// altitudeEstimate = (altitudeEstimate * const_velAccTrust) + Float(alt) * const_velAltiTrust;
	F32_opMul,     altitudeEstimate, const_velAccTrust, Temp_lhs,
	F32_opFloat,   alt, 0, Temp_lhs,
	F32_opMul,     Temp_lhs, const_velAltiTrust, Temp_rhs,
	F32_opAdd,     Temp_lhs, Temp_rhs, altitudeEstimate,
	// 
	// output integer values for PIDs
	// AltitudeEstMM = Trunc(altitudeEstimate);
	F32_opTruncRound,altitudeEstimate, const_I0, AltitudeEstMM,
	// VelocityEstMM = Trunc(velocityEstimate);
	F32_opTruncRound,velocityEstimate, const_I0, VelocityEstMM,
	// 
	0,0,0,0
