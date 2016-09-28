
#include "inputsandoutputs.h"
#include "floatfunctions.h"
#include "functionstream.h"

using namespace FloatStream;

int zx = 0, zy = 0, zz = 0;
int zeroCount = 16;


void QuatUpdate( SensorData & sens )
{
	if( zeroCount > 0 ) {
		zx += sens.GyroX;
		zy += sens.GyroY;
		zz += sens.GyroZ;

		zeroCount--;
		if( zeroCount == 0 ) {
			QuatIMU_InitVars();
			zx = (zx + 4) / 16;
			zy = (zy + 4) / 16;
			zz = (zz + 4) / 16;
		}
		return;
	}

	gx = sens.GyroX - zx;
	gy = sens.GyroY - zy;
	gz = sens.GyroZ - zz;
	ax = sens.AccelX;
	ay = sens.AccelY;
	az = sens.AccelZ;
	mx = sens.MagX;
	my = sens.MagY;
	mz = sens.MagZ;
	alt = 0;
	altRate = 0;


	//FUNCTION: QuatUpdate
	//--------------------------------------------------------------
	// Convert gyro rates to radians, add in the previous cycle error corrections
	//--------------------------------------------------------------
	float rx = Float(gx) * const_GyroScale    + errCorrX;
	float ry = Float(gz) * const_NegGyroScale + errCorrY;
	float rz = Float(gy) * const_NegGyroScale + errCorrZ;

	//--------------------------------------------------------------
	// Update the orientation quaternion
	//--------------------------------------------------------------
	float rmag = Sqrt(rx*rx + ry*ry + rz*rz + const_epsilon) * 0.5f;

	float sinr, cosr;
	cosr = SinCos(rmag, sinr);
	sinr /= rmag;

	float qdw = -(rx*qx + ry*qy + rz*qz) * 0.5f;
	float qdx =  (rx*qw + rz*qy - ry*qz) * 0.5f;
	float qdy =  (ry*qw - rz*qx + rx*qz) * 0.5f;
	float qdz =  (rz*qw + ry*qx - rx*qy) * 0.5f;

	qw = cosr * qw + sinr * qdw;
	qx = cosr * qx + sinr * qdx;
	qy = cosr * qy + sinr * qdy;
	qz = cosr * qz + sinr * qdz;

	rmag = Sqrt(qx*qx + qy*qy + qz*qz + qw*qw + const_epsilon);
	qw /= rmag;
	qx /= rmag;
	qy /= rmag;
	qz /= rmag;

	//--------------------------------------------------------------
	// Convert the updated quaternion to a rotation matrix
	//--------------------------------------------------------------
	float fx2 = qx*qx;
	float fy2 = qy*qy;
	float fz2 = qz*qz;

	float fwx = qw*qx;
	float fwy = qw*qy;
	float fwz = qw*qz;

	float fxy = qx*qy;
	float fxz = qx*qz;
	float fyz = qy*qz;

	m00 = 1.0f - (fy2 + fz2) * 2.0f;
	m01 =        (fxy - fwz) * 2.0f;
	m02 =        (fxz + fwy) * 2.0f;

	m10 =        (fxy + fwz) * 2.0f;
	m11 = 1.0f - (fx2 + fz2) * 2.0f;
	m12 =        (fyz - fwx) * 2.0f;

	m20 =        (fxz - fwy) * 2.0f;
	m21 =        (fyz + fwx) * 2.0f;
	m22 = 1.0f - (fx2 + fy2) * 2.0f;


	//--------------------------------------------------------------
	// Grab the accelerometer values as floats
	//--------------------------------------------------------------
	float fax = -Float(ax);
	float fay = Float(az);
	float faz = Float(ay);

	//--------------------------------------------------------------
	// Rotate accelerometer vector by the level correction angles
	//--------------------------------------------------------------
	float axRot = (fax * accRollCorrCos) - (fay * accRollCorrSin);
	float ayRot = (fax * accRollCorrSin) + (fay * accRollCorrCos);
	fax = axRot;
	fay = ayRot;

	axRot = (faz * accPitchCorrCos) - (fay * accPitchCorrSin);
	ayRot = (fax * accPitchCorrSin) + (fay * accPitchCorrCos);
	faz = axRot;
	fay = ayRot;

	//--------------------------------------------------------------
	// Compute length of the accelerometer vector and normalize it.
	// Use the computed length to decide weighting, IE how likely is
	// it a good reading to use to correct our rotation estimate.
	// If it's too long/short, weight it less.
	//--------------------------------------------------------------

	rmag = Sqrt( fax*fax + fay*fay + faz*faz + const_epsilon );
	float faxn = fax / rmag;
	float fayn = fay / rmag;
	float fazn = faz / rmag;

	float accWeight = 1.0f - FMin( FAbs( 2.0f - const_AccScale * rmag * 2.0f), 1.0f );


	//--------------------------------------------------------------
	// Compute the cross product of our normalized accelerometer vector
	// and our current orientation "up" vector.  If they're identical,
	// the cross product will be zeros.  Any difference produces an
	// axis of rotation between the two vectors, and the magnitude of
	// the vector is the amount to rotate to align them.
	//--------------------------------------------------------------

	float errDiffX = fayn * m12 - fazn * m11;
	float errDiffY = fazn * m10 - faxn * m12;
	float errDiffZ = faxn * m11 - fayn * m10;

	accWeight *= const_AccErrScale;

	//--------------------------------------------------------------
	// Scale the resulting difference by the weighting factor.  This
	// gets mixed in with the gyro values on the next update to pull
	// the "up" part of our rotation back into alignment with gravity
	// over time.
	//--------------------------------------------------------------

	errCorrX = errDiffX * accWeight;
	errCorrY = errDiffY * accWeight;
	errCorrZ = errDiffZ * accWeight;

	// compute heading using Atan2 and the Z vector of the orientation matrix
	FloatYaw = -ATan2(m20, m22);

	// When switching between manual and auto, or just lifting off, I need to
	// know the half-angle of the craft so I can use it as my initial Heading value
	// to be fed into the quaternion construction code.  This HalfYaw value serves that purpose

	HalfYaw = FloatYaw * 0.5f;


	// Compute pitch and roll in integer form, used by compass calibration, possible user code

	Pitch = Trunc( ASin(m12) * const_outAngleScale );
	Roll = Trunc( ASin(m10) * const_outNegAngleScale );

	// 1.0/m11 = scale factor for thrust - this will be infinite if perpendicular to ground
	ThrustFactor = Trunc( (1.0f / m11) * 256.0f );


	//--------------------------------------------------------------
	// Compute the running height estimate - this is a fusion of the
	// height computed directly from barometric pressure, and and
	// running estimate of vertical velocity computed from the
	// accelerometer, integrated to produce a height estimate.
	// The two different values are used to correct each other.
	//--------------------------------------------------------------

	//forceX = fax / 4096.0f;
	//forceY = fay / 4096.0f;
	//forceZ = faz / 4096.0f;

	//Orient force vector into world frame & subtract gravity (1G)
	forceWY = (m01*fax + m11*fay + m21*faz)/4096.0f - 1.0f;

	//Convert G to mm/sec^2
	forceWY *= const_G_mm_PerSec;

	velocityEstimate += forceWY * const_UpdateScale;

	velocityEstimate = (velocityEstimate * const_velAccScale) + (Float(altRate) * const_velAltiScale);
	altitudeEstimate += velocityEstimate * const_UpdateScale;

	altitudeEstimate = (altitudeEstimate * const_velAccTrust) + Float(alt) * const_velAltiTrust;

	// output integer values for PIDs
	AltitudeEstMM = Trunc(altitudeEstimate);
	VelocityEstMM = Trunc(velocityEstimate);

	//ENDFUNCTION
}

void UpdateControls_Manual(void)
{
	//FUNCTION: UpdateControls_Manual

	float rx = Float(In_Elev) * const_ManualBankScale;
	float rz = Float(In_Aile) * -const_ManualBankScale;
	float ry = Float(In_Rudd) * const_ManualYawScale;

	// QR = CQ * Quaternion(0,rx,ry,rz)

	// Expands to ( * qw zero terms removed):
	float qrx =             cqy * rz - cqz * ry + cqw * rx;
	float qry = -cqx * rz            + cqz * rx + cqw * ry;
	float qrz =  cqx * ry - cqy * rx            + cqw * rz;
	float qrw = -cqx * rx - cqy * ry - cqz * rz;


	// CQ = CQ + QR;
	cqw += qrw;
	cqx += qrx;
	cqy += qry;
	cqz += qrz;


	// CQ.Normalize();
	float rmag = Sqrt(cqx*cqx + cqy*cqy + cqz*cqz + cqw*cqw + const_epsilon);

	//cq /= rmag
	cqw /= rmag;
	cqx /= rmag;
	cqy /= rmag;
	cqz /= rmag;

	//ENDFUNCTION
}

void UpdateControls_AutoLevel(void)
{
	//FUNCTION: UpdateControls_Manual

	// Convert radio inputs to float, scale them to get them into the range we want
	float rx = Float(In_Elev) * const_AutoBankScale;
	float rz = Float(In_Aile) * -const_AutoBankScale;
	float ry = Float(In_Rudd) * const_YawRateScale;

	// Add scaled rudd to desired Heading
	Heading += ry;

	// Keep Heading in the range of 0 to TwoPI
	Heading -= Float(Trunc(Heading / const_TwoPI)) * const_TwoPI;

	// Compute sines and cosines of scaled control input values

	float csx, csy, csz, snx, sny, snz;
	csx = SinCos(rx, snx);
	csy = SinCos(ry, sny);
	csz = SinCos(rz, snz);

	// Pre-compute some re-used terms to save computation time
	float snycsx = sny * csx;
	float snysnx = sny * snx;
	float csycsz = csy * csz;
	float csysnz = csy * snz;

	// Compute the quaternion that represents our new desired orientation  ((CQ = Control Quaternion))
	cqx =  snycsx * snz + csycsz * snx;
	cqy =  snycsx * csz + csysnz * snx;
	cqz = csysnz * csx - snysnx * csz;
	cqw = csycsz * csx - snysnx * snz;

	//ENDFUNCTION
}


void UpdateControls_ComputeOrientationChange(void)
{
	//FUNCTION: UpdateControls_ComputeOrientationChange

	//---------------------------------------------------------------------------
	// Compute the quaternion which is the rotation from our current orientation (Q)
	// to our desired one (CQ)
	//
	// IE, QR = rotation from (Q) to (CQ)
	// Computation is QR = CQ.Conjugate() * Q,  where Conjugate is just (w, -x, -y, -z)
	//---------------------------------------------------------------------------

	// With all the appropriate sign flips, the formula becomes:

	float qrx = -cqx * qw - cqy * qz + cqz * qy + cqw * qx;
	float qry =  cqx * qz - cqy * qw - cqz * qx + cqw * qy;
	float qrz = -cqx * qy + cqy * qx - cqz * qw + cqw * qz;
	float qrw =  cqx * qx + cqy * qy + cqz * qz + cqw * qw;

	// qrw < 0?
	float temp = Cmp(qrw, 0);

	// Conditionally negate QR if QR.w < 0
	qrw = CNeg(qrw, temp);
	qrx = CNeg(qrx, temp);
	qry = CNeg(qry, temp);
	qrz = CNeg(qrz, temp);


	// float diffAngle = qrot.ToAngleAxis( out DiffAxis );

	// Converts to:

	// float diffAngle = 2.0f * Acos(qrw);
	// float rmag = Sqrt( 1.0f - qrw*qrw );	  // assuming quaternion normalised then w is less than 1, so term always positive.
	// rmag = min( rmag, 0.001 )
	// DiffAxis.x = qrx / rmag; // normalise axis
	// DiffAxis.y = qry / rmag;
	// DiffAxis.z = qrz / rmag;

	// PitchDiff = DiffAxis.x * diffAngle
	// RollDiff =  DiffAxis.z * diffAngle
	// YawDiff =   DiffAxis.y * diffAngle


	// float diffAngle = 2.0f * Acos(qrw);

	// clamp qrw to -1.0 to +1.0 range
	qrw = -FMin(qrw, 1.0f);
	qrw = -FMin(qrw, 1.0f);	// Yes, this executes twice, it looks funny

	// diffAngle = ACos(qrw) * 2.0
	float diffAngle = ACos(qrw) * 2.0f;

	DebugFloat = diffAngle;


	// float rmag = Sqrt( 1.0f - qrw*qrw );	  // assuming quaternion normalised then w is less than 1, so term always positive.
	temp = 1.0f - qrw*qrw;

	// make sure temp is >= 0.0 (don't have FMax, so negate, use FMin, negate again)
	temp = -FMin( 1.0f, -temp );
	float rmag = Sqrt(temp) + const_epsilon;


	// rmag = (1.0/rmag * diffAngle) * 4096  equivalent to rmag = (diffAngle / rmag)
	rmag = Shift(diffAngle / rmag, const_OutControlShift);

	// Simplified this a little by changing  X / rmag * diffAngle into X * (1.0/rmag * diffAngle)
	// PitchDiff = qrx / rmag * diffAngle
	// RollDiff =  qry / rmag * diffAngle
	// YawDiff =   qrz / rmag * diffAngle

	PitchDiff = Trunc(qrx * rmag);
	RollDiff =  Trunc(qrz * rmag);
	YawDiff =   Trunc(qry * rmag);

	//ENDFUNCTION
}


#define Const_UpdateRate  (200/8)
#define Const_OneG  4096					//Must match the scale of the accelerometer
#define Const_Alti_UpdateRate  25		    //Must match the update rate of the device

#define RadToDeg (180.0 / 3.141592654)                         //Degrees per Radian
#define GyroToDeg  (1000.0 / 70.0)                             //Gyro units per degree @ 2000 deg/sec sens = 70 mdps/bit
#define AccToG  (float)(Const_OneG)                            //Accelerometer per G @ 8g sensitivity = ~0.24414 mg/bit
#define GyroScale  (GyroToDeg * RadToDeg * (float)Const_UpdateRate)

#define PI  3.141592654


void QuatIMU_InitVars(void)
{
	qx = 0.0f;
	qy = 0.0f;
	qz = 0.0f;
	qw = 1.0f;

	accRollCorrSin = Sin(-1.5f / RadToDeg);                       // used to correct the accelerometer vector angle offset
	accRollCorrCos = Cos(-1.5f / RadToDeg);
	accPitchCorrSin = 0.0f;
	accPitchCorrCos = 1.0f;

	//Various constants used by the float math engine - Every command in the instruction stream reads two
	//arguments from memory using memory addresses, so the values actually need to exist somewhere

	const_GyroScale         =    1.0f / (float)GyroScale;
	const_NegGyroScale      =   -1.0f / (float)GyroScale;

	errCorrX = errCorrY = errCorrZ = 0.0f;

	const_epsilon           =    0.00000001f;     //Added to vector length value before inverting (1/X) to insure no divide-by-zero problems


	// Multiplied by 8 to account for lower sensor update rate on the PC
	const_AccErrScale       =    1.0f/512.0f * 8.f;  //How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)
	const_MagErrScale       =    1.0f/512.0f * 8.f;  //How much accelerometer to fuse in each update (runs a little faster if it's a fractional power of two)

	const_AccScale          =    1.0f/(float)AccToG;//Conversion factor from accel units to G's
	const_G_mm_PerSec       =    9.80665f * 1000.0f;  // gravity in mm/sec^2
	const_UpdateScale       =    1.0f / (float)Const_UpdateRate;    //Convert units/sec to units/update

	const_velAccScale       =    0.9995f;     // was 0.9995     - Used to generate the vertical velocity estimate
	const_velAltiScale      =    0.0005f;     // was 0.0005

	const_velAccTrust       =    0.9993f;      // was 0.9990    - used to generate the absolute altitude estimate
	const_velAltiTrust      =    0.0007f;      // was 0.0010

	const_YawRateScale      =    ((120.0f / 200.0f) / 1024.0f) * (PI/180.f) * 0.5f; // 120 deg/sec / UpdateRate * Deg2Rad * HalfAngle
	const_AutoBankScale     =    (45.0f / 1024.0f) * (PI/180.0f) * 0.5f;

	const_ManualYawScale    =   ((180.0f / 200.0f) / 1024.0f) * (PI/180.f) * 0.5f; // 120 deg/sec / UpdateRate * Deg2Rad * HalfAngle
	const_ManualBankScale   =   ((120.0f / 200.0f) / 1024.0f) * (PI/180.f) * 0.5f; // 120 deg/sec / UpdateRate * Deg2Rad * HalfAngle

	const_TwoPI             =    2.0f * PI;

	const_outAngleScale     =    65536.0f / PI;
	const_outNegAngleScale  =   -65536.0f / PI;

	const_OutControlShift   =    12;
}



void Quat_GetQuaternion(QQuaternion & cq)
{
	cq.setX(qx);
	cq.setY(qy);
	cq.setZ(qz);
	cq.setScalar(qw);
}
