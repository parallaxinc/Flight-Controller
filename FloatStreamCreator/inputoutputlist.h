
INPUT int gx, gy, gz;
INPUT int ax, ay, az;
INPUT int mx, my, mz;
INPUT int alt;
INPUT int altRate;

INPUT int In_Elev, In_Aile, In_Rudd;

INPUT float accRollCorrCos, accRollCorrSin;
INPUT float accPitchCorrCos, accPitchCorrSin;

INPUT float const_GyroScale;
INPUT float const_NegGyroScale;
INPUT float const_epsilon;

INPUT int   const_OutControlShift;
INPUT int   const_ThrustShift;

INPUT float const_AccErrScale;
INPUT float const_MagErrScale;
INPUT float const_AccScale;
INPUT float const_G_mm_PerSec;
INPUT float const_UpdateScale;

INPUT float const_velAccScale;
INPUT float const_velAltiScale;

INPUT float const_velAccTrust;
INPUT float const_velAltiTrust;

INPUT float const_YawRateScale;
INPUT float const_ManualYawScale;
INPUT float const_AutoBankScale;
INPUT float const_ManualBankScale;
INPUT float const_TwoPI;

INPUT float const_outAngleScale;
INPUT float const_outNegAngleScale;


PERSIST float altitudeEstimate, velocityEstimate;
PERSIST float errCorrX, errCorrY, errCorrZ;
PERSIST float cqx, cqy, cqz, cqw;
PERSIST float Heading;

//OUTPUT float forceX, forceY, forceZ;              // Current forces acting on craft, excluding gravity
//OUTPUT float forceWX, forceWY, forceWZ;           // Current forces acting on craft, excluding gravity, in world frame
OUTPUT float forceWY;           // Current forces acting on craft, excluding gravity, in world frame


OUTPUT int Roll;
OUTPUT int Pitch;
OUTPUT int Yaw;
OUTPUT int ThrustFactor;

OUTPUT float m00, m01, m02;
OUTPUT float m10, m11, m12;
OUTPUT float m20, m21, m22;

OUTPUT float qx, qy, qz, qw;
OUTPUT int	 VelocityEstMM;
OUTPUT int	 AltitudeEstMM;

OUTPUT float FloatYaw;                                    // Current heading (yaw) in floating point
OUTPUT float HalfYaw;                                     // Heading / 2, used for quaternion construction

OUTPUT int PitchDiff, RollDiff, YawDiff;
OUTPUT float DebugFloat;
