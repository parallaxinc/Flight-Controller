/*
  Elev8 GroundStation

  Copyright 2015 Parallax Inc

  This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-nc-sa/4.0/

  Written by Jason Dorie
*/

using System;

namespace Elev8
{
	public class Quaternion
	{
		public float w;
		public float x;
		public float y;
		public float z;

		public Quaternion() {
			w = 1;
			x = y = z = 0;
		}

		public Quaternion( float _w, float _x, float _y, float _z )
		{
			w = _w;
			x = _x;
			y = _y;
			z = _z;
		}

		public float Length()
		{
			float len = (float)Math.Sqrt( w * w + x * x + y * y + z * z );
			return len;
		}

		public Quaternion Normalize()
		{
			float len = Length();
			if(len > 0.000001f)
				len = 1.0f / len;

			return new Quaternion( w * len, x * len, y * len, z * len );
		}

		public float DotProduct( Quaternion q )
		{
			return w*q.w + x*q.x + y*q.y + z*q.z;
		}

		public Quaternion Conjugate()
		{
			return new Quaternion( w, -x, -y, -z );
		}

		public Quaternion Slerp( Quaternion To, float Tween )
		{
		float cosOmega , startWeight , endWeight , absCosOmega;
		float oneOverSineOmega;
		float omega , tweenAngle;

			// Recover the cosine of the angle between input quaternions.
			cosOmega = this.DotProduct( To );

			// If this cosine is negative , we want to interpolate to -end rather than end.
			absCosOmega = Math.Abs( cosOmega );

			// Compute the intepolation weights.
			if( absCosOmega <=  0.9998f ) {

				// The quaternions were far enough apart to use the real slerp operation.
				omega = (float)Math.Acos( absCosOmega );

				// Now determine the tween angle.
				tweenAngle = Tween * omega;

				// Lastly , calculate the weights.
				//#if 1
				oneOverSineOmega = (float)(1.0 / Math.Sin( omega ));
				//#else
				//oneOverSineOmega = reciprocal_sqrt( 1 - (cosOmega * cosOmega) );
				//#endif
				startWeight = (float)Math.Sin( omega - tweenAngle ) * oneOverSineOmega;
				endWeight = (float)Math.Sin( tweenAngle ) * oneOverSineOmega;

			} else {

				// The quaternions were _really_ close together (less than one degree); use a linear blend.
				// This avoids the roundoff-error-prone quotient of two very small numbers in the slerp ratios.
				startWeight = 1.0f - Tween;
				endWeight = Tween;
			}

			// If we need to negate , do so now.
			//__fsel( cosOmega, endWeight, -endWeight );
			endWeight = cosOmega >= 0 ? endWeight : -endWeight ;

			// Now blend the quaternions.
			Quaternion r = new Quaternion();
			r.w = startWeight * w + endWeight * To.w;
			r.x = startWeight * x + endWeight * To.x;
			r.y = startWeight * y + endWeight * To.y;
			r.z = startWeight * z + endWeight * To.z;

			return r;
		}

	};
}
