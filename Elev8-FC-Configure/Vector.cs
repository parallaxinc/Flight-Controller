
using System;

namespace Elev8
{
	public class Vector
	{
		public float[] v = new float[3];

		public Vector() {;}
		public Vector(float v0, float v1, float v2)
		{
			v[0] = v0;
			v[1] = v1;
			v[2] = v2;
		}

		public float x
		{
			get { return v[0]; }
			set { v[0] = value; }
		}

		public float y
		{
			get { return v[1]; }
			set { v[1] = value; }
		}

		public float z
		{
			get { return v[2]; }
			set { v[2] = value; }
		}

		public float Length
		{
			get {
				return (float)Math.Sqrt( x*x + y*y + z*z );
			}
		}

		public Vector Normalize()
		{
			float Len = Length;
			if( Len < 0.0001f ) Len = 0.0001f;
			float Scale = 1.0f / Len;

			return new Vector( x * Scale, y * Scale, z * Scale );
		}
	};
}
