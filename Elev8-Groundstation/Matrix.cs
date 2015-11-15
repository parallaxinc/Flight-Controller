
using System;


namespace Elev8
{
	public class Matrix
	{
		public float[,] m = new float[3, 3];

		public Matrix()
		{
			m[0, 0] = 1;
			m[0, 1] = 0;
			m[0, 2] = 0;

			m[1, 0] = 0;
			m[1, 1] = 1;
			m[1, 2] = 0;

			m[2, 0] = 0;
			m[2, 1] = 0;
			m[2, 2] = 1;
		}

		public Matrix(Matrix from)
		{
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					m[i, j] = from.m[i, j];
				}
			}
		}

		public void From( Quaternion Q )
		{
			float qx2 =  Q.x * Q.x * 2;
			float qy2 =  Q.y * Q.y * 2;
			float qz2 =  Q.z * Q.z * 2;
			float qxqy2 = Q.x * Q.y * 2;
			float qxqz2 = Q.x * Q.z * 2;
			float qxqw2 = Q.x * Q.w * 2;
			float qyqz2 = Q.y * Q.z * 2;
			float qyqw2 = Q.y * Q.w * 2;
			float qzqw2 = Q.z * Q.w * 2;

			m[0, 0] = 1 - qy2 - qz2;
			m[0, 1] = qxqy2 - qzqw2;
			m[0, 2] = qxqz2 + qyqw2;

			m[1, 0] = qxqy2 + qzqw2;
			m[1, 1] = 1 - qx2 - qz2;
			m[1, 2] = qyqz2 - qxqw2;

			m[2, 0] = qxqz2 - qyqw2;
			m[2, 1] = qyqz2 + qxqw2;
			m[2, 2] = 1 - qx2 - qy2;
		}


		public Matrix Mul(Matrix m1)
		{
			Matrix r = new Matrix();
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					r.m[i, j] = 0.0f;
					for (int k = 0; k < 3; k++)
					{
						r.m[i, j] += m[i, k] * m1.m[k, j];
					}
				}
			}
			return r;
		}

		public Matrix Transpose()
		{
			Matrix r = new Matrix();
			for(int i = 0; i < 3; i++) {
				for(int j = 0; j < 3; j++) {
					r.m[i, j] = m[j, i];
				}
			}
			return r;
		}

		public Vector Mul( Vector v1 )
		{
			Vector r = new Vector();
			for (int i = 0; i < 3; i++)
			{
				r.v[i] = 0.0f;
				for (int j = 0; j < 3; j++)
				{
					r.v[i] += m[i, j] * v1.v[j];
				}
			}
			return r;
		}


		public Matrix RotateX( float r )
		{
			float s = (float)Math.Sin( r );
			float c = (float)Math.Cos( r );

			Matrix mrot = new Matrix();
			mrot.m[1,1] = c;
			mrot.m[1, 2] = -s;
			mrot.m[2, 1] = s;
			mrot.m[2, 2] = c;

			return this.Mul( mrot );
		}

		public Matrix RotateY( float r )
		{
			float s = (float)Math.Sin( r );
			float c = (float)Math.Cos( r );

			Matrix mrot = new Matrix();
			mrot.m[0, 0] = c;
			mrot.m[0, 2] = s;
			mrot.m[2, 0] = -s;
			mrot.m[2, 2] = c;

			return this.Mul( mrot );
		}

		public Matrix RotateZ( float r )
		{
			float s = (float)Math.Sin( r );
			float c = (float)Math.Cos( r );

			Matrix mrot = new Matrix();
			mrot.m[0, 0] = c;
			mrot.m[0, 1] = -s;
			mrot.m[1, 0] = s;
			mrot.m[1, 1] = c;

			return this.Mul( mrot );
		}
	}
}
