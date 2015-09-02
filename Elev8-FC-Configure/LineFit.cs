using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Text;
using System.Windows.Forms;

namespace Elev8
{
	public partial class LineFit : UserControl
	{
		public class Sample
		{
			public int t, x, y, z;
		};

		public class SampleD
		{
			public double t, x, y, z;
		}


		List<Sample> Samples = new List<Sample>();
		int sampleIndex = 0;
		float xMin, yMin;
		float xMax, yMax;

		public SampleD dSlope = new SampleD();
		public SampleD dIntercept = new SampleD();


		public LineFit()
		{
			InitializeComponent();
			ResizeRedraw = true;
			DoubleBuffered = true;

			//xMin =  1000000.0f;
			//xMax = -1000000.0f;
			//yMin = 1000000.0f;
			//yMax = -1000000.0f;

			xMin = -200.0f;
			xMax =  500.0f;
			yMin = -1000.0f;
			yMax =  2000.0f;

			dSlope.x = dSlope.y = dSlope.z = dSlope.t = 1.0;
		}


		public void Reset()
		{
			Samples.Clear();
			sampleIndex = 0;
		}


		public void AddSample( Sample newSample , bool bRedraw )
		{
			if(Samples.Count < 4096) {
				Samples.Add( newSample );
			}
			else {
				Samples[sampleIndex] = newSample;
				sampleIndex = (sampleIndex + 1) & 4095;
			}

			if(bRedraw) {
				ComputeLine();
				Invalidate();
			}
		}


		protected override void OnPaint(PaintEventArgs e)
		{
			base.OnPaint( e );

			Graphics g = e.Graphics;
			g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;

			float xs = (float)ClientRectangle.Width / (xMax - xMin);
			float ys = (float)ClientRectangle.Height / (yMax - yMin);

			g.Clear( SystemColors.ControlLight);

			if(Samples.Count < 10) return;

			for( int i = 0; i < Samples.Count; i++ )
			{
				float y, x = (Samples[i].t - xMin) * xs;

				y = ClientRectangle.Height - (Samples[i].x - yMin) * ys;
				g.FillEllipse( Brushes.LightSalmon, x - 1.0f, y - 1.0f, 2.0f, 2.0f );

				y = ClientRectangle.Height - (Samples[i].y - yMin) * ys;
				g.FillEllipse( Brushes.LightGreen, x - 1.0f, y - 1.0f, 2.0f, 2.0f );

				y = ClientRectangle.Height - (Samples[i].z - yMin) * ys;
				g.FillEllipse( Brushes.LightBlue, x - 1.0f, y - 1.0f, 2.0f, 2.0f );
			}

			double sy, sx = xMin;
			double ey, ex = xMax;

			sy = dSlope.x * sx + dIntercept.x;
			ey = dSlope.x * ex + dIntercept.x;
			DrawInterceptLine( g, (float)sx, (float)sy, (float)ex, (float)ey, xs, ys, Pens.Red );

			sy = dSlope.y * sx + dIntercept.y;
			ey = dSlope.y * ex + dIntercept.y;
			DrawInterceptLine( g, (float)sx, (float)sy, (float)ex, (float)ey, xs, ys, Pens.Green );

			sy = dSlope.z * sx + dIntercept.z;
			ey = dSlope.z * ex + dIntercept.z;
			DrawInterceptLine( g, (float)sx, (float)sy, (float)ex, (float)ey, xs, ys, Pens.Blue );
		}


		void DrawInterceptLine( Graphics g, float sx, float sy, float ex, float ey, float xs, float ys, Pen pen )
		{
			sx = (sx - xMin) * xs;
			sy = ClientRectangle.Height - (sy - yMin) * ys;

			ex = (ex - xMin) * xs;
			ey = ClientRectangle.Height - (ey - yMin) * ys;

			g.DrawLine( pen, (float)sx, (float)sy, (float)ex, (float)ey );
		}



		public void ComputeLine()
		{
			SampleD s = new SampleD();
			SampleD st = new SampleD();

			//xMin = yMin = 1000000.0f;
			//xMax = yMax = -1000000.0f;

			s.x = s.y = s.z = s.t = 0.0;
			st.t = st.x = st.y = st.z = 1.0;

			int n = Samples.Count;
			for( int i = 0; i < n; ++i )
			{
				s.t += Samples[i].t;
				s.x += Samples[i].x;
				s.y += Samples[i].y;
				s.z += Samples[i].z;

				/*
				xMin = Math.Min( xMin, Samples[i].t );
				xMax = Math.Max( xMax, Samples[i].t );
				yMin = Math.Min( yMin, Samples[i].x );
				yMax = Math.Max( yMax, Samples[i].x );
				yMin = Math.Min( yMin, Samples[i].y );
				yMax = Math.Max( yMax, Samples[i].y );
				yMin = Math.Min( yMin, Samples[i].z );
				yMax = Math.Max( yMax, Samples[i].z );
				//*/
			}

			for( int i = 0; i < n; ++i )
			{
				double t = Samples[i].t - s.t / n;
				st.t += t * t;
				st.x += t * Samples[i].x;
				st.y += t * Samples[i].y;
				st.z += t * Samples[i].z;
			}

			dSlope.x = st.x / st.t;
			dSlope.y = st.y / st.t;
			dSlope.z = st.z / st.t;

			dIntercept.x = (s.x - s.t * dSlope.x) / n;
			dIntercept.y = (s.y - s.t * dSlope.y) / n;
			dIntercept.z = (s.z - s.t * dSlope.z) / n;
		}
	}
}
