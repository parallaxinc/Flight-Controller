using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Text;
using System.Windows.Forms;

namespace Elev8
{
	public partial class Graph : UserControl
	{
		public bool Scrolling = true;

		int numSamples = 0;
		int curSample = 0;
		int[][] samples = null;

		Brush backBrush = new SolidBrush( SystemColors.ControlLight );
		Pen[] dataPen = {Pens.Red, Pens.Green, Pens.Blue};
		Rectangle invalidRect = new Rectangle();


		public int[] Mins = new int[3];
		public int[] Maxs = new int[3];
		public float[] Avgs = new float[3];
		public float[] Vars = new float[3];


		public Graph()
		{
			InitializeComponent();

			ResizeRedraw = true;
			for(int i = 0; i < 3; i++)
			{
				Mins[i] = -32700;
				Maxs[i] = 32700;
			}

			SetSampleCount( ClientSize.Width );
			invalidRect = ClientRectangle;
		}

		public void SetSampleCount( int count )
		{
			// TODO : Change this code to copy the maximum possible
			// of the existing sample data to the new array

			int[][] newSamples = new int[count][];
			samples = newSamples;
			numSamples = count;
			curSample = 0;
		}

		public void AddSample( int[] x , bool bInvalidate )
		{
			samples[curSample] = x;
			int invalid = curSample;

			curSample++;
			if(curSample == numSamples) curSample = 0;

			int L = Math.Min( invalidRect.Left, invalid );
			int R = Math.Max( invalidRect.Right, invalid );

			invalidRect = new Rectangle( 0, L, R - L, ClientRectangle.Height );

			if(bInvalidate)
			{
				if(Scrolling)
					Invalidate();
				else
					Invalidate( invalidRect );
			}
		}


		public int[] GetPastSample( int x )
		{
			return samples[(numSamples + curSample - x) % numSamples];
		}

		protected override void OnPaint( PaintEventArgs e )
		{
			base.OnPaint( e );

			Graphics g = e.Graphics;
			int HalfY = ClientSize.Height / 2;
			int fullMin = Mins[0], fullMax = Maxs[0];
			int Range;
			int CenterSample;

			for(int i = 1; i < 3; i++)
			{
				fullMin = Math.Min( fullMin, Mins[i] );
				fullMax = Math.Max( fullMax, Maxs[i] );
			}

			Range = fullMax - fullMin;
			CenterSample = (fullMax + fullMin) / 2;
			if(Range < 100) Range = 100;

			g.FillRectangle( backBrush, e.ClipRectangle );
			if(samples == null) return;

			int start = e.ClipRectangle.Left;
			int end = e.ClipRectangle.Right;

			if(Scrolling == false)
			{
				for(int i = start; i < end; i++)
				{
					if(samples[i] == null) continue;
					for(int j = 0; j < 3; j++)
					{
						int v = (samples[i][j] - CenterSample) * HalfY / Range;
						g.DrawLine( dataPen[j], i, HalfY - v, i, HalfY - v + 1 );
					}
				}
			}
			else
			{
				for(int i = start; i < end; i++)
				{
					int[] s = GetPastSample(numSamples - i);
					if(s == null) continue;
					for(int j = 0; j < 3; j++)
					{
						int v = (s[j] - CenterSample) * HalfY / Range;
						g.DrawLine( dataPen[j], i, HalfY - v, i, HalfY - v + 1 );
					}
				}
			}

			invalidRect = new Rectangle( 0, 0, 0, 0 );
		}

		public void UpdateStats( )
		{
			Mins = new int[3];
			Maxs = new int[3];
			Avgs = new float[3];
			Vars = new float[3];

			if(samples[0] == null) return;

			samples[0].CopyTo( Mins , 0 );
			samples[0].CopyTo( Maxs , 0 );

			for(int i = 0; i < numSamples; i++)
			{
				if(samples[i] == null) return;

				for(int j = 0; j < 3; j++)
				{
					Mins[j] = Math.Min( samples[i][j], Mins[j] );
					Maxs[j] = Math.Max( samples[i][j], Maxs[j] );

					Avgs[j] += (float)samples[i][j];
				}
			}
			Avgs[0] /= (float)numSamples;
			Avgs[1] /= (float)numSamples;
			Avgs[2] /= (float)numSamples;

			for(int i = 0; i < numSamples; i++)
			{
				for(int j = 0; j < 3; j++) {
					Vars[j] += Math.Abs( (float)samples[i][j] - Avgs[j] );
				}
			}

			Vars[0] /= (float)numSamples;
			Vars[1] /= (float)numSamples;
			Vars[2] /= (float)numSamples;

			invalidRect = new Rectangle();
			Update();
		}


		private void Graph_SizeChanged( object sender, EventArgs e )
		{
			SetSampleCount( ClientSize.Width );
		}

	}
}
