/*
  Elev8 GroundStation

  This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-nc-sa/4.0/

  Written by Jason Dorie
*/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace Elev8
{
	public partial class Gauge : UserControl
	{
		public float range = 100.0f;
		public float value = 0.0f;
		public float GaugeCircle = 0.80f;	// 80% of a circle is full-range for a standard gauge, set to 1.0 to use the entire angular range

		public float displayOffset = 0.0f;
		public float displayScale = 1.0f;
		public string displayPostfix = "";

		int avgCount = 128;

		public MovingAverage movAvg = null;

		//bool autoRange = false;

		public Gauge()
		{
			InitializeComponent();
			movAvg = new MovingAverage( avgCount );

			DoubleBuffered = true;
			ResizeRedraw = true;
		}

		public float Range
		{
			get { return range; }
			set { this.range = value; Invalidate(); }
		}

		public int AverageCount
		{
			get { return avgCount; }
			set { avgCount = value; movAvg = new MovingAverage( avgCount ); }
		}
		
		public float MovingAverage
		{
			get { return movAvg.Value; }
		}

		public float Value
		{
			get { return value; }
			set { this.value = value; movAvg.AddSample( value * displayScale + displayOffset ); Invalidate(); }
		}

		private void Gauge_Paint(object sender, PaintEventArgs e)
		{
			// Compute the angle for the gauge based on the value and current range
			float temp;
			if(Value <= -Range) {
				temp = -Range;
			}
			else if(Value >= Range) {
				temp = Range;
			}
			else {
				temp = Value;
			}

			float angle = (temp / Range) * (float)Math.PI * GaugeCircle;

			float centerX = (float)ClientRectangle.Width * 0.5f;
			float centerY = (float)ClientRectangle.Height * 0.5f;
			float radius = Math.Min( centerX, centerY );

			// Paint the gauge
			Graphics g = e.Graphics;
			g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

			float endX = centerX + (float)Math.Sin( angle ) * radius;
			float endY = centerY - (float)Math.Cos( angle ) * radius;

			g.Clear( BackColor );
			g.FillEllipse( SystemBrushes.ControlLight, this.ClientRectangle );
			g.DrawEllipse( Pens.White, this.ClientRectangle );

			g.DrawLine( Pens.Black, centerX, centerY, endX, endY );

			RectangleF rect = new RectangleF( new PointF(10, ClientRectangle.Height-20), new SizeF( ClientRectangle.Width-10, 20) );

			temp = value * displayScale + displayOffset;
			string s;
			if( displayScale != 1.0 ) {
				s = string.Format( "{0}{1}", temp.ToString("F1"), displayPostfix );
			}
			else {
				s = string.Format( "{0}{1}", temp, displayPostfix );
			}
			g.DrawString( s, this.Font, SystemBrushes.ControlText, rect);



			s = movAvg.Value.ToString( "F1" );
			SizeF strSize = g.MeasureString( s, this.Font );

			float left = ClientRectangle.Width - strSize.Width - 10;
			float width = strSize.Width + 10;
			rect = new RectangleF( new PointF( left, ClientRectangle.Height - 20 ), new SizeF( width, 20 ) );

			g.DrawString( s, this.Font, SystemBrushes.ControlText, rect );
		}
	}
}
