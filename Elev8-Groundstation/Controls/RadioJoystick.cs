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
	public partial class RadioJoystick : UserControl
	{
		public float range = 1024.0f;
		public float xvalue = 0.0f;
		public float yvalue = 0.0f;


		public RadioJoystick()
		{
			InitializeComponent();

			DoubleBuffered = true;
			ResizeRedraw = true;
		}

		public float Range
		{
			get { return range; }
			set { this.range = value; Invalidate(); }
		}

		public float XValue
		{
			get { return xvalue; }
			set { xvalue = value; Invalidate(); }
		}

		public float YValue
		{
			get { return yvalue; }
			set { yvalue = value; Invalidate(); }
		}

		private void RadioJoystick_Paint(object sender, PaintEventArgs e)
		{
			// Compute the angle for the gauge based on the value and current range
			float xtemp, ytemp;
			if(xvalue <= -range) {
				xtemp = -range;
			}
			else if(xvalue >= range) {
				xtemp = range;
			}
			else {
				xtemp = xvalue;
			}

			if(yvalue <= -range) {
				ytemp = -range;
			}
			else if(yvalue >= range) {
				ytemp = range;
			}
			else {
				ytemp = yvalue;
			}

			float centerX = (float)ClientRectangle.Width * 0.5f;
			float centerY = (float)ClientRectangle.Height * 0.5f;

			float radius = Math.Min( centerX, centerY );

			xtemp = (xtemp/range) * radius;
			ytemp = (ytemp/range) * radius;

			float px = centerX + xtemp;
			float py = centerY - ytemp;		// Y is inverted when drawing

			// Paint the gauge
			Graphics g = e.Graphics;
			g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

			g.Clear( SystemColors.ControlLight );
			g.DrawRectangle( Pens.White, this.ClientRectangle );

			g.FillEllipse( Brushes.Red, px - 3, py - 3, 7, 7 );
			g.DrawEllipse( Pens.Black, px - 3, py - 3, 7, 7 );

			RectangleF rect = new RectangleF( new PointF(0, ClientRectangle.Height-20), new SizeF( ClientRectangle.Width, 20) );

			string s = string.Format( "{0}\t{1}", xvalue, yvalue );

			g.DrawString( s, this.Font, SystemBrushes.ControlText, rect);
		}
	}
}
