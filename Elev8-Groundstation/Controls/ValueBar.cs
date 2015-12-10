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

namespace Elev8.Controls
{
	public partial class ValueBar : UserControl
	{
		int val = 0;
		int minVal = -1024;
		int maxVal = 1024;
		bool fromLeft = true;
		string leftLabel = "";
		string rightLabel = "";
		int buffer = 2;

		Brush barBrush = Brushes.LightGreen;
		Color barColor = Color.LightGreen;


		public ValueBar()
		{
			SetStyle( ControlStyles.DoubleBuffer | ControlStyles.UserPaint |
				ControlStyles.AllPaintingInWmPaint, true );

			InitializeComponent();
		}

		public int Value
		{
			get { return val; }
			set {
				if(val == value) return;
				val = value; Invalidate();
			}
		}

		public int MinValue
		{
			get { return minVal; }
			set { minVal = value; }
		}

		public int MaxValue
		{
			get { return maxVal; }
			set { maxVal = value; }
		}

		public bool FromLeft
		{
			get { return fromLeft; }
			set { fromLeft = value; }
		}


		public string LeftLabel
		{
			get { return leftLabel; }
			set { leftLabel = value; Invalidate(); }
		}

		public string RightLabel
		{
			get { return rightLabel; }
			set { rightLabel = value; Invalidate(); }
		}

		public Color BarColor
		{
			get { return barColor; }
			set { barColor = value; barBrush = new SolidBrush( barColor ); Invalidate(); }
		}


		private void ValueBar_Paint( object sender, PaintEventArgs e )
		{
			// Draw the bar as background color

			Graphics g = e.Graphics;
			int scale = ClientSize.Width - (buffer * 2);
			int height = ClientSize.Height;

			g.Clear( BackColor );
			float l;

			int clampedVal = Math.Max( minVal, val );
			clampedVal = Math.Min( maxVal, clampedVal);

			float width = (clampedVal - minVal) * scale / (maxVal - minVal);

			if(fromLeft) {
				l = buffer;
			}
			else {
				l = (ClientSize.Width-buffer) - width;
			}

			g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
			g.FillRectangle( barBrush, l, 0.0f, width, (float)height );

			System.Windows.Forms.ControlPaint.DrawBorder3D( g, 0, 0, ClientSize.Width, height );

			// Draw the left label
			SizeF strSize = g.MeasureString( leftLabel, Font );
			g.DrawString( leftLabel, Font, Brushes.Black, buffer, ((float)Height - strSize.Height) * 0.5f );

			// Draw the right label
			strSize = g.MeasureString( rightLabel, Font );
			g.DrawString( rightLabel, Font, Brushes.Black, (ClientSize.Width - buffer) - strSize.Width, ((float)Height - strSize.Height) * 0.5f );
		}



	}
}
