/*****************************************************************************/
/* Project  : AvionicsInstrumentControlDemo                                  */
/* File     : HeadingIndicatorInstrumentControl.cs                           */
/* Version  : 1                                                              */
/* Language : C#                                                             */
/* Summary  : The heading indicator instrument control                       */
/* Creation : 25/06/2008                                                     */
/* Autor    : Guillaume CHOUTEAU                                             */
/* History  :                                                                */
/*****************************************************************************/

using System;
using System.ComponentModel;
using System.Windows.Forms;
using System.Collections;
using System.Drawing;
using System.Text;
using System.Data;

namespace Elev8
{
    class RadioStick : InstrumentControl
    {
        #region Fields

        // Parameters
        int x, y;
		Pen stickPen = null;
		const int StickDiam = 15;


        // Images
        Bitmap bmpCadran = null; // new Bitmap(AvionicsInstrumentControlDemo.AvionicsInstrumentsControls.AvionicsInstrumentsControlsRessources.RadioStick_Background);

        #endregion

        #region Contructor

        /// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.Container components = null;

		public RadioStick()
		{
			// Double bufferisation
			SetStyle(ControlStyles.DoubleBuffer | ControlStyles.UserPaint |
				ControlStyles.AllPaintingInWmPaint, true);

			bmpCadran = new Bitmap( GetEmbeddedResourceStream( "Elev8.Instruments.RadioStick_Background.bmp" ) );
			bmpCadran.MakeTransparent( Color.Yellow );

			stickPen = new Pen( Color.DimGray, StickDiam * 2 / 3 );
			stickPen.StartCap = System.Drawing.Drawing2D.LineCap.Round;
		}

        #endregion

        #region Component Designer generated code
        /// <summary>
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            components = new System.ComponentModel.Container();
        }
        #endregion

        #region Paint

        protected override void OnPaint(PaintEventArgs pe)
        {
            // Calling the base class OnPaint
            base.OnPaint(pe);

            // Pre Display computings
            //Point ptRotation = new Point(150, 150);
            //Point ptImgAircraft = new Point(73,41);
            //Point ptImgHeadingWeel = new Point(13, 13);

            float scale = (float)this.Width / bmpCadran.Width;
			pe.Graphics.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBilinear;

            // diplay mask
            Pen maskPen = new Pen(this.BackColor, 20 * scale);
            pe.Graphics.DrawRectangle(maskPen, 0, 0, bmpCadran.Width * scale, bmpCadran.Height * scale);

            // display cadran
            pe.Graphics.DrawImage(bmpCadran, 0, 0, (float)(bmpCadran.Width * scale), (float)(bmpCadran.Height * scale));

			// display radio stick

			float centerX = (float)ClientRectangle.Width * 0.5f;
			float centerY = (float)ClientRectangle.Height * 0.5f;

			float radius = Math.Min( centerX, centerY ) * 0.6f;
			float range = 1024.0f;


			float xtemp, ytemp;
			if(x <= -range) {
				xtemp = -range;
			}
			else if(x >= range) {
				xtemp = range;
			}
			else {
				xtemp = x;
			}

			if(y <= -range) {
				ytemp = -range;
			}
			else if(y >= range) {
				ytemp = range;
			}
			else {
				ytemp = y;
			}

			xtemp = (xtemp / range) * radius;
			ytemp = (ytemp / range) * radius;

			float px = centerX + xtemp;
			float py = centerY - ytemp;		// Y is inverted when drawing

			pe.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
			pe.Graphics.DrawLine( stickPen, centerX, centerY, px, py );

			int diam = StickDiam;
			pe.Graphics.FillEllipse( Brushes.DarkGray, px - diam / 2, py - diam / 2, diam, diam );
			pe.Graphics.DrawEllipse( Pens.Black, px - diam / 2, py - diam / 2, diam, diam );
        }

        #endregion

        #region Methods

        /// <summary>
        /// Define the physical value to be displayed on the indicator
        /// </summary>
        /// <param name="aircraftHeading">The aircraft heading in °deg</param>
        public void SetParameters(int XAxis , int YAxis)
        {
			if(x == XAxis && y == YAxis) return;

			x = XAxis;
			y = YAxis;

            this.Refresh();
        }

        #endregion
    }
}
