/*****************************************************************************/
/* Project  : AvionicsInstrumentControlDemo                                  */
/* File     : AltimeterInstrumentControl.cs                                  */
/* Version  : 1                                                              */
/* Language : C#                                                             */
/* Summary  : The altimeter instrument control                     */
/* Creation : 16/06/2008                                                     */
/* Autor    : Guillaume CHOUTEAU                                             */
/* History  :                                                                */
/*****************************************************************************/

// Modified by Jason Dorie for resource embedding and high-quality rendering

using System;
using System.ComponentModel;
using System.Windows.Forms;
using System.Collections;
using System.Drawing;
using System.Text;
using System.Data;

namespace Elev8
{
    class Altimeter : InstrumentControl
    {
        #region Fields

        // Parameters
        float altitude;

        // Images
        Bitmap bmpCadran = null;
        Bitmap bmpSmallNeedle = null;
        Bitmap bmpLongNeedle = null;
        Bitmap bmpScroll = null;

        #endregion

        #region Contructor

        /// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.Container components = null;

        public Altimeter()
		{
			// Double bufferisation
			SetStyle(ControlStyles.DoubleBuffer | ControlStyles.UserPaint |
				ControlStyles.AllPaintingInWmPaint, true);

			bmpCadran = new Bitmap( GetEmbeddedResourceStream( "Elev8.Instruments.Altimeter_Background.bmp" ) );
			bmpSmallNeedle = new Bitmap( GetEmbeddedResourceStream( "Elev8.Instruments.SmallNeedleAltimeter.bmp" ) );
			bmpLongNeedle = new Bitmap( GetEmbeddedResourceStream( "Elev8.Instruments.LongNeedleAltimeter.bmp" ) );
			bmpScroll = new Bitmap( GetEmbeddedResourceStream( "Elev8.Instruments.Bandeau_Dérouleur.bmp" ) );

			bmpCadran.MakeTransparent( Color.Yellow );
			bmpLongNeedle.MakeTransparent( Color.Yellow );
			bmpSmallNeedle.MakeTransparent( Color.Yellow );
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
            Point ptCounter = new Point(35, 135);
            Point ptRotation = new Point(150, 150);
            Point ptimgNeedle = new Point(136,39);

			double alphaSmallNeedle;
			float altiTemp = altitude;
			if(altitude >= 0.0) {
				altiTemp = 10000.0f - altitude;
			}
			alphaSmallNeedle = InterpolPhyToAngle( altiTemp, 0, 10000, 0, 359 );

			float altiFraction = altiTemp - (float)Math.Floor(altiTemp / 1000.0f) * 1000.0f;
			double alphaLongNeedle = InterpolPhyToAngle( altiFraction, 0, 1000, 0, 359 );

            float scale = (float)this.Width / bmpCadran.Width;

			pe.Graphics.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBilinear;

            // display counter
            ScrollCounter(pe, bmpScroll, 5, altitude, ptCounter, scale);

            // diplay mask
            Pen maskPen = new Pen(this.BackColor, 30 * scale);
            pe.Graphics.DrawRectangle(maskPen, 0, 0, bmpCadran.Width * scale, bmpCadran.Height * scale);

            // display cadran
            pe.Graphics.DrawImage(bmpCadran, 0, 0, (float)(bmpCadran.Width * scale), (float)(bmpCadran.Height * scale));

			pe.Graphics.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;

            // display small needle
            RotateImage(pe, bmpSmallNeedle, alphaSmallNeedle, ptimgNeedle, ptRotation, scale);

            // display long needle
            RotateImage(pe, bmpLongNeedle, alphaLongNeedle, ptimgNeedle, ptRotation, scale);
        }

        #endregion

        #region Methods


        /// <summary>
        /// Define the physical value to be displayed on the indicator
        /// </summary>
        /// <param name="aircraftAltitude">The aircraft altitude in meters</param>
        public void SetAlimeterParameters(float aircraftAltitude)
        {
			if(altitude == aircraftAltitude) return;
            altitude = aircraftAltitude;

            this.Invalidate();
        }

        #endregion

    }
}
