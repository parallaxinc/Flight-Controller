/*****************************************************************************/
/* Project  : AvionicsInstrumentControlDemo                                  */
/* File     : AttitudeIndicator.cs                                           */
/* Version  : 1                                                              */
/* Language : C#                                                             */
/* Summary  : The attitude indicator instrument control                      */
/* Creation : 22/06/2008                                                     */
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
using System.IO;
using System.Reflection;

namespace Elev8
{
    class AttitudeIndicator : InstrumentControl
    {
        #region Fields

        // Parameters
        double PitchAngle = 0; // Phi
		double RollAngle = 0; // Theta


        Bitmap bmpCadran = null;
        Bitmap bmpBoule = null;
        Bitmap bmpAvion = null;

        #endregion

        #region Contructor

        /// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.Container components = null;

        public AttitudeIndicator()
		{
			// Double bufferisation
			SetStyle(ControlStyles.DoubleBuffer | ControlStyles.UserPaint |
				ControlStyles.AllPaintingInWmPaint, true);

			// Images
			bmpCadran = new Bitmap( GetEmbeddedResourceStream( "Elev8.Instruments.Horizon_Background.png" ) );
			bmpBoule = new Bitmap( GetEmbeddedResourceStream( "Elev8.Instruments.Horizon_GroundSky.png" ) );
			bmpAvion = new Bitmap( GetEmbeddedResourceStream( "Elev8.Instruments.Maquette_Avion.png" ) );

			bmpCadran.MakeTransparent( Color.Yellow );
			bmpAvion.MakeTransparent( Color.Yellow );
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

            Point ptBoule = new Point(25, - 210);
            Point ptRotation = new Point(150, 150);

            float scale = (float)this.Width / bmpCadran.Width;

            // Affichages - - - - - - - - - - - - - - - - - - - - - - 

			pe.Graphics.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;

            // display Horizon
            RotateAndTranslate(pe, bmpBoule, RollAngle, 0, ptBoule, (int)(4*PitchAngle), ptRotation, scale);


            // diplay mask
            Pen maskPen = new Pen(this.BackColor, 20*scale);
            pe.Graphics.DrawRectangle(maskPen, 0, 0, bmpCadran.Width * scale, bmpCadran.Height * scale);

            // display cadran
			pe.Graphics.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBilinear;
			pe.Graphics.DrawImage( bmpCadran, 0, 0, (float)(bmpCadran.Width * scale), (float)(bmpCadran.Height * scale) );

            // display aircraft symbol
            pe.Graphics.DrawImage(bmpAvion, (float)((0.5 * bmpCadran.Width - 0.5 * bmpAvion.Width) * scale), (float)((0.5 * bmpCadran.Height - 0.5 * bmpAvion.Height) * scale), (float)(bmpAvion.Width * scale), (float)(bmpAvion.Height * scale));
        }

        #endregion

        #region Methods

        /// <summary>
        /// Define the physical value to be displayed on the indicator
        /// </summary>
        /// <param name="aircraftPitchAngle">The aircraft pitch angle in °deg</param>
        /// <param name="aircraftRollAngle">The aircraft roll angle in °deg</param
        public void SetAttitudeIndicatorParameters(double aircraftPitchAngle, double aircraftRollAngle)
        {
			if(PitchAngle == aircraftPitchAngle && RollAngle == (aircraftRollAngle * Math.PI / 180)) return;

            PitchAngle = aircraftPitchAngle;
            RollAngle = aircraftRollAngle * Math.PI / 180;

            this.Refresh();
        }

        #endregion
    }
}
