namespace QuaternionVisualizer
{
	partial class MainForm
	{
		/// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.IContainer components = null;

		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		/// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
		protected override void Dispose( bool disposing )
		{
			if(disposing && (components != null))
			{
				components.Dispose();
			}
			base.Dispose( disposing );
		}

		#region Windows Form Designer generated code

		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
			this.components = new System.ComponentModel.Container();
			this.tmCommTimer = new System.Windows.Forms.Timer( this.components );
			this.lblAngles = new System.Windows.Forms.Label();
			this.ocCube = new Elev8.OrientationCube();
			this.SuspendLayout();
			// 
			// tmCommTimer
			// 
			this.tmCommTimer.Enabled = true;
			this.tmCommTimer.Interval = 50;
			this.tmCommTimer.Tick += new System.EventHandler( this.tmCommTimer_Tick );
			// 
			// lblAngles
			// 
			this.lblAngles.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
			this.lblAngles.Location = new System.Drawing.Point( 418, 12 );
			this.lblAngles.Name = "lblAngles";
			this.lblAngles.Size = new System.Drawing.Size( 182, 35 );
			this.lblAngles.TabIndex = 1;
			this.lblAngles.Text = "Angles";
			this.lblAngles.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// ocCube
			// 
			this.ocCube.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
						| System.Windows.Forms.AnchorStyles.Left)
						| System.Windows.Forms.AnchorStyles.Right)));
			this.ocCube.CubeDepth = 1.5F;
			this.ocCube.CubeHeight = 0.8F;
			this.ocCube.CubeWidth = 1.5F;
			this.ocCube.Location = new System.Drawing.Point( 12, 12 );
			this.ocCube.Name = "ocCube";
			this.ocCube.Size = new System.Drawing.Size( 400, 287 );
			this.ocCube.TabIndex = 0;
			this.ocCube.MouseMove += new System.Windows.Forms.MouseEventHandler( this.ocCube_MouseMove );
			this.ocCube.MouseDown += new System.Windows.Forms.MouseEventHandler( this.ocCube_MouseDown );
			this.ocCube.MouseUp += new System.Windows.Forms.MouseEventHandler( this.ocCube_MouseUp );
			// 
			// MainForm
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF( 6F, 13F );
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size( 612, 311 );
			this.Controls.Add( this.lblAngles );
			this.Controls.Add( this.ocCube );
			this.Name = "MainForm";
			this.Text = "Quaternion Visualizer";
			this.FormClosing += new System.Windows.Forms.FormClosingEventHandler( this.MainForm_FormClosing );
			this.ResumeLayout( false );

		}

		#endregion

		private System.Windows.Forms.Timer tmCommTimer;
		private Elev8.OrientationCube ocCube;
		private System.Windows.Forms.Label lblAngles;
	}
}

