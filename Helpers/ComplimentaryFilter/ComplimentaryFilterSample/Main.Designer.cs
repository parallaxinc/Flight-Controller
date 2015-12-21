namespace FilterSample
{
	partial class Main
	{
		/// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.IContainer components = null;

		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		/// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
		protected override void Dispose(bool disposing)
		{
			if (disposing && (components != null))
			{
				components.Dispose();
			}
			base.Dispose(disposing);
		}

		#region Windows Form Designer generated code

		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
			this.components = new System.ComponentModel.Container();
			this.hsStrength = new System.Windows.Forms.HScrollBar();
			this.lblStrength = new System.Windows.Forms.Label();
			this.tmTick = new System.Windows.Forms.Timer(this.components);
			this.gaugeFiltered = new Elev8.Gauge();
			this.gaugeAccel = new Elev8.Gauge();
			this.gaugeGyro = new Elev8.Gauge();
			this.SuspendLayout();
			// 
			// hsStrength
			// 
			this.hsStrength.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
			this.hsStrength.LargeChange = 1;
			this.hsStrength.Location = new System.Drawing.Point(13, 232);
			this.hsStrength.Name = "hsStrength";
			this.hsStrength.Size = new System.Drawing.Size(178, 17);
			this.hsStrength.TabIndex = 1;
			this.hsStrength.TabStop = true;
			this.hsStrength.Value = 50;
			this.hsStrength.ValueChanged += new System.EventHandler(this.hsStrength_ValueChanged);
			// 
			// lblStrength
			// 
			this.lblStrength.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
			this.lblStrength.Location = new System.Drawing.Point(224, 232);
			this.lblStrength.Name = "lblStrength";
			this.lblStrength.Size = new System.Drawing.Size(171, 17);
			this.lblStrength.TabIndex = 2;
			this.lblStrength.Text = "label1";
			this.lblStrength.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
			// 
			// tmTick
			// 
			this.tmTick.Interval = 20;
			this.tmTick.Tick += new System.EventHandler(this.tmTick_Tick);
			// 
			// gaugeFiltered
			// 
			this.gaugeFiltered.Location = new System.Drawing.Point(156, 12);
			this.gaugeFiltered.Name = "gaugeFiltered";
			this.gaugeFiltered.Range = 180F;
			this.gaugeFiltered.Size = new System.Drawing.Size(184, 184);
			this.gaugeFiltered.TabIndex = 4;
			this.gaugeFiltered.Value = 0F;
			// 
			// gaugeAccel
			// 
			this.gaugeAccel.Location = new System.Drawing.Point(16, 42);
			this.gaugeAccel.Name = "gaugeAccel";
			this.gaugeAccel.Range = 180F;
			this.gaugeAccel.Size = new System.Drawing.Size(134, 134);
			this.gaugeAccel.TabIndex = 5;
			this.gaugeAccel.Value = 0F;
			// 
			// gaugeGyro
			// 
			this.gaugeGyro.Location = new System.Drawing.Point(346, 42);
			this.gaugeGyro.Name = "gaugeGyro";
			this.gaugeGyro.Range = 180F;
			this.gaugeGyro.Size = new System.Drawing.Size(134, 134);
			this.gaugeGyro.TabIndex = 6;
			this.gaugeGyro.Value = 0F;
			// 
			// Main
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size(497, 261);
			this.Controls.Add(this.gaugeGyro);
			this.Controls.Add(this.gaugeAccel);
			this.Controls.Add(this.gaugeFiltered);
			this.Controls.Add(this.lblStrength);
			this.Controls.Add(this.hsStrength);
			this.Name = "Main";
			this.Text = "Complimentary Filter Sample";
			this.ResumeLayout(false);

		}

		#endregion

		private System.Windows.Forms.HScrollBar hsStrength;
		private System.Windows.Forms.Label lblStrength;
		private System.Windows.Forms.Timer tmTick;
		private Elev8.Gauge gaugeFiltered;
		private Elev8.Gauge gaugeAccel;
		private Elev8.Gauge gaugeGyro;
	}
}

