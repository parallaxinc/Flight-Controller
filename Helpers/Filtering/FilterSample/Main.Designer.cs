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
			this.grGraph = new Elev8.Graph();
			this.SuspendLayout();
			// 
			// hsStrength
			// 
			this.hsStrength.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
			this.hsStrength.LargeChange = 1;
			this.hsStrength.Location = new System.Drawing.Point(13, 232);
			this.hsStrength.Maximum = 9;
			this.hsStrength.Name = "hsStrength";
			this.hsStrength.Size = new System.Drawing.Size(315, 17);
			this.hsStrength.TabIndex = 1;
			this.hsStrength.TabStop = true;
			this.hsStrength.Value = 9;
			this.hsStrength.ValueChanged += new System.EventHandler(this.hsStrength_ValueChanged);
			// 
			// lblStrength
			// 
			this.lblStrength.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
			this.lblStrength.Location = new System.Drawing.Point(359, 232);
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
			// grGraph
			// 
			this.grGraph.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
			this.grGraph.Location = new System.Drawing.Point(13, 13);
			this.grGraph.Name = "grGraph";
			this.grGraph.Size = new System.Drawing.Size(591, 196);
			this.grGraph.TabIndex = 0;
			// 
			// Main
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size(616, 261);
			this.Controls.Add(this.lblStrength);
			this.Controls.Add(this.hsStrength);
			this.Controls.Add(this.grGraph);
			this.Name = "Main";
			this.Text = "Filter Sample";
			this.ResumeLayout(false);

		}

		#endregion

		private Elev8.Graph grGraph;
		private System.Windows.Forms.HScrollBar hsStrength;
		private System.Windows.Forms.Label lblStrength;
		private System.Windows.Forms.Timer tmTick;
	}
}

