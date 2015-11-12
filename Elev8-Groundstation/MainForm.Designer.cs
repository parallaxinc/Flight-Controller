namespace Elev8
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
			System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager( typeof( MainForm ) );
			this.stMainStatus = new System.Windows.Forms.StatusStrip();
			this.tssStatusText = new System.Windows.Forms.ToolStripStatusLabel();
			this.tmCommTimer = new System.Windows.Forms.Timer( this.components );
			this.tcTabs = new System.Windows.Forms.TabControl();
			this.tpStatus = new System.Windows.Forms.TabPage();
			this.vbChannel8 = new Elev8.Controls.ValueBar();
			this.vbChannel7 = new Elev8.Controls.ValueBar();
			this.vbChannel6 = new Elev8.Controls.ValueBar();
			this.vbChannel5 = new Elev8.Controls.ValueBar();
			this.vbRS_XValue = new Elev8.Controls.ValueBar();
			this.vbRS_YValue = new Elev8.Controls.ValueBar();
			this.vbLS_XValue = new Elev8.Controls.ValueBar();
			this.vbLS_YValue = new Elev8.Controls.ValueBar();
			this.rsRight = new Elev8.RadioStick();
			this.rsLeft = new Elev8.RadioStick();
			this.aicAttitude = new Elev8.AttitudeIndicator();
			this.ocOrientation = new Elev8.OrientationCube();
			this.aicHeading = new Elev8.HeadingIndicator();
			this.aicAltimeter = new Elev8.Altimeter();
			this.tpSensors = new System.Windows.Forms.TabPage();
			this.cbVoltage = new System.Windows.Forms.CheckBox();
			this.cbYaw = new System.Windows.Forms.CheckBox();
			this.cbRoll = new System.Windows.Forms.CheckBox();
			this.cbPitch = new System.Windows.Forms.CheckBox();
			this.cbAltitude = new System.Windows.Forms.CheckBox();
			this.cbMagZ = new System.Windows.Forms.CheckBox();
			this.cbMagY = new System.Windows.Forms.CheckBox();
			this.cbMagX = new System.Windows.Forms.CheckBox();
			this.cbAccelZ = new System.Windows.Forms.CheckBox();
			this.cbAccelY = new System.Windows.Forms.CheckBox();
			this.cbAccelX = new System.Windows.Forms.CheckBox();
			this.cbGyroZ = new System.Windows.Forms.CheckBox();
			this.cbGyroY = new System.Windows.Forms.CheckBox();
			this.cbGyroX = new System.Windows.Forms.CheckBox();
			this.tpSysTest = new System.Windows.Forms.TabPage();
			this.lblCalibrateDocs = new System.Windows.Forms.Label();
			this.btnThrottleCalibrate = new System.Windows.Forms.Button();
			this.btnLED = new System.Windows.Forms.Button();
			this.btnBeeper = new System.Windows.Forms.Button();
			this.textBox3 = new System.Windows.Forms.TextBox();
			this.btnMotor4 = new System.Windows.Forms.Button();
			this.btnMotor3 = new System.Windows.Forms.Button();
			this.btnMotor2 = new System.Windows.Forms.Button();
			this.btnMotor1 = new System.Windows.Forms.Button();
			this.tpControlSetup = new System.Windows.Forms.TabPage();
			this.cbChannel8 = new System.Windows.Forms.ComboBox();
			this.cbChannel7 = new System.Windows.Forms.ComboBox();
			this.cbChannel2 = new System.Windows.Forms.ComboBox();
			this.cbChannel3 = new System.Windows.Forms.ComboBox();
			this.cbChannel6 = new System.Windows.Forms.ComboBox();
			this.cbChannel5 = new System.Windows.Forms.ComboBox();
			this.cbChannel4 = new System.Windows.Forms.ComboBox();
			this.cbChannel1 = new System.Windows.Forms.ComboBox();
			this.vbR_Channel8 = new Elev8.Controls.ValueBar();
			this.vbR_Channel7 = new Elev8.Controls.ValueBar();
			this.vbR_Channel6 = new Elev8.Controls.ValueBar();
			this.vbR_Channel5 = new Elev8.Controls.ValueBar();
			this.vbR_RS_XValue = new Elev8.Controls.ValueBar();
			this.vbR_RS_YValue = new Elev8.Controls.ValueBar();
			this.vbR_LS_XValue = new Elev8.Controls.ValueBar();
			this.vbR_LS_YValue = new Elev8.Controls.ValueBar();
			this.rsR_Right = new Elev8.RadioStick();
			this.rsR_Left = new Elev8.RadioStick();
			this.tpSystemSetup = new System.Windows.Forms.TabPage();
			this.msMainMenu = new System.Windows.Forms.MenuStrip();
			this.settingsToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
			this.radioDisplayToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
			this.toolStripMenuItem2 = new System.Windows.Forms.ToolStripSeparator();
			this.miRadioMode1 = new System.Windows.Forms.ToolStripMenuItem();
			this.miRadioMode2 = new System.Windows.Forms.ToolStripMenuItem();
			this.toolStripMenuItem1 = new System.Windows.Forms.ToolStripSeparator();
			this.stMainStatus.SuspendLayout();
			this.tcTabs.SuspendLayout();
			this.tpStatus.SuspendLayout();
			this.tpSensors.SuspendLayout();
			this.tpSysTest.SuspendLayout();
			this.tpControlSetup.SuspendLayout();
			this.msMainMenu.SuspendLayout();
			this.SuspendLayout();
			// 
			// stMainStatus
			// 
			this.stMainStatus.Items.AddRange( new System.Windows.Forms.ToolStripItem[] {
            this.tssStatusText} );
			this.stMainStatus.Location = new System.Drawing.Point( 0, 406 );
			this.stMainStatus.Name = "stMainStatus";
			this.stMainStatus.Size = new System.Drawing.Size( 729, 22 );
			this.stMainStatus.TabIndex = 0;
			this.stMainStatus.Text = "status";
			// 
			// tssStatusText
			// 
			this.tssStatusText.Name = "tssStatusText";
			this.tssStatusText.Size = new System.Drawing.Size( 78, 17 );
			this.tssStatusText.Text = "Connecting...";
			// 
			// tmCommTimer
			// 
			this.tmCommTimer.Enabled = true;
			this.tmCommTimer.Interval = 50;
			this.tmCommTimer.Tick += new System.EventHandler( this.tmCommTimer_Tick );
			// 
			// tcTabs
			// 
			this.tcTabs.Controls.Add( this.tpStatus );
			this.tcTabs.Controls.Add( this.tpSensors );
			this.tcTabs.Controls.Add( this.tpSysTest );
			this.tcTabs.Controls.Add( this.tpControlSetup );
			this.tcTabs.Controls.Add( this.tpSystemSetup );
			this.tcTabs.Dock = System.Windows.Forms.DockStyle.Fill;
			this.tcTabs.Location = new System.Drawing.Point( 0, 24 );
			this.tcTabs.Name = "tcTabs";
			this.tcTabs.SelectedIndex = 0;
			this.tcTabs.Size = new System.Drawing.Size( 729, 382 );
			this.tcTabs.TabIndex = 7;
			// 
			// tpStatus
			// 
			this.tpStatus.BackColor = System.Drawing.SystemColors.Control;
			this.tpStatus.Controls.Add( this.vbChannel8 );
			this.tpStatus.Controls.Add( this.vbChannel7 );
			this.tpStatus.Controls.Add( this.vbChannel6 );
			this.tpStatus.Controls.Add( this.vbChannel5 );
			this.tpStatus.Controls.Add( this.vbRS_XValue );
			this.tpStatus.Controls.Add( this.vbRS_YValue );
			this.tpStatus.Controls.Add( this.vbLS_XValue );
			this.tpStatus.Controls.Add( this.vbLS_YValue );
			this.tpStatus.Controls.Add( this.rsRight );
			this.tpStatus.Controls.Add( this.rsLeft );
			this.tpStatus.Controls.Add( this.aicAttitude );
			this.tpStatus.Controls.Add( this.ocOrientation );
			this.tpStatus.Controls.Add( this.aicHeading );
			this.tpStatus.Controls.Add( this.aicAltimeter );
			this.tpStatus.Location = new System.Drawing.Point( 4, 22 );
			this.tpStatus.Name = "tpStatus";
			this.tpStatus.Padding = new System.Windows.Forms.Padding( 3 );
			this.tpStatus.Size = new System.Drawing.Size( 721, 356 );
			this.tpStatus.TabIndex = 0;
			this.tpStatus.Text = "Status";
			// 
			// vbChannel8
			// 
			this.vbChannel8.FromLeft = true;
			this.vbChannel8.LeftLabel = "Aux3";
			this.vbChannel8.Location = new System.Drawing.Point( 425, 290 );
			this.vbChannel8.MaxValue = 1024;
			this.vbChannel8.MinValue = -1024;
			this.vbChannel8.Name = "vbChannel8";
			this.vbChannel8.RightLabel = "0";
			this.vbChannel8.Size = new System.Drawing.Size( 83, 20 );
			this.vbChannel8.TabIndex = 32;
			this.vbChannel8.Value = 0;
			// 
			// vbChannel7
			// 
			this.vbChannel7.FromLeft = true;
			this.vbChannel7.LeftLabel = "Aux2";
			this.vbChannel7.Location = new System.Drawing.Point( 425, 262 );
			this.vbChannel7.MaxValue = 1024;
			this.vbChannel7.MinValue = -1024;
			this.vbChannel7.Name = "vbChannel7";
			this.vbChannel7.RightLabel = "0";
			this.vbChannel7.Size = new System.Drawing.Size( 83, 20 );
			this.vbChannel7.TabIndex = 31;
			this.vbChannel7.Value = 0;
			// 
			// vbChannel6
			// 
			this.vbChannel6.FromLeft = true;
			this.vbChannel6.LeftLabel = "Aux1";
			this.vbChannel6.Location = new System.Drawing.Point( 9, 290 );
			this.vbChannel6.MaxValue = 1024;
			this.vbChannel6.MinValue = -1024;
			this.vbChannel6.Name = "vbChannel6";
			this.vbChannel6.RightLabel = "0";
			this.vbChannel6.Size = new System.Drawing.Size( 83, 20 );
			this.vbChannel6.TabIndex = 30;
			this.vbChannel6.Value = 0;
			// 
			// vbChannel5
			// 
			this.vbChannel5.FromLeft = true;
			this.vbChannel5.LeftLabel = "Gear";
			this.vbChannel5.Location = new System.Drawing.Point( 9, 262 );
			this.vbChannel5.MaxValue = 1024;
			this.vbChannel5.MinValue = -1024;
			this.vbChannel5.Name = "vbChannel5";
			this.vbChannel5.RightLabel = "0";
			this.vbChannel5.Size = new System.Drawing.Size( 83, 20 );
			this.vbChannel5.TabIndex = 29;
			this.vbChannel5.Value = 0;
			// 
			// vbRS_XValue
			// 
			this.vbRS_XValue.FromLeft = true;
			this.vbRS_XValue.LeftLabel = "Aileron";
			this.vbRS_XValue.Location = new System.Drawing.Point( 425, 234 );
			this.vbRS_XValue.MaxValue = 1024;
			this.vbRS_XValue.MinValue = -1024;
			this.vbRS_XValue.Name = "vbRS_XValue";
			this.vbRS_XValue.RightLabel = "0";
			this.vbRS_XValue.Size = new System.Drawing.Size( 83, 20 );
			this.vbRS_XValue.TabIndex = 28;
			this.vbRS_XValue.Value = 0;
			// 
			// vbRS_YValue
			// 
			this.vbRS_YValue.FromLeft = true;
			this.vbRS_YValue.LeftLabel = "Elevator";
			this.vbRS_YValue.Location = new System.Drawing.Point( 425, 206 );
			this.vbRS_YValue.MaxValue = 1024;
			this.vbRS_YValue.MinValue = -1024;
			this.vbRS_YValue.Name = "vbRS_YValue";
			this.vbRS_YValue.RightLabel = "0";
			this.vbRS_YValue.Size = new System.Drawing.Size( 83, 20 );
			this.vbRS_YValue.TabIndex = 27;
			this.vbRS_YValue.Value = 0;
			// 
			// vbLS_XValue
			// 
			this.vbLS_XValue.FromLeft = true;
			this.vbLS_XValue.LeftLabel = "Rudder";
			this.vbLS_XValue.Location = new System.Drawing.Point( 9, 234 );
			this.vbLS_XValue.MaxValue = 1024;
			this.vbLS_XValue.MinValue = -1024;
			this.vbLS_XValue.Name = "vbLS_XValue";
			this.vbLS_XValue.RightLabel = "0";
			this.vbLS_XValue.Size = new System.Drawing.Size( 83, 20 );
			this.vbLS_XValue.TabIndex = 26;
			this.vbLS_XValue.Value = 0;
			// 
			// vbLS_YValue
			// 
			this.vbLS_YValue.FromLeft = true;
			this.vbLS_YValue.LeftLabel = "Throttle";
			this.vbLS_YValue.Location = new System.Drawing.Point( 9, 206 );
			this.vbLS_YValue.MaxValue = 1024;
			this.vbLS_YValue.MinValue = -1024;
			this.vbLS_YValue.Name = "vbLS_YValue";
			this.vbLS_YValue.RightLabel = "0";
			this.vbLS_YValue.Size = new System.Drawing.Size( 83, 20 );
			this.vbLS_YValue.TabIndex = 25;
			this.vbLS_YValue.Value = 0;
			// 
			// rsRight
			// 
			this.rsRight.Location = new System.Drawing.Point( 265, 182 );
			this.rsRight.Name = "rsRight";
			this.rsRight.Size = new System.Drawing.Size( 150, 150 );
			this.rsRight.TabIndex = 8;
			this.rsRight.Text = "radioStick2";
			// 
			// rsLeft
			// 
			this.rsLeft.Location = new System.Drawing.Point( 102, 182 );
			this.rsLeft.Name = "rsLeft";
			this.rsLeft.Size = new System.Drawing.Size( 150, 150 );
			this.rsLeft.TabIndex = 7;
			this.rsLeft.Text = "radioStick1";
			// 
			// aicAttitude
			// 
			this.aicAttitude.Location = new System.Drawing.Point( 183, 6 );
			this.aicAttitude.Name = "aicAttitude";
			this.aicAttitude.Size = new System.Drawing.Size( 150, 150 );
			this.aicAttitude.TabIndex = 4;
			this.aicAttitude.Text = "attitudeIndicatorInstrumentControl1";
			// 
			// ocOrientation
			// 
			this.ocOrientation.CubeDepth = 1.2F;
			this.ocOrientation.CubeHeight = 0.8F;
			this.ocOrientation.CubeWidth = 1.2F;
			this.ocOrientation.Location = new System.Drawing.Point( 507, 6 );
			this.ocOrientation.Name = "ocOrientation";
			this.ocOrientation.Size = new System.Drawing.Size( 206, 150 );
			this.ocOrientation.TabIndex = 3;
			// 
			// aicHeading
			// 
			this.aicHeading.Location = new System.Drawing.Point( 351, 6 );
			this.aicHeading.Name = "aicHeading";
			this.aicHeading.Size = new System.Drawing.Size( 150, 150 );
			this.aicHeading.TabIndex = 6;
			this.aicHeading.Text = "headingIndicator1";
			// 
			// aicAltimeter
			// 
			this.aicAltimeter.Location = new System.Drawing.Point( 15, 6 );
			this.aicAltimeter.Name = "aicAltimeter";
			this.aicAltimeter.Size = new System.Drawing.Size( 150, 150 );
			this.aicAltimeter.TabIndex = 5;
			this.aicAltimeter.Text = "altimeter1";
			// 
			// tpSensors
			// 
			this.tpSensors.BackColor = System.Drawing.SystemColors.Control;
			this.tpSensors.Controls.Add( this.cbVoltage );
			this.tpSensors.Controls.Add( this.cbYaw );
			this.tpSensors.Controls.Add( this.cbRoll );
			this.tpSensors.Controls.Add( this.cbPitch );
			this.tpSensors.Controls.Add( this.cbAltitude );
			this.tpSensors.Controls.Add( this.cbMagZ );
			this.tpSensors.Controls.Add( this.cbMagY );
			this.tpSensors.Controls.Add( this.cbMagX );
			this.tpSensors.Controls.Add( this.cbAccelZ );
			this.tpSensors.Controls.Add( this.cbAccelY );
			this.tpSensors.Controls.Add( this.cbAccelX );
			this.tpSensors.Controls.Add( this.cbGyroZ );
			this.tpSensors.Controls.Add( this.cbGyroY );
			this.tpSensors.Controls.Add( this.cbGyroX );
			this.tpSensors.Location = new System.Drawing.Point( 4, 22 );
			this.tpSensors.Name = "tpSensors";
			this.tpSensors.Padding = new System.Windows.Forms.Padding( 3 );
			this.tpSensors.Size = new System.Drawing.Size( 721, 356 );
			this.tpSensors.TabIndex = 1;
			this.tpSensors.Text = "Sensors";
			// 
			// cbVoltage
			// 
			this.cbVoltage.AutoSize = true;
			this.cbVoltage.Location = new System.Drawing.Point( 8, 288 );
			this.cbVoltage.Name = "cbVoltage";
			this.cbVoltage.Size = new System.Drawing.Size( 62, 17 );
			this.cbVoltage.TabIndex = 13;
			this.cbVoltage.Text = "Voltage";
			this.cbVoltage.UseVisualStyleBackColor = true;
			// 
			// cbYaw
			// 
			this.cbYaw.AutoSize = true;
			this.cbYaw.Location = new System.Drawing.Point( 8, 268 );
			this.cbYaw.Name = "cbYaw";
			this.cbYaw.Size = new System.Drawing.Size( 47, 17 );
			this.cbYaw.TabIndex = 12;
			this.cbYaw.Text = "Yaw";
			this.cbYaw.UseVisualStyleBackColor = true;
			// 
			// cbRoll
			// 
			this.cbRoll.AutoSize = true;
			this.cbRoll.Location = new System.Drawing.Point( 8, 248 );
			this.cbRoll.Name = "cbRoll";
			this.cbRoll.Size = new System.Drawing.Size( 44, 17 );
			this.cbRoll.TabIndex = 11;
			this.cbRoll.Text = "Roll";
			this.cbRoll.UseVisualStyleBackColor = true;
			// 
			// cbPitch
			// 
			this.cbPitch.AutoSize = true;
			this.cbPitch.Location = new System.Drawing.Point( 8, 228 );
			this.cbPitch.Name = "cbPitch";
			this.cbPitch.Size = new System.Drawing.Size( 50, 17 );
			this.cbPitch.TabIndex = 10;
			this.cbPitch.Text = "Pitch";
			this.cbPitch.UseVisualStyleBackColor = true;
			// 
			// cbAltitude
			// 
			this.cbAltitude.AutoSize = true;
			this.cbAltitude.Location = new System.Drawing.Point( 8, 208 );
			this.cbAltitude.Name = "cbAltitude";
			this.cbAltitude.Size = new System.Drawing.Size( 61, 17 );
			this.cbAltitude.TabIndex = 9;
			this.cbAltitude.Text = "Altitude";
			this.cbAltitude.UseVisualStyleBackColor = true;
			// 
			// cbMagZ
			// 
			this.cbMagZ.AutoSize = true;
			this.cbMagZ.Location = new System.Drawing.Point( 8, 176 );
			this.cbMagZ.Name = "cbMagZ";
			this.cbMagZ.Size = new System.Drawing.Size( 57, 17 );
			this.cbMagZ.TabIndex = 8;
			this.cbMagZ.Text = "Mag Z";
			this.cbMagZ.UseVisualStyleBackColor = true;
			// 
			// cbMagY
			// 
			this.cbMagY.AutoSize = true;
			this.cbMagY.Location = new System.Drawing.Point( 8, 156 );
			this.cbMagY.Name = "cbMagY";
			this.cbMagY.Size = new System.Drawing.Size( 57, 17 );
			this.cbMagY.TabIndex = 7;
			this.cbMagY.Text = "Mag Y";
			this.cbMagY.UseVisualStyleBackColor = true;
			// 
			// cbMagX
			// 
			this.cbMagX.AutoSize = true;
			this.cbMagX.Location = new System.Drawing.Point( 8, 136 );
			this.cbMagX.Name = "cbMagX";
			this.cbMagX.Size = new System.Drawing.Size( 57, 17 );
			this.cbMagX.TabIndex = 6;
			this.cbMagX.Text = "Mag X";
			this.cbMagX.UseVisualStyleBackColor = true;
			// 
			// cbAccelZ
			// 
			this.cbAccelZ.AutoSize = true;
			this.cbAccelZ.Location = new System.Drawing.Point( 8, 116 );
			this.cbAccelZ.Name = "cbAccelZ";
			this.cbAccelZ.Size = new System.Drawing.Size( 63, 17 );
			this.cbAccelZ.TabIndex = 5;
			this.cbAccelZ.Text = "Accel Z";
			this.cbAccelZ.UseVisualStyleBackColor = true;
			// 
			// cbAccelY
			// 
			this.cbAccelY.AutoSize = true;
			this.cbAccelY.Location = new System.Drawing.Point( 8, 96 );
			this.cbAccelY.Name = "cbAccelY";
			this.cbAccelY.Size = new System.Drawing.Size( 63, 17 );
			this.cbAccelY.TabIndex = 4;
			this.cbAccelY.Text = "Accel Y";
			this.cbAccelY.UseVisualStyleBackColor = true;
			// 
			// cbAccelX
			// 
			this.cbAccelX.AutoSize = true;
			this.cbAccelX.Location = new System.Drawing.Point( 8, 76 );
			this.cbAccelX.Name = "cbAccelX";
			this.cbAccelX.Size = new System.Drawing.Size( 63, 17 );
			this.cbAccelX.TabIndex = 3;
			this.cbAccelX.Text = "Accel X";
			this.cbAccelX.UseVisualStyleBackColor = true;
			// 
			// cbGyroZ
			// 
			this.cbGyroZ.AutoSize = true;
			this.cbGyroZ.Location = new System.Drawing.Point( 8, 56 );
			this.cbGyroZ.Name = "cbGyroZ";
			this.cbGyroZ.Size = new System.Drawing.Size( 58, 17 );
			this.cbGyroZ.TabIndex = 2;
			this.cbGyroZ.Text = "Gyro Z";
			this.cbGyroZ.UseVisualStyleBackColor = true;
			// 
			// cbGyroY
			// 
			this.cbGyroY.AutoSize = true;
			this.cbGyroY.Location = new System.Drawing.Point( 8, 36 );
			this.cbGyroY.Name = "cbGyroY";
			this.cbGyroY.Size = new System.Drawing.Size( 58, 17 );
			this.cbGyroY.TabIndex = 1;
			this.cbGyroY.Text = "Gyro Y";
			this.cbGyroY.UseVisualStyleBackColor = true;
			// 
			// cbGyroX
			// 
			this.cbGyroX.AutoSize = true;
			this.cbGyroX.Location = new System.Drawing.Point( 8, 16 );
			this.cbGyroX.Name = "cbGyroX";
			this.cbGyroX.Size = new System.Drawing.Size( 58, 17 );
			this.cbGyroX.TabIndex = 0;
			this.cbGyroX.Text = "Gyro X";
			this.cbGyroX.UseVisualStyleBackColor = true;
			// 
			// tpSysTest
			// 
			this.tpSysTest.BackColor = System.Drawing.SystemColors.Control;
			this.tpSysTest.Controls.Add( this.lblCalibrateDocs );
			this.tpSysTest.Controls.Add( this.btnThrottleCalibrate );
			this.tpSysTest.Controls.Add( this.btnLED );
			this.tpSysTest.Controls.Add( this.btnBeeper );
			this.tpSysTest.Controls.Add( this.textBox3 );
			this.tpSysTest.Controls.Add( this.btnMotor4 );
			this.tpSysTest.Controls.Add( this.btnMotor3 );
			this.tpSysTest.Controls.Add( this.btnMotor2 );
			this.tpSysTest.Controls.Add( this.btnMotor1 );
			this.tpSysTest.Location = new System.Drawing.Point( 4, 22 );
			this.tpSysTest.Name = "tpSysTest";
			this.tpSysTest.Size = new System.Drawing.Size( 721, 356 );
			this.tpSysTest.TabIndex = 2;
			this.tpSysTest.Text = "System Test";
			// 
			// lblCalibrateDocs
			// 
			this.lblCalibrateDocs.Location = new System.Drawing.Point( 119, 296 );
			this.lblCalibrateDocs.Name = "lblCalibrateDocs";
			this.lblCalibrateDocs.Size = new System.Drawing.Size( 301, 31 );
			this.lblCalibrateDocs.TabIndex = 27;
			this.lblCalibrateDocs.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// btnThrottleCalibrate
			// 
			this.btnThrottleCalibrate.Location = new System.Drawing.Point( 232, 236 );
			this.btnThrottleCalibrate.Name = "btnThrottleCalibrate";
			this.btnThrottleCalibrate.Size = new System.Drawing.Size( 75, 47 );
			this.btnThrottleCalibrate.TabIndex = 26;
			this.btnThrottleCalibrate.Text = "Throttle Calibration";
			this.btnThrottleCalibrate.UseVisualStyleBackColor = true;
			// 
			// btnLED
			// 
			this.btnLED.Location = new System.Drawing.Point( 232, 178 );
			this.btnLED.Name = "btnLED";
			this.btnLED.Size = new System.Drawing.Size( 75, 33 );
			this.btnLED.TabIndex = 24;
			this.btnLED.Text = "LED";
			this.btnLED.UseVisualStyleBackColor = true;
			// 
			// btnBeeper
			// 
			this.btnBeeper.Location = new System.Drawing.Point( 232, 130 );
			this.btnBeeper.Name = "btnBeeper";
			this.btnBeeper.Size = new System.Drawing.Size( 75, 33 );
			this.btnBeeper.TabIndex = 23;
			this.btnBeeper.Text = "Beeper";
			this.btnBeeper.UseVisualStyleBackColor = true;
			// 
			// textBox3
			// 
			this.textBox3.BorderStyle = System.Windows.Forms.BorderStyle.None;
			this.textBox3.Enabled = false;
			this.textBox3.Location = new System.Drawing.Point( 469, 90 );
			this.textBox3.Multiline = true;
			this.textBox3.Name = "textBox3";
			this.textBox3.ReadOnly = true;
			this.textBox3.Size = new System.Drawing.Size( 185, 155 );
			this.textBox3.TabIndex = 25;
			this.textBox3.Text = resources.GetString( "textBox3.Text" );
			this.textBox3.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
			// 
			// btnMotor4
			// 
			this.btnMotor4.BackgroundImage = global::Elev8.Properties.Resources.Icon_Counter_clockwise;
			this.btnMotor4.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
			this.btnMotor4.Location = new System.Drawing.Point( 119, 226 );
			this.btnMotor4.Name = "btnMotor4";
			this.btnMotor4.Size = new System.Drawing.Size( 75, 67 );
			this.btnMotor4.TabIndex = 22;
			this.btnMotor4.Text = "4";
			this.btnMotor4.TextAlign = System.Drawing.ContentAlignment.BottomLeft;
			this.btnMotor4.UseVisualStyleBackColor = true;
			// 
			// btnMotor3
			// 
			this.btnMotor3.BackgroundImage = global::Elev8.Properties.Resources.Icon_Clockwise;
			this.btnMotor3.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
			this.btnMotor3.Location = new System.Drawing.Point( 345, 226 );
			this.btnMotor3.Name = "btnMotor3";
			this.btnMotor3.Size = new System.Drawing.Size( 75, 67 );
			this.btnMotor3.TabIndex = 21;
			this.btnMotor3.Text = "3";
			this.btnMotor3.TextAlign = System.Drawing.ContentAlignment.BottomRight;
			this.btnMotor3.UseVisualStyleBackColor = true;
			// 
			// btnMotor2
			// 
			this.btnMotor2.BackgroundImage = global::Elev8.Properties.Resources.Icon_Counter_clockwise;
			this.btnMotor2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
			this.btnMotor2.Location = new System.Drawing.Point( 345, 48 );
			this.btnMotor2.Name = "btnMotor2";
			this.btnMotor2.Size = new System.Drawing.Size( 75, 67 );
			this.btnMotor2.TabIndex = 20;
			this.btnMotor2.Text = "2";
			this.btnMotor2.TextAlign = System.Drawing.ContentAlignment.TopRight;
			this.btnMotor2.UseVisualStyleBackColor = true;
			// 
			// btnMotor1
			// 
			this.btnMotor1.BackgroundImage = global::Elev8.Properties.Resources.Icon_Clockwise;
			this.btnMotor1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
			this.btnMotor1.Location = new System.Drawing.Point( 119, 48 );
			this.btnMotor1.Name = "btnMotor1";
			this.btnMotor1.Size = new System.Drawing.Size( 75, 67 );
			this.btnMotor1.TabIndex = 19;
			this.btnMotor1.Text = "1";
			this.btnMotor1.TextAlign = System.Drawing.ContentAlignment.TopLeft;
			this.btnMotor1.UseVisualStyleBackColor = true;
			// 
			// tpControlSetup
			// 
			this.tpControlSetup.BackColor = System.Drawing.SystemColors.Control;
			this.tpControlSetup.Controls.Add( this.cbChannel8 );
			this.tpControlSetup.Controls.Add( this.cbChannel7 );
			this.tpControlSetup.Controls.Add( this.cbChannel2 );
			this.tpControlSetup.Controls.Add( this.cbChannel3 );
			this.tpControlSetup.Controls.Add( this.cbChannel6 );
			this.tpControlSetup.Controls.Add( this.cbChannel5 );
			this.tpControlSetup.Controls.Add( this.cbChannel4 );
			this.tpControlSetup.Controls.Add( this.cbChannel1 );
			this.tpControlSetup.Controls.Add( this.vbR_Channel8 );
			this.tpControlSetup.Controls.Add( this.vbR_Channel7 );
			this.tpControlSetup.Controls.Add( this.vbR_Channel6 );
			this.tpControlSetup.Controls.Add( this.vbR_Channel5 );
			this.tpControlSetup.Controls.Add( this.vbR_RS_XValue );
			this.tpControlSetup.Controls.Add( this.vbR_RS_YValue );
			this.tpControlSetup.Controls.Add( this.vbR_LS_XValue );
			this.tpControlSetup.Controls.Add( this.vbR_LS_YValue );
			this.tpControlSetup.Controls.Add( this.rsR_Right );
			this.tpControlSetup.Controls.Add( this.rsR_Left );
			this.tpControlSetup.Location = new System.Drawing.Point( 4, 22 );
			this.tpControlSetup.Name = "tpControlSetup";
			this.tpControlSetup.Size = new System.Drawing.Size( 721, 356 );
			this.tpControlSetup.TabIndex = 3;
			this.tpControlSetup.Text = "Control Setup";
			// 
			// cbChannel8
			// 
			this.cbChannel8.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel8.FormattingEnabled = true;
			this.cbChannel8.Location = new System.Drawing.Point( 616, 176 );
			this.cbChannel8.Name = "cbChannel8";
			this.cbChannel8.Size = new System.Drawing.Size( 79, 21 );
			this.cbChannel8.TabIndex = 50;
			// 
			// cbChannel7
			// 
			this.cbChannel7.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel7.FormattingEnabled = true;
			this.cbChannel7.Location = new System.Drawing.Point( 616, 133 );
			this.cbChannel7.Name = "cbChannel7";
			this.cbChannel7.Size = new System.Drawing.Size( 79, 21 );
			this.cbChannel7.TabIndex = 49;
			// 
			// cbChannel2
			// 
			this.cbChannel2.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel2.FormattingEnabled = true;
			this.cbChannel2.Location = new System.Drawing.Point( 616, 90 );
			this.cbChannel2.Name = "cbChannel2";
			this.cbChannel2.Size = new System.Drawing.Size( 79, 21 );
			this.cbChannel2.TabIndex = 48;
			// 
			// cbChannel3
			// 
			this.cbChannel3.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel3.FormattingEnabled = true;
			this.cbChannel3.Location = new System.Drawing.Point( 616, 47 );
			this.cbChannel3.Name = "cbChannel3";
			this.cbChannel3.Size = new System.Drawing.Size( 79, 21 );
			this.cbChannel3.TabIndex = 47;
			// 
			// cbChannel6
			// 
			this.cbChannel6.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel6.FormattingEnabled = true;
			this.cbChannel6.Location = new System.Drawing.Point( 26, 176 );
			this.cbChannel6.Name = "cbChannel6";
			this.cbChannel6.Size = new System.Drawing.Size( 79, 21 );
			this.cbChannel6.TabIndex = 46;
			// 
			// cbChannel5
			// 
			this.cbChannel5.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel5.FormattingEnabled = true;
			this.cbChannel5.Location = new System.Drawing.Point( 26, 133 );
			this.cbChannel5.Name = "cbChannel5";
			this.cbChannel5.Size = new System.Drawing.Size( 79, 21 );
			this.cbChannel5.TabIndex = 45;
			// 
			// cbChannel4
			// 
			this.cbChannel4.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel4.FormattingEnabled = true;
			this.cbChannel4.Location = new System.Drawing.Point( 26, 90 );
			this.cbChannel4.Name = "cbChannel4";
			this.cbChannel4.Size = new System.Drawing.Size( 79, 21 );
			this.cbChannel4.TabIndex = 44;
			// 
			// cbChannel1
			// 
			this.cbChannel1.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel1.FormattingEnabled = true;
			this.cbChannel1.Location = new System.Drawing.Point( 26, 47 );
			this.cbChannel1.Name = "cbChannel1";
			this.cbChannel1.Size = new System.Drawing.Size( 79, 21 );
			this.cbChannel1.TabIndex = 43;
			this.cbChannel1.SelectedIndexChanged += new System.EventHandler( this.cbChannel1_SelectedIndexChanged );
			// 
			// vbR_Channel8
			// 
			this.vbR_Channel8.FromLeft = true;
			this.vbR_Channel8.LeftLabel = "Aux3";
			this.vbR_Channel8.Location = new System.Drawing.Point( 527, 176 );
			this.vbR_Channel8.MaxValue = 1024;
			this.vbR_Channel8.MinValue = -1024;
			this.vbR_Channel8.Name = "vbR_Channel8";
			this.vbR_Channel8.RightLabel = "0";
			this.vbR_Channel8.Size = new System.Drawing.Size( 83, 21 );
			this.vbR_Channel8.TabIndex = 42;
			this.vbR_Channel8.Value = 0;
			// 
			// vbR_Channel7
			// 
			this.vbR_Channel7.FromLeft = true;
			this.vbR_Channel7.LeftLabel = "Aux2";
			this.vbR_Channel7.Location = new System.Drawing.Point( 527, 133 );
			this.vbR_Channel7.MaxValue = 1024;
			this.vbR_Channel7.MinValue = -1024;
			this.vbR_Channel7.Name = "vbR_Channel7";
			this.vbR_Channel7.RightLabel = "0";
			this.vbR_Channel7.Size = new System.Drawing.Size( 83, 21 );
			this.vbR_Channel7.TabIndex = 41;
			this.vbR_Channel7.Value = 0;
			// 
			// vbR_Channel6
			// 
			this.vbR_Channel6.FromLeft = true;
			this.vbR_Channel6.LeftLabel = "Aux1";
			this.vbR_Channel6.Location = new System.Drawing.Point( 111, 176 );
			this.vbR_Channel6.MaxValue = 1024;
			this.vbR_Channel6.MinValue = -1024;
			this.vbR_Channel6.Name = "vbR_Channel6";
			this.vbR_Channel6.RightLabel = "0";
			this.vbR_Channel6.Size = new System.Drawing.Size( 83, 21 );
			this.vbR_Channel6.TabIndex = 40;
			this.vbR_Channel6.Value = 0;
			// 
			// vbR_Channel5
			// 
			this.vbR_Channel5.FromLeft = true;
			this.vbR_Channel5.LeftLabel = "Gear";
			this.vbR_Channel5.Location = new System.Drawing.Point( 111, 133 );
			this.vbR_Channel5.MaxValue = 1024;
			this.vbR_Channel5.MinValue = -1024;
			this.vbR_Channel5.Name = "vbR_Channel5";
			this.vbR_Channel5.RightLabel = "0";
			this.vbR_Channel5.Size = new System.Drawing.Size( 83, 21 );
			this.vbR_Channel5.TabIndex = 39;
			this.vbR_Channel5.Value = 0;
			// 
			// vbR_RS_XValue
			// 
			this.vbR_RS_XValue.FromLeft = true;
			this.vbR_RS_XValue.LeftLabel = "Aileron";
			this.vbR_RS_XValue.Location = new System.Drawing.Point( 527, 90 );
			this.vbR_RS_XValue.MaxValue = 1024;
			this.vbR_RS_XValue.MinValue = -1024;
			this.vbR_RS_XValue.Name = "vbR_RS_XValue";
			this.vbR_RS_XValue.RightLabel = "0";
			this.vbR_RS_XValue.Size = new System.Drawing.Size( 83, 21 );
			this.vbR_RS_XValue.TabIndex = 38;
			this.vbR_RS_XValue.Value = 0;
			// 
			// vbR_RS_YValue
			// 
			this.vbR_RS_YValue.FromLeft = true;
			this.vbR_RS_YValue.LeftLabel = "Elevator";
			this.vbR_RS_YValue.Location = new System.Drawing.Point( 527, 47 );
			this.vbR_RS_YValue.MaxValue = 1024;
			this.vbR_RS_YValue.MinValue = -1024;
			this.vbR_RS_YValue.Name = "vbR_RS_YValue";
			this.vbR_RS_YValue.RightLabel = "0";
			this.vbR_RS_YValue.Size = new System.Drawing.Size( 83, 21 );
			this.vbR_RS_YValue.TabIndex = 37;
			this.vbR_RS_YValue.Value = 0;
			// 
			// vbR_LS_XValue
			// 
			this.vbR_LS_XValue.FromLeft = true;
			this.vbR_LS_XValue.LeftLabel = "Rudder";
			this.vbR_LS_XValue.Location = new System.Drawing.Point( 111, 90 );
			this.vbR_LS_XValue.MaxValue = 1024;
			this.vbR_LS_XValue.MinValue = -1024;
			this.vbR_LS_XValue.Name = "vbR_LS_XValue";
			this.vbR_LS_XValue.RightLabel = "0";
			this.vbR_LS_XValue.Size = new System.Drawing.Size( 83, 21 );
			this.vbR_LS_XValue.TabIndex = 36;
			this.vbR_LS_XValue.Value = 0;
			// 
			// vbR_LS_YValue
			// 
			this.vbR_LS_YValue.FromLeft = true;
			this.vbR_LS_YValue.LeftLabel = "Throttle";
			this.vbR_LS_YValue.Location = new System.Drawing.Point( 111, 47 );
			this.vbR_LS_YValue.MaxValue = 1024;
			this.vbR_LS_YValue.MinValue = -1024;
			this.vbR_LS_YValue.Name = "vbR_LS_YValue";
			this.vbR_LS_YValue.RightLabel = "0";
			this.vbR_LS_YValue.Size = new System.Drawing.Size( 83, 21 );
			this.vbR_LS_YValue.TabIndex = 35;
			this.vbR_LS_YValue.Value = 0;
			// 
			// rsR_Right
			// 
			this.rsR_Right.Location = new System.Drawing.Point( 367, 47 );
			this.rsR_Right.Name = "rsR_Right";
			this.rsR_Right.Size = new System.Drawing.Size( 150, 150 );
			this.rsR_Right.TabIndex = 34;
			this.rsR_Right.Text = "radioStick2";
			// 
			// rsR_Left
			// 
			this.rsR_Left.Location = new System.Drawing.Point( 204, 47 );
			this.rsR_Left.Name = "rsR_Left";
			this.rsR_Left.Size = new System.Drawing.Size( 150, 150 );
			this.rsR_Left.TabIndex = 33;
			this.rsR_Left.Text = "radioStick1";
			// 
			// tpSystemSetup
			// 
			this.tpSystemSetup.Location = new System.Drawing.Point( 4, 22 );
			this.tpSystemSetup.Name = "tpSystemSetup";
			this.tpSystemSetup.Size = new System.Drawing.Size( 721, 356 );
			this.tpSystemSetup.TabIndex = 4;
			this.tpSystemSetup.Text = "System Setup";
			// 
			// msMainMenu
			// 
			this.msMainMenu.Items.AddRange( new System.Windows.Forms.ToolStripItem[] {
            this.settingsToolStripMenuItem} );
			this.msMainMenu.Location = new System.Drawing.Point( 0, 0 );
			this.msMainMenu.Name = "msMainMenu";
			this.msMainMenu.Size = new System.Drawing.Size( 729, 24 );
			this.msMainMenu.TabIndex = 8;
			this.msMainMenu.Text = "menuStrip1";
			// 
			// settingsToolStripMenuItem
			// 
			this.settingsToolStripMenuItem.DropDownItems.AddRange( new System.Windows.Forms.ToolStripItem[] {
            this.radioDisplayToolStripMenuItem,
            this.toolStripMenuItem2,
            this.miRadioMode1,
            this.miRadioMode2,
            this.toolStripMenuItem1} );
			this.settingsToolStripMenuItem.Name = "settingsToolStripMenuItem";
			this.settingsToolStripMenuItem.Size = new System.Drawing.Size( 61, 20 );
			this.settingsToolStripMenuItem.Text = "Settings";
			// 
			// radioDisplayToolStripMenuItem
			// 
			this.radioDisplayToolStripMenuItem.Enabled = false;
			this.radioDisplayToolStripMenuItem.Name = "radioDisplayToolStripMenuItem";
			this.radioDisplayToolStripMenuItem.Size = new System.Drawing.Size( 147, 22 );
			this.radioDisplayToolStripMenuItem.Text = "Radio Display";
			// 
			// toolStripMenuItem2
			// 
			this.toolStripMenuItem2.Name = "toolStripMenuItem2";
			this.toolStripMenuItem2.Size = new System.Drawing.Size( 144, 6 );
			// 
			// miRadioMode1
			// 
			this.miRadioMode1.Name = "miRadioMode1";
			this.miRadioMode1.Size = new System.Drawing.Size( 147, 22 );
			this.miRadioMode1.Text = "Radio Mode 1";
			this.miRadioMode1.Click += new System.EventHandler( this.miRadioMode1_Click );
			// 
			// miRadioMode2
			// 
			this.miRadioMode2.Checked = true;
			this.miRadioMode2.CheckState = System.Windows.Forms.CheckState.Checked;
			this.miRadioMode2.Name = "miRadioMode2";
			this.miRadioMode2.Size = new System.Drawing.Size( 147, 22 );
			this.miRadioMode2.Text = "Radio Mode 2";
			this.miRadioMode2.Click += new System.EventHandler( this.miRadioMode2_Click );
			// 
			// toolStripMenuItem1
			// 
			this.toolStripMenuItem1.Name = "toolStripMenuItem1";
			this.toolStripMenuItem1.Size = new System.Drawing.Size( 144, 6 );
			// 
			// MainForm
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF( 6F, 13F );
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size( 729, 428 );
			this.Controls.Add( this.tcTabs );
			this.Controls.Add( this.stMainStatus );
			this.Controls.Add( this.msMainMenu );
			this.MainMenuStrip = this.msMainMenu;
			this.Name = "MainForm";
			this.Text = "Elev8 Ground Station";
			this.FormClosing += new System.Windows.Forms.FormClosingEventHandler( this.MainForm_FormClosing );
			this.stMainStatus.ResumeLayout( false );
			this.stMainStatus.PerformLayout();
			this.tcTabs.ResumeLayout( false );
			this.tpStatus.ResumeLayout( false );
			this.tpSensors.ResumeLayout( false );
			this.tpSensors.PerformLayout();
			this.tpSysTest.ResumeLayout( false );
			this.tpSysTest.PerformLayout();
			this.tpControlSetup.ResumeLayout( false );
			this.msMainMenu.ResumeLayout( false );
			this.msMainMenu.PerformLayout();
			this.ResumeLayout( false );
			this.PerformLayout();

		}

		#endregion

		private System.Windows.Forms.StatusStrip stMainStatus;
		private System.Windows.Forms.ToolStripStatusLabel tssStatusText;
		private System.Windows.Forms.Timer tmCommTimer;
		private System.Windows.Forms.TabControl tcTabs;
		private System.Windows.Forms.TabPage tpSensors;
		private System.Windows.Forms.TabPage tpStatus;
		private RadioStick rsRight;
		private RadioStick rsLeft;
		private AttitudeIndicator aicAttitude;
		private OrientationCube ocOrientation;
		private HeadingIndicator aicHeading;
		private Altimeter aicAltimeter;
		private System.Windows.Forms.MenuStrip msMainMenu;
		private System.Windows.Forms.ToolStripMenuItem settingsToolStripMenuItem;
		private System.Windows.Forms.ToolStripMenuItem radioDisplayToolStripMenuItem;
		private System.Windows.Forms.ToolStripSeparator toolStripMenuItem2;
		private System.Windows.Forms.ToolStripMenuItem miRadioMode1;
		private System.Windows.Forms.ToolStripMenuItem miRadioMode2;
		private System.Windows.Forms.ToolStripSeparator toolStripMenuItem1;
		private Elev8.Controls.ValueBar vbLS_YValue;
		private Elev8.Controls.ValueBar vbLS_XValue;
		private Elev8.Controls.ValueBar vbRS_XValue;
		private Elev8.Controls.ValueBar vbRS_YValue;
		private Elev8.Controls.ValueBar vbChannel8;
		private Elev8.Controls.ValueBar vbChannel7;
		private Elev8.Controls.ValueBar vbChannel6;
		private Elev8.Controls.ValueBar vbChannel5;
		private System.Windows.Forms.CheckBox cbAccelZ;
		private System.Windows.Forms.CheckBox cbAccelY;
		private System.Windows.Forms.CheckBox cbAccelX;
		private System.Windows.Forms.CheckBox cbGyroZ;
		private System.Windows.Forms.CheckBox cbGyroY;
		private System.Windows.Forms.CheckBox cbGyroX;
		private System.Windows.Forms.CheckBox cbMagZ;
		private System.Windows.Forms.CheckBox cbMagY;
		private System.Windows.Forms.CheckBox cbMagX;
		private System.Windows.Forms.CheckBox cbVoltage;
		private System.Windows.Forms.CheckBox cbYaw;
		private System.Windows.Forms.CheckBox cbRoll;
		private System.Windows.Forms.CheckBox cbPitch;
		private System.Windows.Forms.CheckBox cbAltitude;
		private System.Windows.Forms.TabPage tpSysTest;
		private System.Windows.Forms.Label lblCalibrateDocs;
		private System.Windows.Forms.Button btnThrottleCalibrate;
		private System.Windows.Forms.Button btnLED;
		private System.Windows.Forms.Button btnBeeper;
		private System.Windows.Forms.TextBox textBox3;
		private System.Windows.Forms.Button btnMotor4;
		private System.Windows.Forms.Button btnMotor3;
		private System.Windows.Forms.Button btnMotor2;
		private System.Windows.Forms.Button btnMotor1;
		private System.Windows.Forms.TabPage tpControlSetup;
		private System.Windows.Forms.TabPage tpSystemSetup;
		private Elev8.Controls.ValueBar vbR_Channel8;
		private Elev8.Controls.ValueBar vbR_Channel7;
		private Elev8.Controls.ValueBar vbR_Channel6;
		private Elev8.Controls.ValueBar vbR_Channel5;
		private Elev8.Controls.ValueBar vbR_RS_XValue;
		private Elev8.Controls.ValueBar vbR_RS_YValue;
		private Elev8.Controls.ValueBar vbR_LS_XValue;
		private Elev8.Controls.ValueBar vbR_LS_YValue;
		private RadioStick rsR_Right;
		private RadioStick rsR_Left;
		private System.Windows.Forms.ComboBox cbChannel8;
		private System.Windows.Forms.ComboBox cbChannel7;
		private System.Windows.Forms.ComboBox cbChannel2;
		private System.Windows.Forms.ComboBox cbChannel3;
		private System.Windows.Forms.ComboBox cbChannel6;
		private System.Windows.Forms.ComboBox cbChannel5;
		private System.Windows.Forms.ComboBox cbChannel4;
		private System.Windows.Forms.ComboBox cbChannel1;
	}
}

