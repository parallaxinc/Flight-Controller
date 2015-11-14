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
			this.lblAccelCorrectionFilter = new System.Windows.Forms.Label();
			this.label13 = new System.Windows.Forms.Label();
			this.tbAccelCorrectionFilter = new System.Windows.Forms.TrackBar();
			this.lblThrustCorrection = new System.Windows.Forms.Label();
			this.label17 = new System.Windows.Forms.Label();
			this.tbThrustCorrection = new System.Windows.Forms.TrackBar();
			this.btnControlReset = new System.Windows.Forms.Button();
			this.tbCalibrateDocs = new System.Windows.Forms.TextBox();
			this.lblYawSpeed = new System.Windows.Forms.Label();
			this.label5 = new System.Windows.Forms.Label();
			this.tbYawSpeed = new System.Windows.Forms.TrackBar();
			this.btnUploadRollPitch = new System.Windows.Forms.Button();
			this.lblRollPitchSpeed = new System.Windows.Forms.Label();
			this.lblRollPitchAngle = new System.Windows.Forms.Label();
			this.label3 = new System.Windows.Forms.Label();
			this.tbRollPitchSpeed = new System.Windows.Forms.TrackBar();
			this.label2 = new System.Windows.Forms.Label();
			this.tbRollPitchAngle = new System.Windows.Forms.TrackBar();
			this.btnCalibrate = new System.Windows.Forms.Button();
			this.label1 = new System.Windows.Forms.Label();
			this.cbReceiverType = new System.Windows.Forms.ComboBox();
			this.cbRev8 = new System.Windows.Forms.CheckBox();
			this.cbRev7 = new System.Windows.Forms.CheckBox();
			this.cbRev2 = new System.Windows.Forms.CheckBox();
			this.cbRev3 = new System.Windows.Forms.CheckBox();
			this.cbRev6 = new System.Windows.Forms.CheckBox();
			this.cbRev5 = new System.Windows.Forms.CheckBox();
			this.cbRev4 = new System.Windows.Forms.CheckBox();
			this.cbRev1 = new System.Windows.Forms.CheckBox();
			this.cbChannel8 = new System.Windows.Forms.ComboBox();
			this.cbChannel7 = new System.Windows.Forms.ComboBox();
			this.cbChannel2 = new System.Windows.Forms.ComboBox();
			this.cbChannel3 = new System.Windows.Forms.ComboBox();
			this.cbChannel6 = new System.Windows.Forms.ComboBox();
			this.cbChannel5 = new System.Windows.Forms.ComboBox();
			this.cbChannel4 = new System.Windows.Forms.ComboBox();
			this.cbChannel1 = new System.Windows.Forms.ComboBox();
			this.tpSystemSetup = new System.Windows.Forms.TabPage();
			this.cbLowVoltageBuzzer = new System.Windows.Forms.CheckBox();
			this.label16 = new System.Windows.Forms.Label();
			this.cbArmingDelay = new System.Windows.Forms.ComboBox();
			this.label15 = new System.Windows.Forms.Label();
			this.cbDisarmDelay = new System.Windows.Forms.ComboBox();
			this.label10 = new System.Windows.Forms.Label();
			this.udVoltageOffset = new System.Windows.Forms.NumericUpDown();
			this.label12 = new System.Windows.Forms.Label();
			this.label9 = new System.Windows.Forms.Label();
			this.btnUploadThrottle = new System.Windows.Forms.Button();
			this.udLowVoltageAlarm = new System.Windows.Forms.NumericUpDown();
			this.cbUseBatteryMonitor = new System.Windows.Forms.CheckBox();
			this.groupBox1 = new System.Windows.Forms.GroupBox();
			this.label6 = new System.Windows.Forms.Label();
			this.label4 = new System.Windows.Forms.Label();
			this.udTestThrottle = new System.Windows.Forms.NumericUpDown();
			this.label7 = new System.Windows.Forms.Label();
			this.label8 = new System.Windows.Forms.Label();
			this.udLowThrottle = new System.Windows.Forms.NumericUpDown();
			this.udHighThrottle = new System.Windows.Forms.NumericUpDown();
			this.udArmedLowThrottle = new System.Windows.Forms.NumericUpDown();
			this.tpGyroCalibration = new System.Windows.Forms.TabPage();
			this.btnUploadGyroCalibration = new System.Windows.Forms.Button();
			this.btnResetGyroCal = new System.Windows.Forms.Button();
			this.groupBox10 = new System.Windows.Forms.GroupBox();
			this.label18 = new System.Windows.Forms.Label();
			this.gzOffset = new System.Windows.Forms.Label();
			this.label20 = new System.Windows.Forms.Label();
			this.gzScale = new System.Windows.Forms.Label();
			this.groupBox11 = new System.Windows.Forms.GroupBox();
			this.label22 = new System.Windows.Forms.Label();
			this.gyOffset = new System.Windows.Forms.Label();
			this.label24 = new System.Windows.Forms.Label();
			this.gyScale = new System.Windows.Forms.Label();
			this.groupBox12 = new System.Windows.Forms.GroupBox();
			this.label26 = new System.Windows.Forms.Label();
			this.gxOffset = new System.Windows.Forms.Label();
			this.label28 = new System.Windows.Forms.Label();
			this.gxScale = new System.Windows.Forms.Label();
			this.tpAccelCalibration = new System.Windows.Forms.TabPage();
			this.groupBox6 = new System.Windows.Forms.GroupBox();
			this.btnUploadAngleCorrection = new System.Windows.Forms.Button();
			this.udRollCorrection = new System.Windows.Forms.NumericUpDown();
			this.label41 = new System.Windows.Forms.Label();
			this.label42 = new System.Windows.Forms.Label();
			this.udPitchCorrection = new System.Windows.Forms.NumericUpDown();
			this.groupBox5 = new System.Windows.Forms.GroupBox();
			this.label39 = new System.Windows.Forms.Label();
			this.label38 = new System.Windows.Forms.Label();
			this.label37 = new System.Windows.Forms.Label();
			this.lblAccelCalFinal = new System.Windows.Forms.Label();
			this.btnAccelCal1 = new System.Windows.Forms.Button();
			this.lblAccelCal4 = new System.Windows.Forms.Label();
			this.btnAccelCal2 = new System.Windows.Forms.Button();
			this.lblAccelCal3 = new System.Windows.Forms.Label();
			this.btnAccelCal3 = new System.Windows.Forms.Button();
			this.lblAccelCal2 = new System.Windows.Forms.Label();
			this.btnAccelCal4 = new System.Windows.Forms.Button();
			this.lblAccelCal1 = new System.Windows.Forms.Label();
			this.btnUploadAccelCal = new System.Windows.Forms.Button();
			this.msMainMenu = new System.Windows.Forms.MenuStrip();
			this.settingsToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
			this.radioDisplayToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
			this.toolStripMenuItem2 = new System.Windows.Forms.ToolStripSeparator();
			this.miRadioMode1 = new System.Windows.Forms.ToolStripMenuItem();
			this.miRadioMode2 = new System.Windows.Forms.ToolStripMenuItem();
			this.toolStripMenuItem1 = new System.Windows.Forms.ToolStripSeparator();
			this.checkBox4 = new System.Windows.Forms.CheckBox();
			this.checkBox5 = new System.Windows.Forms.CheckBox();
			this.checkBox6 = new System.Windows.Forms.CheckBox();
			this.radioButton1 = new System.Windows.Forms.RadioButton();
			this.radioButton2 = new System.Windows.Forms.RadioButton();
			this.comboBox1 = new System.Windows.Forms.ComboBox();
			this.comboBox2 = new System.Windows.Forms.ComboBox();
			this.comboBox3 = new System.Windows.Forms.ComboBox();
			this.comboBox4 = new System.Windows.Forms.ComboBox();
			this.comboBox5 = new System.Windows.Forms.ComboBox();
			this.comboBox6 = new System.Windows.Forms.ComboBox();
			this.comboBox7 = new System.Windows.Forms.ComboBox();
			this.comboBox8 = new System.Windows.Forms.ComboBox();
			this.tlToolTip = new System.Windows.Forms.ToolTip( this.components );
			this.vbVoltage = new Elev8.Controls.ValueBar();
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
			this.plotSensors = new GraphLib.PlotterDisplayEx();
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
			this.vbVoltage2 = new Elev8.Controls.ValueBar();
			this.gCalibTemp = new Elev8.Gauge();
			this.gCalibZ = new Elev8.Gauge();
			this.gCalibY = new Elev8.Gauge();
			this.gCalibX = new Elev8.Gauge();
			this.lfGraph = new Elev8.LineFit();
			this.ocAccelOrient = new Elev8.OrientationCube();
			this.gAccelXCal = new Elev8.Gauge();
			this.gAccelYCal = new Elev8.Gauge();
			this.gAccelZCal = new Elev8.Gauge();
			this.valueBar1 = new Elev8.Controls.ValueBar();
			this.valueBar2 = new Elev8.Controls.ValueBar();
			this.valueBar3 = new Elev8.Controls.ValueBar();
			this.valueBar4 = new Elev8.Controls.ValueBar();
			this.valueBar5 = new Elev8.Controls.ValueBar();
			this.valueBar6 = new Elev8.Controls.ValueBar();
			this.valueBar7 = new Elev8.Controls.ValueBar();
			this.valueBar8 = new Elev8.Controls.ValueBar();
			this.radioStick1 = new Elev8.RadioStick();
			this.radioStick2 = new Elev8.RadioStick();
			this.stMainStatus.SuspendLayout();
			this.tcTabs.SuspendLayout();
			this.tpStatus.SuspendLayout();
			this.tpSensors.SuspendLayout();
			this.tpSysTest.SuspendLayout();
			this.tpControlSetup.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)(this.tbAccelCorrectionFilter)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbThrustCorrection)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbYawSpeed)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbRollPitchSpeed)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbRollPitchAngle)).BeginInit();
			this.tpSystemSetup.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)(this.udVoltageOffset)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udLowVoltageAlarm)).BeginInit();
			this.groupBox1.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)(this.udTestThrottle)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udLowThrottle)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udHighThrottle)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udArmedLowThrottle)).BeginInit();
			this.tpGyroCalibration.SuspendLayout();
			this.groupBox10.SuspendLayout();
			this.groupBox11.SuspendLayout();
			this.groupBox12.SuspendLayout();
			this.tpAccelCalibration.SuspendLayout();
			this.groupBox6.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)(this.udRollCorrection)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udPitchCorrection)).BeginInit();
			this.groupBox5.SuspendLayout();
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
			this.tcTabs.Controls.Add( this.tpGyroCalibration );
			this.tcTabs.Controls.Add( this.tpAccelCalibration );
			this.tcTabs.Dock = System.Windows.Forms.DockStyle.Fill;
			this.tcTabs.Location = new System.Drawing.Point( 0, 24 );
			this.tcTabs.Name = "tcTabs";
			this.tcTabs.SelectedIndex = 0;
			this.tcTabs.Size = new System.Drawing.Size( 729, 382 );
			this.tcTabs.TabIndex = 0;
			this.tcTabs.SelectedIndexChanged += new System.EventHandler( this.tcTabs_SelectedIndexChanged );
			// 
			// tpStatus
			// 
			this.tpStatus.BackColor = System.Drawing.SystemColors.Control;
			this.tpStatus.Controls.Add( this.vbVoltage );
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
			// tpSensors
			// 
			this.tpSensors.BackColor = System.Drawing.Color.Transparent;
			this.tpSensors.Controls.Add( this.plotSensors );
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
			this.cbVoltage.Tag = "14";
			this.cbVoltage.Text = "Voltage";
			this.cbVoltage.UseVisualStyleBackColor = true;
			this.cbVoltage.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbYaw
			// 
			this.cbYaw.AutoSize = true;
			this.cbYaw.Location = new System.Drawing.Point( 8, 268 );
			this.cbYaw.Name = "cbYaw";
			this.cbYaw.Size = new System.Drawing.Size( 47, 17 );
			this.cbYaw.TabIndex = 12;
			this.cbYaw.Tag = "13";
			this.cbYaw.Text = "Yaw";
			this.cbYaw.UseVisualStyleBackColor = true;
			this.cbYaw.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbRoll
			// 
			this.cbRoll.AutoSize = true;
			this.cbRoll.Location = new System.Drawing.Point( 8, 248 );
			this.cbRoll.Name = "cbRoll";
			this.cbRoll.Size = new System.Drawing.Size( 44, 17 );
			this.cbRoll.TabIndex = 11;
			this.cbRoll.Tag = "12";
			this.cbRoll.Text = "Roll";
			this.cbRoll.UseVisualStyleBackColor = true;
			this.cbRoll.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbPitch
			// 
			this.cbPitch.AutoSize = true;
			this.cbPitch.Location = new System.Drawing.Point( 8, 228 );
			this.cbPitch.Name = "cbPitch";
			this.cbPitch.Size = new System.Drawing.Size( 50, 17 );
			this.cbPitch.TabIndex = 10;
			this.cbPitch.Tag = "11";
			this.cbPitch.Text = "Pitch";
			this.cbPitch.UseVisualStyleBackColor = true;
			this.cbPitch.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbAltitude
			// 
			this.cbAltitude.AutoSize = true;
			this.cbAltitude.Location = new System.Drawing.Point( 8, 208 );
			this.cbAltitude.Name = "cbAltitude";
			this.cbAltitude.Size = new System.Drawing.Size( 61, 17 );
			this.cbAltitude.TabIndex = 9;
			this.cbAltitude.Tag = "10";
			this.cbAltitude.Text = "Altitude";
			this.cbAltitude.UseVisualStyleBackColor = true;
			this.cbAltitude.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbMagZ
			// 
			this.cbMagZ.AutoSize = true;
			this.cbMagZ.Location = new System.Drawing.Point( 8, 176 );
			this.cbMagZ.Name = "cbMagZ";
			this.cbMagZ.Size = new System.Drawing.Size( 57, 17 );
			this.cbMagZ.TabIndex = 8;
			this.cbMagZ.Tag = "9";
			this.cbMagZ.Text = "Mag Z";
			this.cbMagZ.UseVisualStyleBackColor = true;
			this.cbMagZ.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbMagY
			// 
			this.cbMagY.AutoSize = true;
			this.cbMagY.Location = new System.Drawing.Point( 8, 156 );
			this.cbMagY.Name = "cbMagY";
			this.cbMagY.Size = new System.Drawing.Size( 57, 17 );
			this.cbMagY.TabIndex = 7;
			this.cbMagY.Tag = "8";
			this.cbMagY.Text = "Mag Y";
			this.cbMagY.UseVisualStyleBackColor = true;
			this.cbMagY.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbMagX
			// 
			this.cbMagX.AutoSize = true;
			this.cbMagX.Location = new System.Drawing.Point( 8, 136 );
			this.cbMagX.Name = "cbMagX";
			this.cbMagX.Size = new System.Drawing.Size( 57, 17 );
			this.cbMagX.TabIndex = 6;
			this.cbMagX.Tag = "7";
			this.cbMagX.Text = "Mag X";
			this.cbMagX.UseVisualStyleBackColor = true;
			this.cbMagX.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbAccelZ
			// 
			this.cbAccelZ.AutoSize = true;
			this.cbAccelZ.Location = new System.Drawing.Point( 8, 116 );
			this.cbAccelZ.Name = "cbAccelZ";
			this.cbAccelZ.Size = new System.Drawing.Size( 63, 17 );
			this.cbAccelZ.TabIndex = 5;
			this.cbAccelZ.Tag = "6";
			this.cbAccelZ.Text = "Accel Z";
			this.cbAccelZ.UseVisualStyleBackColor = true;
			this.cbAccelZ.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbAccelY
			// 
			this.cbAccelY.AutoSize = true;
			this.cbAccelY.Location = new System.Drawing.Point( 8, 96 );
			this.cbAccelY.Name = "cbAccelY";
			this.cbAccelY.Size = new System.Drawing.Size( 63, 17 );
			this.cbAccelY.TabIndex = 4;
			this.cbAccelY.Tag = "5";
			this.cbAccelY.Text = "Accel Y";
			this.cbAccelY.UseVisualStyleBackColor = true;
			this.cbAccelY.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbAccelX
			// 
			this.cbAccelX.AutoSize = true;
			this.cbAccelX.Location = new System.Drawing.Point( 8, 76 );
			this.cbAccelX.Name = "cbAccelX";
			this.cbAccelX.Size = new System.Drawing.Size( 63, 17 );
			this.cbAccelX.TabIndex = 3;
			this.cbAccelX.Tag = "4";
			this.cbAccelX.Text = "Accel X";
			this.cbAccelX.UseVisualStyleBackColor = true;
			this.cbAccelX.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbGyroZ
			// 
			this.cbGyroZ.AutoSize = true;
			this.cbGyroZ.Location = new System.Drawing.Point( 8, 56 );
			this.cbGyroZ.Name = "cbGyroZ";
			this.cbGyroZ.Size = new System.Drawing.Size( 58, 17 );
			this.cbGyroZ.TabIndex = 2;
			this.cbGyroZ.Tag = "3";
			this.cbGyroZ.Text = "Gyro Z";
			this.cbGyroZ.UseVisualStyleBackColor = true;
			this.cbGyroZ.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbGyroY
			// 
			this.cbGyroY.AutoSize = true;
			this.cbGyroY.Location = new System.Drawing.Point( 8, 36 );
			this.cbGyroY.Name = "cbGyroY";
			this.cbGyroY.Size = new System.Drawing.Size( 58, 17 );
			this.cbGyroY.TabIndex = 1;
			this.cbGyroY.Tag = "2";
			this.cbGyroY.Text = "Gyro Y";
			this.cbGyroY.UseVisualStyleBackColor = true;
			this.cbGyroY.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbGyroX
			// 
			this.cbGyroX.AutoSize = true;
			this.cbGyroX.Location = new System.Drawing.Point( 8, 16 );
			this.cbGyroX.Name = "cbGyroX";
			this.cbGyroX.Size = new System.Drawing.Size( 58, 17 );
			this.cbGyroX.TabIndex = 0;
			this.cbGyroX.Tag = "1";
			this.cbGyroX.Text = "Gyro X";
			this.cbGyroX.UseVisualStyleBackColor = true;
			this.cbGyroX.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// tpSysTest
			// 
			this.tpSysTest.BackColor = System.Drawing.Color.Transparent;
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
			this.btnThrottleCalibrate.Click += new System.EventHandler( this.btnThrottleCalibrate_Click );
			// 
			// btnLED
			// 
			this.btnLED.Location = new System.Drawing.Point( 232, 178 );
			this.btnLED.Name = "btnLED";
			this.btnLED.Size = new System.Drawing.Size( 75, 33 );
			this.btnLED.TabIndex = 24;
			this.btnLED.Text = "LED";
			this.btnLED.UseVisualStyleBackColor = true;
			this.btnLED.Click += new System.EventHandler( this.btnLED_Click );
			// 
			// btnBeeper
			// 
			this.btnBeeper.Location = new System.Drawing.Point( 232, 130 );
			this.btnBeeper.Name = "btnBeeper";
			this.btnBeeper.Size = new System.Drawing.Size( 75, 33 );
			this.btnBeeper.TabIndex = 23;
			this.btnBeeper.Text = "Beeper";
			this.btnBeeper.UseVisualStyleBackColor = true;
			this.btnBeeper.Click += new System.EventHandler( this.btnBeeper_Click );
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
			this.btnMotor4.MouseDown += new System.Windows.Forms.MouseEventHandler( this.btnMotor4_MouseDown );
			this.btnMotor4.MouseUp += new System.Windows.Forms.MouseEventHandler( this.btnMotor_MouseUp );
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
			this.btnMotor3.MouseDown += new System.Windows.Forms.MouseEventHandler( this.btnMotor3_MouseDown );
			this.btnMotor3.MouseUp += new System.Windows.Forms.MouseEventHandler( this.btnMotor_MouseUp );
			// 
			// btnMotor2
			// 
			this.btnMotor2.BackgroundImage = global::Elev8.Properties.Resources.Icon_Counter_clockwise;
			this.btnMotor2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
			this.btnMotor2.Location = new System.Drawing.Point( 345, 68 );
			this.btnMotor2.Name = "btnMotor2";
			this.btnMotor2.Size = new System.Drawing.Size( 75, 67 );
			this.btnMotor2.TabIndex = 20;
			this.btnMotor2.Text = "2";
			this.btnMotor2.TextAlign = System.Drawing.ContentAlignment.TopRight;
			this.btnMotor2.UseVisualStyleBackColor = true;
			this.btnMotor2.MouseDown += new System.Windows.Forms.MouseEventHandler( this.btnMotor2_MouseDown );
			this.btnMotor2.MouseUp += new System.Windows.Forms.MouseEventHandler( this.btnMotor_MouseUp );
			// 
			// btnMotor1
			// 
			this.btnMotor1.BackgroundImage = global::Elev8.Properties.Resources.Icon_Clockwise;
			this.btnMotor1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
			this.btnMotor1.Location = new System.Drawing.Point( 119, 68 );
			this.btnMotor1.Name = "btnMotor1";
			this.btnMotor1.Size = new System.Drawing.Size( 75, 67 );
			this.btnMotor1.TabIndex = 19;
			this.btnMotor1.Text = "1";
			this.btnMotor1.TextAlign = System.Drawing.ContentAlignment.TopLeft;
			this.btnMotor1.UseVisualStyleBackColor = true;
			this.btnMotor1.MouseDown += new System.Windows.Forms.MouseEventHandler( this.btnMotor1_MouseDown );
			this.btnMotor1.MouseUp += new System.Windows.Forms.MouseEventHandler( this.btnMotor_MouseUp );
			// 
			// tpControlSetup
			// 
			this.tpControlSetup.BackColor = System.Drawing.SystemColors.Control;
			this.tpControlSetup.Controls.Add( this.lblAccelCorrectionFilter );
			this.tpControlSetup.Controls.Add( this.label13 );
			this.tpControlSetup.Controls.Add( this.tbAccelCorrectionFilter );
			this.tpControlSetup.Controls.Add( this.lblThrustCorrection );
			this.tpControlSetup.Controls.Add( this.label17 );
			this.tpControlSetup.Controls.Add( this.tbThrustCorrection );
			this.tpControlSetup.Controls.Add( this.btnControlReset );
			this.tpControlSetup.Controls.Add( this.tbCalibrateDocs );
			this.tpControlSetup.Controls.Add( this.lblYawSpeed );
			this.tpControlSetup.Controls.Add( this.label5 );
			this.tpControlSetup.Controls.Add( this.tbYawSpeed );
			this.tpControlSetup.Controls.Add( this.btnUploadRollPitch );
			this.tpControlSetup.Controls.Add( this.lblRollPitchSpeed );
			this.tpControlSetup.Controls.Add( this.lblRollPitchAngle );
			this.tpControlSetup.Controls.Add( this.label3 );
			this.tpControlSetup.Controls.Add( this.tbRollPitchSpeed );
			this.tpControlSetup.Controls.Add( this.label2 );
			this.tpControlSetup.Controls.Add( this.tbRollPitchAngle );
			this.tpControlSetup.Controls.Add( this.btnCalibrate );
			this.tpControlSetup.Controls.Add( this.label1 );
			this.tpControlSetup.Controls.Add( this.cbReceiverType );
			this.tpControlSetup.Controls.Add( this.cbRev8 );
			this.tpControlSetup.Controls.Add( this.cbRev7 );
			this.tpControlSetup.Controls.Add( this.cbRev2 );
			this.tpControlSetup.Controls.Add( this.cbRev3 );
			this.tpControlSetup.Controls.Add( this.cbRev6 );
			this.tpControlSetup.Controls.Add( this.cbRev5 );
			this.tpControlSetup.Controls.Add( this.cbRev4 );
			this.tpControlSetup.Controls.Add( this.cbRev1 );
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
			// lblAccelCorrectionFilter
			// 
			this.lblAccelCorrectionFilter.AutoSize = true;
			this.lblAccelCorrectionFilter.Location = new System.Drawing.Point( 678, 252 );
			this.lblAccelCorrectionFilter.Name = "lblAccelCorrectionFilter";
			this.lblAccelCorrectionFilter.Size = new System.Drawing.Size( 34, 13 );
			this.lblAccelCorrectionFilter.TabIndex = 82;
			this.lblAccelCorrectionFilter.Text = "0.062";
			// 
			// label13
			// 
			this.label13.Location = new System.Drawing.Point( 496, 236 );
			this.label13.Name = "label13";
			this.label13.Size = new System.Drawing.Size( 73, 45 );
			this.label13.TabIndex = 81;
			this.label13.Text = "Vertical Disturbance Correction";
			this.label13.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// tbAccelCorrectionFilter
			// 
			this.tbAccelCorrectionFilter.LargeChange = 16;
			this.tbAccelCorrectionFilter.Location = new System.Drawing.Point( 575, 236 );
			this.tbAccelCorrectionFilter.Maximum = 256;
			this.tbAccelCorrectionFilter.Name = "tbAccelCorrectionFilter";
			this.tbAccelCorrectionFilter.Size = new System.Drawing.Size( 97, 45 );
			this.tbAccelCorrectionFilter.TabIndex = 80;
			this.tbAccelCorrectionFilter.TickFrequency = 16;
			this.tlToolTip.SetToolTip( this.tbAccelCorrectionFilter, "Smaller numbers mean vertical disturbance correction is softer.  (0 is disabled)" );
			this.tbAccelCorrectionFilter.Value = 16;
			this.tbAccelCorrectionFilter.Scroll += new System.EventHandler( this.tbAccelCorrectionFilter_Scroll );
			// 
			// lblThrustCorrection
			// 
			this.lblThrustCorrection.AutoSize = true;
			this.lblThrustCorrection.Location = new System.Drawing.Point( 678, 312 );
			this.lblThrustCorrection.Name = "lblThrustCorrection";
			this.lblThrustCorrection.Size = new System.Drawing.Size( 34, 13 );
			this.lblThrustCorrection.TabIndex = 79;
			this.lblThrustCorrection.Text = "1.000";
			// 
			// label17
			// 
			this.label17.Location = new System.Drawing.Point( 496, 296 );
			this.label17.Name = "label17";
			this.label17.Size = new System.Drawing.Size( 73, 45 );
			this.label17.TabIndex = 78;
			this.label17.Text = "Thrust Angle Correction";
			this.label17.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// tbThrustCorrection
			// 
			this.tbThrustCorrection.LargeChange = 32;
			this.tbThrustCorrection.Location = new System.Drawing.Point( 575, 296 );
			this.tbThrustCorrection.Maximum = 512;
			this.tbThrustCorrection.Name = "tbThrustCorrection";
			this.tbThrustCorrection.Size = new System.Drawing.Size( 97, 45 );
			this.tbThrustCorrection.TabIndex = 77;
			this.tbThrustCorrection.TickFrequency = 32;
			this.tlToolTip.SetToolTip( this.tbThrustCorrection, "Compensates for loss of thrust when the craft is tilted.  (0 is disabled)" );
			this.tbThrustCorrection.Value = 256;
			this.tbThrustCorrection.Scroll += new System.EventHandler( this.tbThrustCorrection_Scroll );
			// 
			// btnControlReset
			// 
			this.btnControlReset.Location = new System.Drawing.Point( 136, 19 );
			this.btnControlReset.Name = "btnControlReset";
			this.btnControlReset.Size = new System.Drawing.Size( 75, 23 );
			this.btnControlReset.TabIndex = 0;
			this.btnControlReset.Text = "Reset";
			this.tlToolTip.SetToolTip( this.btnControlReset, "Automatically determine channel ranges and reverse settings" );
			this.btnControlReset.UseVisualStyleBackColor = true;
			this.btnControlReset.Click += new System.EventHandler( this.btnControlReset_Click );
			// 
			// tbCalibrateDocs
			// 
			this.tbCalibrateDocs.AcceptsReturn = true;
			this.tbCalibrateDocs.BackColor = System.Drawing.Color.Orange;
			this.tbCalibrateDocs.BorderStyle = System.Windows.Forms.BorderStyle.None;
			this.tbCalibrateDocs.Font = new System.Drawing.Font( "Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)) );
			this.tbCalibrateDocs.Location = new System.Drawing.Point( 7, 213 );
			this.tbCalibrateDocs.Multiline = true;
			this.tbCalibrateDocs.Name = "tbCalibrateDocs";
			this.tbCalibrateDocs.ReadOnly = true;
			this.tbCalibrateDocs.Size = new System.Drawing.Size( 706, 10 );
			this.tbCalibrateDocs.TabIndex = 64;
			this.tbCalibrateDocs.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
			this.tbCalibrateDocs.Visible = false;
			// 
			// lblYawSpeed
			// 
			this.lblYawSpeed.AutoSize = true;
			this.lblYawSpeed.Location = new System.Drawing.Point( 461, 268 );
			this.lblYawSpeed.Name = "lblYawSpeed";
			this.lblYawSpeed.Size = new System.Drawing.Size( 19, 13 );
			this.lblYawSpeed.TabIndex = 74;
			this.lblYawSpeed.Text = "40";
			// 
			// label5
			// 
			this.label5.Location = new System.Drawing.Point( 266, 252 );
			this.label5.Name = "label5";
			this.label5.Size = new System.Drawing.Size( 73, 45 );
			this.label5.TabIndex = 73;
			this.label5.Text = "Yaw Speed (Manual)";
			this.label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// tbYawSpeed
			// 
			this.tbYawSpeed.LargeChange = 12;
			this.tbYawSpeed.Location = new System.Drawing.Point( 345, 252 );
			this.tbYawSpeed.Maximum = 128;
			this.tbYawSpeed.Minimum = 8;
			this.tbYawSpeed.Name = "tbYawSpeed";
			this.tbYawSpeed.Size = new System.Drawing.Size( 110, 45 );
			this.tbYawSpeed.TabIndex = 20;
			this.tbYawSpeed.TickFrequency = 5;
			this.tlToolTip.SetToolTip( this.tbYawSpeed, "How quickly the craft will change heading.  Larger numbers are faster." );
			this.tbYawSpeed.Value = 40;
			this.tbYawSpeed.Scroll += new System.EventHandler( this.tbYawSpeed_Scroll );
			// 
			// btnUploadRollPitch
			// 
			this.btnUploadRollPitch.Location = new System.Drawing.Point( 323, 328 );
			this.btnUploadRollPitch.Name = "btnUploadRollPitch";
			this.btnUploadRollPitch.Size = new System.Drawing.Size( 75, 22 );
			this.btnUploadRollPitch.TabIndex = 22;
			this.btnUploadRollPitch.Text = "Upload";
			this.btnUploadRollPitch.UseVisualStyleBackColor = true;
			this.btnUploadRollPitch.Click += new System.EventHandler( this.btnUploadRollPitch_Click );
			// 
			// lblRollPitchSpeed
			// 
			this.lblRollPitchSpeed.AutoSize = true;
			this.lblRollPitchSpeed.Location = new System.Drawing.Point( 217, 312 );
			this.lblRollPitchSpeed.Name = "lblRollPitchSpeed";
			this.lblRollPitchSpeed.Size = new System.Drawing.Size( 19, 13 );
			this.lblRollPitchSpeed.TabIndex = 70;
			this.lblRollPitchSpeed.Text = "64";
			// 
			// lblRollPitchAngle
			// 
			this.lblRollPitchAngle.AutoSize = true;
			this.lblRollPitchAngle.Location = new System.Drawing.Point( 217, 252 );
			this.lblRollPitchAngle.Name = "lblRollPitchAngle";
			this.lblRollPitchAngle.Size = new System.Drawing.Size( 40, 13 );
			this.lblRollPitchAngle.TabIndex = 69;
			this.lblRollPitchAngle.Text = "30 deg";
			// 
			// label3
			// 
			this.label3.Location = new System.Drawing.Point( 8, 296 );
			this.label3.Name = "label3";
			this.label3.Size = new System.Drawing.Size( 87, 45 );
			this.label3.TabIndex = 68;
			this.label3.Text = "Roll / Pitch Speed (Manual)";
			this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// tbRollPitchSpeed
			// 
			this.tbRollPitchSpeed.LargeChange = 12;
			this.tbRollPitchSpeed.Location = new System.Drawing.Point( 101, 296 );
			this.tbRollPitchSpeed.Maximum = 128;
			this.tbRollPitchSpeed.Minimum = 8;
			this.tbRollPitchSpeed.Name = "tbRollPitchSpeed";
			this.tbRollPitchSpeed.Size = new System.Drawing.Size( 110, 45 );
			this.tbRollPitchSpeed.TabIndex = 21;
			this.tbRollPitchSpeed.TickFrequency = 5;
			this.tlToolTip.SetToolTip( this.tbRollPitchSpeed, "Speed the craft will tilt when given a full-stick command in manual mode (larger " +
					"numbers are faster)" );
			this.tbRollPitchSpeed.Value = 64;
			this.tbRollPitchSpeed.ValueChanged += new System.EventHandler( this.tbRollPitchSpeed_ValueChanged );
			// 
			// label2
			// 
			this.label2.Location = new System.Drawing.Point( 8, 236 );
			this.label2.Name = "label2";
			this.label2.Size = new System.Drawing.Size( 87, 45 );
			this.label2.TabIndex = 66;
			this.label2.Text = "Max Roll / Pitch (Auto Level)";
			this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// tbRollPitchAngle
			// 
			this.tbRollPitchAngle.LargeChange = 15;
			this.tbRollPitchAngle.Location = new System.Drawing.Point( 101, 236 );
			this.tbRollPitchAngle.Maximum = 75;
			this.tbRollPitchAngle.Minimum = 10;
			this.tbRollPitchAngle.Name = "tbRollPitchAngle";
			this.tbRollPitchAngle.Size = new System.Drawing.Size( 110, 45 );
			this.tbRollPitchAngle.TabIndex = 19;
			this.tbRollPitchAngle.TickFrequency = 5;
			this.tlToolTip.SetToolTip( this.tbRollPitchAngle, "Maximum angle the craft will tilt when given a full-stick command" );
			this.tbRollPitchAngle.Value = 30;
			this.tbRollPitchAngle.ValueChanged += new System.EventHandler( this.tbRollPitchAngle_ValueChanged );
			// 
			// btnCalibrate
			// 
			this.btnCalibrate.Location = new System.Drawing.Point( 233, 19 );
			this.btnCalibrate.Name = "btnCalibrate";
			this.btnCalibrate.Size = new System.Drawing.Size( 75, 23 );
			this.btnCalibrate.TabIndex = 17;
			this.btnCalibrate.Text = "Calibrate";
			this.tlToolTip.SetToolTip( this.btnCalibrate, "Automatically determine channel ranges and reverse settings" );
			this.btnCalibrate.UseVisualStyleBackColor = true;
			this.btnCalibrate.Click += new System.EventHandler( this.btnCalibrate_Click );
			// 
			// label1
			// 
			this.label1.AutoSize = true;
			this.label1.Location = new System.Drawing.Point( 325, 5 );
			this.label1.Name = "label1";
			this.label1.Size = new System.Drawing.Size( 77, 13 );
			this.label1.TabIndex = 62;
			this.label1.Text = "Receiver Type";
			// 
			// cbReceiverType
			// 
			this.cbReceiverType.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbReceiverType.FormattingEnabled = true;
			this.cbReceiverType.Items.AddRange( new object[] {
            "PWM",
            "SBUS"} );
			this.cbReceiverType.Location = new System.Drawing.Point( 328, 21 );
			this.cbReceiverType.Name = "cbReceiverType";
			this.cbReceiverType.Size = new System.Drawing.Size( 74, 21 );
			this.cbReceiverType.TabIndex = 18;
			this.tlToolTip.SetToolTip( this.cbReceiverType, "Choose the communication type for your receiver (SBUS = 1 wire, PWM = multiple wi" +
					"res)" );
			// 
			// cbRev8
			// 
			this.cbRev8.AutoSize = true;
			this.cbRev8.Location = new System.Drawing.Point( 670, 188 );
			this.cbRev8.Name = "cbRev8";
			this.cbRev8.Size = new System.Drawing.Size( 46, 17 );
			this.cbRev8.TabIndex = 16;
			this.cbRev8.Tag = "8";
			this.cbRev8.Text = "Rev";
			this.cbRev8.UseVisualStyleBackColor = true;
			this.cbRev8.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbRev7
			// 
			this.cbRev7.AutoSize = true;
			this.cbRev7.Location = new System.Drawing.Point( 670, 145 );
			this.cbRev7.Name = "cbRev7";
			this.cbRev7.Size = new System.Drawing.Size( 46, 17 );
			this.cbRev7.TabIndex = 15;
			this.cbRev7.Tag = "7";
			this.cbRev7.Text = "Rev";
			this.cbRev7.UseVisualStyleBackColor = true;
			this.cbRev7.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbRev2
			// 
			this.cbRev2.AutoSize = true;
			this.cbRev2.Location = new System.Drawing.Point( 670, 102 );
			this.cbRev2.Name = "cbRev2";
			this.cbRev2.Size = new System.Drawing.Size( 46, 17 );
			this.cbRev2.TabIndex = 14;
			this.cbRev2.Tag = "2";
			this.cbRev2.Text = "Rev";
			this.cbRev2.UseVisualStyleBackColor = true;
			this.cbRev2.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbRev3
			// 
			this.cbRev3.AutoSize = true;
			this.cbRev3.Location = new System.Drawing.Point( 670, 59 );
			this.cbRev3.Name = "cbRev3";
			this.cbRev3.Size = new System.Drawing.Size( 46, 17 );
			this.cbRev3.TabIndex = 13;
			this.cbRev3.Tag = "3";
			this.cbRev3.Text = "Rev";
			this.cbRev3.UseVisualStyleBackColor = true;
			this.cbRev3.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbRev6
			// 
			this.cbRev6.AutoSize = true;
			this.cbRev6.Location = new System.Drawing.Point( 7, 188 );
			this.cbRev6.Name = "cbRev6";
			this.cbRev6.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
			this.cbRev6.Size = new System.Drawing.Size( 46, 17 );
			this.cbRev6.TabIndex = 3;
			this.cbRev6.Tag = "6";
			this.cbRev6.Text = "Rev";
			this.cbRev6.UseVisualStyleBackColor = true;
			this.cbRev6.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbRev5
			// 
			this.cbRev5.AutoSize = true;
			this.cbRev5.Location = new System.Drawing.Point( 7, 145 );
			this.cbRev5.Name = "cbRev5";
			this.cbRev5.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
			this.cbRev5.Size = new System.Drawing.Size( 46, 17 );
			this.cbRev5.TabIndex = 2;
			this.cbRev5.Tag = "5";
			this.cbRev5.Text = "Rev";
			this.cbRev5.UseVisualStyleBackColor = true;
			this.cbRev5.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbRev4
			// 
			this.cbRev4.AutoSize = true;
			this.cbRev4.Location = new System.Drawing.Point( 7, 102 );
			this.cbRev4.Name = "cbRev4";
			this.cbRev4.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
			this.cbRev4.Size = new System.Drawing.Size( 46, 17 );
			this.cbRev4.TabIndex = 1;
			this.cbRev4.Tag = "4";
			this.cbRev4.Text = "Rev";
			this.cbRev4.UseVisualStyleBackColor = true;
			this.cbRev4.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbRev1
			// 
			this.cbRev1.AutoSize = true;
			this.cbRev1.Location = new System.Drawing.Point( 7, 59 );
			this.cbRev1.Name = "cbRev1";
			this.cbRev1.RightToLeft = System.Windows.Forms.RightToLeft.Yes;
			this.cbRev1.Size = new System.Drawing.Size( 46, 17 );
			this.cbRev1.TabIndex = 0;
			this.cbRev1.Tag = "1";
			this.cbRev1.Text = "Rev";
			this.cbRev1.UseVisualStyleBackColor = true;
			this.cbRev1.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbChannel8
			// 
			this.cbChannel8.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel8.FormattingEnabled = true;
			this.cbChannel8.Location = new System.Drawing.Point( 616, 186 );
			this.cbChannel8.Name = "cbChannel8";
			this.cbChannel8.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel8.TabIndex = 12;
			this.cbChannel8.Tag = "8";
			this.cbChannel8.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
			// 
			// cbChannel7
			// 
			this.cbChannel7.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel7.FormattingEnabled = true;
			this.cbChannel7.Location = new System.Drawing.Point( 616, 143 );
			this.cbChannel7.Name = "cbChannel7";
			this.cbChannel7.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel7.TabIndex = 11;
			this.cbChannel7.Tag = "7";
			this.cbChannel7.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
			// 
			// cbChannel2
			// 
			this.cbChannel2.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel2.FormattingEnabled = true;
			this.cbChannel2.Location = new System.Drawing.Point( 616, 100 );
			this.cbChannel2.Name = "cbChannel2";
			this.cbChannel2.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel2.TabIndex = 9;
			this.cbChannel2.Tag = "2";
			this.cbChannel2.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
			// 
			// cbChannel3
			// 
			this.cbChannel3.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel3.FormattingEnabled = true;
			this.cbChannel3.Location = new System.Drawing.Point( 616, 57 );
			this.cbChannel3.Name = "cbChannel3";
			this.cbChannel3.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel3.TabIndex = 8;
			this.cbChannel3.Tag = "3";
			this.cbChannel3.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
			// 
			// cbChannel6
			// 
			this.cbChannel6.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel6.FormattingEnabled = true;
			this.cbChannel6.Location = new System.Drawing.Point( 59, 186 );
			this.cbChannel6.Name = "cbChannel6";
			this.cbChannel6.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel6.TabIndex = 7;
			this.cbChannel6.Tag = "6";
			this.cbChannel6.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
			// 
			// cbChannel5
			// 
			this.cbChannel5.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel5.FormattingEnabled = true;
			this.cbChannel5.Location = new System.Drawing.Point( 59, 143 );
			this.cbChannel5.Name = "cbChannel5";
			this.cbChannel5.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel5.TabIndex = 6;
			this.cbChannel5.Tag = "5";
			this.cbChannel5.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
			// 
			// cbChannel4
			// 
			this.cbChannel4.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel4.FormattingEnabled = true;
			this.cbChannel4.Location = new System.Drawing.Point( 59, 100 );
			this.cbChannel4.Name = "cbChannel4";
			this.cbChannel4.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel4.TabIndex = 5;
			this.cbChannel4.Tag = "4";
			this.cbChannel4.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
			// 
			// cbChannel1
			// 
			this.cbChannel1.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel1.FormattingEnabled = true;
			this.cbChannel1.Location = new System.Drawing.Point( 59, 57 );
			this.cbChannel1.Name = "cbChannel1";
			this.cbChannel1.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel1.TabIndex = 4;
			this.cbChannel1.Tag = "1";
			this.cbChannel1.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
			// 
			// tpSystemSetup
			// 
			this.tpSystemSetup.Controls.Add( this.cbLowVoltageBuzzer );
			this.tpSystemSetup.Controls.Add( this.label16 );
			this.tpSystemSetup.Controls.Add( this.cbArmingDelay );
			this.tpSystemSetup.Controls.Add( this.label15 );
			this.tpSystemSetup.Controls.Add( this.cbDisarmDelay );
			this.tpSystemSetup.Controls.Add( this.label10 );
			this.tpSystemSetup.Controls.Add( this.udVoltageOffset );
			this.tpSystemSetup.Controls.Add( this.label12 );
			this.tpSystemSetup.Controls.Add( this.label9 );
			this.tpSystemSetup.Controls.Add( this.btnUploadThrottle );
			this.tpSystemSetup.Controls.Add( this.udLowVoltageAlarm );
			this.tpSystemSetup.Controls.Add( this.cbUseBatteryMonitor );
			this.tpSystemSetup.Controls.Add( this.groupBox1 );
			this.tpSystemSetup.Controls.Add( this.vbVoltage2 );
			this.tpSystemSetup.Location = new System.Drawing.Point( 4, 22 );
			this.tpSystemSetup.Name = "tpSystemSetup";
			this.tpSystemSetup.Size = new System.Drawing.Size( 721, 356 );
			this.tpSystemSetup.TabIndex = 4;
			this.tpSystemSetup.Text = "System Setup";
			// 
			// cbLowVoltageBuzzer
			// 
			this.cbLowVoltageBuzzer.AutoSize = true;
			this.cbLowVoltageBuzzer.Location = new System.Drawing.Point( 243, 109 );
			this.cbLowVoltageBuzzer.Name = "cbLowVoltageBuzzer";
			this.cbLowVoltageBuzzer.Size = new System.Drawing.Size( 120, 17 );
			this.cbLowVoltageBuzzer.TabIndex = 39;
			this.cbLowVoltageBuzzer.Text = "Low Voltage Buzzer";
			this.tlToolTip.SetToolTip( this.cbLowVoltageBuzzer, "Enable / Disable audible low-voltage buzzer" );
			this.cbLowVoltageBuzzer.UseVisualStyleBackColor = true;
			// 
			// label16
			// 
			this.label16.AutoSize = true;
			this.label16.Location = new System.Drawing.Point( 34, 150 );
			this.label16.Name = "label16";
			this.label16.Size = new System.Drawing.Size( 69, 13 );
			this.label16.TabIndex = 38;
			this.label16.Text = "Arming Delay";
			// 
			// cbArmingDelay
			// 
			this.cbArmingDelay.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbArmingDelay.FormattingEnabled = true;
			this.cbArmingDelay.Items.AddRange( new object[] {
            "1.00 sec",
            "0.50 sec",
            "0.25 sec",
            "Off"} );
			this.cbArmingDelay.Location = new System.Drawing.Point( 109, 147 );
			this.cbArmingDelay.Name = "cbArmingDelay";
			this.cbArmingDelay.Size = new System.Drawing.Size( 65, 21 );
			this.cbArmingDelay.TabIndex = 37;
			// 
			// label15
			// 
			this.label15.AutoSize = true;
			this.label15.Location = new System.Drawing.Point( 34, 176 );
			this.label15.Name = "label15";
			this.label15.Size = new System.Drawing.Size( 69, 13 );
			this.label15.TabIndex = 36;
			this.label15.Text = "Disarm Delay";
			// 
			// cbDisarmDelay
			// 
			this.cbDisarmDelay.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbDisarmDelay.FormattingEnabled = true;
			this.cbDisarmDelay.Items.AddRange( new object[] {
            "1.00 sec",
            "0.50 sec",
            "0.25 sec",
            "Off"} );
			this.cbDisarmDelay.Location = new System.Drawing.Point( 109, 173 );
			this.cbDisarmDelay.Name = "cbDisarmDelay";
			this.cbDisarmDelay.Size = new System.Drawing.Size( 65, 21 );
			this.cbDisarmDelay.TabIndex = 35;
			// 
			// label10
			// 
			this.label10.AutoSize = true;
			this.label10.Location = new System.Drawing.Point( 261, 84 );
			this.label10.Name = "label10";
			this.label10.Size = new System.Drawing.Size( 74, 13 );
			this.label10.TabIndex = 15;
			this.label10.Text = "Voltage Offset";
			// 
			// udVoltageOffset
			// 
			this.udVoltageOffset.DecimalPlaces = 2;
			this.udVoltageOffset.Increment = new decimal( new int[] {
            1,
            0,
            0,
            65536} );
			this.udVoltageOffset.Location = new System.Drawing.Point( 341, 82 );
			this.udVoltageOffset.Maximum = new decimal( new int[] {
            2,
            0,
            0,
            0} );
			this.udVoltageOffset.Minimum = new decimal( new int[] {
            2,
            0,
            0,
            -2147483648} );
			this.udVoltageOffset.Name = "udVoltageOffset";
			this.udVoltageOffset.Size = new System.Drawing.Size( 57, 20 );
			this.udVoltageOffset.TabIndex = 14;
			this.tlToolTip.SetToolTip( this.udVoltageOffset, "Adjust this value to compensate for the difference between actual voltage and the" +
					" displayed value" );
			this.udVoltageOffset.Value = new decimal( new int[] {
            2,
            0,
            0,
            0} );
			// 
			// label12
			// 
			this.label12.AutoSize = true;
			this.label12.Location = new System.Drawing.Point( 240, 133 );
			this.label12.Name = "label12";
			this.label12.Size = new System.Drawing.Size( 111, 13 );
			this.label12.TabIndex = 13;
			this.label12.Text = "TODO: Ascent limiting";
			// 
			// label9
			// 
			this.label9.AutoSize = true;
			this.label9.Location = new System.Drawing.Point( 240, 60 );
			this.label9.Name = "label9";
			this.label9.Size = new System.Drawing.Size( 95, 13 );
			this.label9.TabIndex = 10;
			this.label9.Text = "Low Voltage Alarm";
			// 
			// btnUploadThrottle
			// 
			this.btnUploadThrottle.Location = new System.Drawing.Point( 246, 161 );
			this.btnUploadThrottle.Name = "btnUploadThrottle";
			this.btnUploadThrottle.Size = new System.Drawing.Size( 130, 23 );
			this.btnUploadThrottle.TabIndex = 4;
			this.btnUploadThrottle.Text = "Upload Changes";
			this.btnUploadThrottle.UseVisualStyleBackColor = true;
			this.btnUploadThrottle.Click += new System.EventHandler( this.btnUploadThrottle_Click );
			// 
			// udLowVoltageAlarm
			// 
			this.udLowVoltageAlarm.DecimalPlaces = 2;
			this.udLowVoltageAlarm.Increment = new decimal( new int[] {
            1,
            0,
            0,
            65536} );
			this.udLowVoltageAlarm.Location = new System.Drawing.Point( 341, 58 );
			this.udLowVoltageAlarm.Maximum = new decimal( new int[] {
            168,
            0,
            0,
            65536} );
			this.udLowVoltageAlarm.Minimum = new decimal( new int[] {
            60,
            0,
            0,
            65536} );
			this.udLowVoltageAlarm.Name = "udLowVoltageAlarm";
			this.udLowVoltageAlarm.Size = new System.Drawing.Size( 57, 20 );
			this.udLowVoltageAlarm.TabIndex = 9;
			this.tlToolTip.SetToolTip( this.udLowVoltageAlarm, "LED and audio warning of low voltage when battery is below this value" );
			this.udLowVoltageAlarm.Value = new decimal( new int[] {
            60,
            0,
            0,
            65536} );
			// 
			// cbUseBatteryMonitor
			// 
			this.cbUseBatteryMonitor.AutoSize = true;
			this.cbUseBatteryMonitor.Location = new System.Drawing.Point( 243, 35 );
			this.cbUseBatteryMonitor.Name = "cbUseBatteryMonitor";
			this.cbUseBatteryMonitor.Size = new System.Drawing.Size( 133, 17 );
			this.cbUseBatteryMonitor.TabIndex = 8;
			this.cbUseBatteryMonitor.Text = "Enable Battery Monitor";
			this.tlToolTip.SetToolTip( this.cbUseBatteryMonitor, "Enable / disable monitoring of battery voltage" );
			this.cbUseBatteryMonitor.UseVisualStyleBackColor = true;
			// 
			// groupBox1
			// 
			this.groupBox1.Controls.Add( this.label6 );
			this.groupBox1.Controls.Add( this.label4 );
			this.groupBox1.Controls.Add( this.udTestThrottle );
			this.groupBox1.Controls.Add( this.label7 );
			this.groupBox1.Controls.Add( this.label8 );
			this.groupBox1.Controls.Add( this.udLowThrottle );
			this.groupBox1.Controls.Add( this.udHighThrottle );
			this.groupBox1.Controls.Add( this.udArmedLowThrottle );
			this.groupBox1.Location = new System.Drawing.Point( 8, 14 );
			this.groupBox1.Name = "groupBox1";
			this.groupBox1.Size = new System.Drawing.Size( 202, 118 );
			this.groupBox1.TabIndex = 7;
			this.groupBox1.TabStop = false;
			this.groupBox1.Text = "Motor Outputs";
			// 
			// label6
			// 
			this.label6.AutoSize = true;
			this.label6.Location = new System.Drawing.Point( 12, 46 );
			this.label6.Name = "label6";
			this.label6.Size = new System.Drawing.Size( 99, 13 );
			this.label6.TabIndex = 1;
			this.label6.Text = "Armed Low Throttle";
			// 
			// label4
			// 
			this.label4.AutoSize = true;
			this.label4.Location = new System.Drawing.Point( 44, 22 );
			this.label4.Name = "label4";
			this.label4.Size = new System.Drawing.Size( 66, 13 );
			this.label4.TabIndex = 0;
			this.label4.Text = "Low Throttle";
			// 
			// udTestThrottle
			// 
			this.udTestThrottle.Increment = new decimal( new int[] {
            20,
            0,
            0,
            0} );
			this.udTestThrottle.Location = new System.Drawing.Point( 117, 68 );
			this.udTestThrottle.Maximum = new decimal( new int[] {
            2000,
            0,
            0,
            0} );
			this.udTestThrottle.Minimum = new decimal( new int[] {
            800,
            0,
            0,
            0} );
			this.udTestThrottle.Name = "udTestThrottle";
			this.udTestThrottle.Size = new System.Drawing.Size( 73, 20 );
			this.udTestThrottle.TabIndex = 2;
			this.tlToolTip.SetToolTip( this.udTestThrottle, "Throttle value sent to ESCs when using Motor Test feature" );
			this.udTestThrottle.Value = new decimal( new int[] {
            1000,
            0,
            0,
            0} );
			// 
			// label7
			// 
			this.label7.AutoSize = true;
			this.label7.Location = new System.Drawing.Point( 44, 94 );
			this.label7.Name = "label7";
			this.label7.Size = new System.Drawing.Size( 68, 13 );
			this.label7.TabIndex = 2;
			this.label7.Text = "High Throttle";
			// 
			// label8
			// 
			this.label8.AutoSize = true;
			this.label8.Location = new System.Drawing.Point( 44, 70 );
			this.label8.Name = "label8";
			this.label8.Size = new System.Drawing.Size( 67, 13 );
			this.label8.TabIndex = 6;
			this.label8.Text = "Test Throttle";
			// 
			// udLowThrottle
			// 
			this.udLowThrottle.Increment = new decimal( new int[] {
            20,
            0,
            0,
            0} );
			this.udLowThrottle.Location = new System.Drawing.Point( 117, 20 );
			this.udLowThrottle.Maximum = new decimal( new int[] {
            2000,
            0,
            0,
            0} );
			this.udLowThrottle.Minimum = new decimal( new int[] {
            800,
            0,
            0,
            0} );
			this.udLowThrottle.Name = "udLowThrottle";
			this.udLowThrottle.Size = new System.Drawing.Size( 73, 20 );
			this.udLowThrottle.TabIndex = 0;
			this.tlToolTip.SetToolTip( this.udLowThrottle, "Minimum throttle value output to ESCs  (in microseconds)" );
			this.udLowThrottle.Value = new decimal( new int[] {
            1000,
            0,
            0,
            0} );
			// 
			// udHighThrottle
			// 
			this.udHighThrottle.Increment = new decimal( new int[] {
            20,
            0,
            0,
            0} );
			this.udHighThrottle.Location = new System.Drawing.Point( 117, 92 );
			this.udHighThrottle.Maximum = new decimal( new int[] {
            2000,
            0,
            0,
            0} );
			this.udHighThrottle.Minimum = new decimal( new int[] {
            800,
            0,
            0,
            0} );
			this.udHighThrottle.Name = "udHighThrottle";
			this.udHighThrottle.Size = new System.Drawing.Size( 73, 20 );
			this.udHighThrottle.TabIndex = 3;
			this.tlToolTip.SetToolTip( this.udHighThrottle, "Maximum throttle value output to ESCs  (in microseconds)" );
			this.udHighThrottle.Value = new decimal( new int[] {
            1000,
            0,
            0,
            0} );
			// 
			// udArmedLowThrottle
			// 
			this.udArmedLowThrottle.Increment = new decimal( new int[] {
            20,
            0,
            0,
            0} );
			this.udArmedLowThrottle.Location = new System.Drawing.Point( 117, 44 );
			this.udArmedLowThrottle.Maximum = new decimal( new int[] {
            2000,
            0,
            0,
            0} );
			this.udArmedLowThrottle.Minimum = new decimal( new int[] {
            800,
            0,
            0,
            0} );
			this.udArmedLowThrottle.Name = "udArmedLowThrottle";
			this.udArmedLowThrottle.Size = new System.Drawing.Size( 73, 20 );
			this.udArmedLowThrottle.TabIndex = 1;
			this.tlToolTip.SetToolTip( this.udArmedLowThrottle, "Minimum throttle value output to ESCs when armed  (in microseconds)" );
			this.udArmedLowThrottle.Value = new decimal( new int[] {
            1000,
            0,
            0,
            0} );
			// 
			// tpGyroCalibration
			// 
			this.tpGyroCalibration.Controls.Add( this.gCalibTemp );
			this.tpGyroCalibration.Controls.Add( this.gCalibZ );
			this.tpGyroCalibration.Controls.Add( this.gCalibY );
			this.tpGyroCalibration.Controls.Add( this.gCalibX );
			this.tpGyroCalibration.Controls.Add( this.btnUploadGyroCalibration );
			this.tpGyroCalibration.Controls.Add( this.btnResetGyroCal );
			this.tpGyroCalibration.Controls.Add( this.groupBox10 );
			this.tpGyroCalibration.Controls.Add( this.groupBox11 );
			this.tpGyroCalibration.Controls.Add( this.groupBox12 );
			this.tpGyroCalibration.Controls.Add( this.lfGraph );
			this.tpGyroCalibration.Location = new System.Drawing.Point( 4, 22 );
			this.tpGyroCalibration.Name = "tpGyroCalibration";
			this.tpGyroCalibration.Size = new System.Drawing.Size( 721, 356 );
			this.tpGyroCalibration.TabIndex = 5;
			this.tpGyroCalibration.Text = "Gyro Calibration";
			// 
			// btnUploadGyroCalibration
			// 
			this.btnUploadGyroCalibration.Location = new System.Drawing.Point( 61, 228 );
			this.btnUploadGyroCalibration.Name = "btnUploadGyroCalibration";
			this.btnUploadGyroCalibration.Size = new System.Drawing.Size( 147, 23 );
			this.btnUploadGyroCalibration.TabIndex = 34;
			this.btnUploadGyroCalibration.Text = "Upload Calibration Settings";
			this.btnUploadGyroCalibration.UseVisualStyleBackColor = true;
			this.btnUploadGyroCalibration.Click += new System.EventHandler( this.btnUploadCalibration_Click );
			// 
			// btnResetGyroCal
			// 
			this.btnResetGyroCal.Location = new System.Drawing.Point( 60, 176 );
			this.btnResetGyroCal.Name = "btnResetGyroCal";
			this.btnResetGyroCal.Size = new System.Drawing.Size( 147, 23 );
			this.btnResetGyroCal.TabIndex = 33;
			this.btnResetGyroCal.Text = "Restart Calibration";
			this.btnResetGyroCal.UseVisualStyleBackColor = true;
			this.btnResetGyroCal.Click += new System.EventHandler( this.btnResetCalib_Click );
			// 
			// groupBox10
			// 
			this.groupBox10.Controls.Add( this.label18 );
			this.groupBox10.Controls.Add( this.gzOffset );
			this.groupBox10.Controls.Add( this.label20 );
			this.groupBox10.Controls.Add( this.gzScale );
			this.groupBox10.Location = new System.Drawing.Point( 50, 111 );
			this.groupBox10.Name = "groupBox10";
			this.groupBox10.Size = new System.Drawing.Size( 168, 40 );
			this.groupBox10.TabIndex = 32;
			this.groupBox10.TabStop = false;
			this.groupBox10.Text = "GZ";
			// 
			// label18
			// 
			this.label18.AutoSize = true;
			this.label18.Location = new System.Drawing.Point( 91, 14 );
			this.label18.Name = "label18";
			this.label18.Size = new System.Drawing.Size( 35, 13 );
			this.label18.TabIndex = 11;
			this.label18.Text = "Offset";
			this.label18.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// gzOffset
			// 
			this.gzOffset.Location = new System.Drawing.Point( 132, 14 );
			this.gzOffset.Name = "gzOffset";
			this.gzOffset.Size = new System.Drawing.Size( 36, 13 );
			this.gzOffset.TabIndex = 10;
			this.gzOffset.Text = "0";
			this.gzOffset.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// label20
			// 
			this.label20.AutoSize = true;
			this.label20.Location = new System.Drawing.Point( 8, 14 );
			this.label20.Name = "label20";
			this.label20.Size = new System.Drawing.Size( 34, 13 );
			this.label20.TabIndex = 8;
			this.label20.Text = "Scale";
			this.label20.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
			// 
			// gzScale
			// 
			this.gzScale.Location = new System.Drawing.Point( 47, 14 );
			this.gzScale.Name = "gzScale";
			this.gzScale.Size = new System.Drawing.Size( 36, 13 );
			this.gzScale.TabIndex = 9;
			this.gzScale.Text = "0";
			this.gzScale.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// groupBox11
			// 
			this.groupBox11.Controls.Add( this.label22 );
			this.groupBox11.Controls.Add( this.gyOffset );
			this.groupBox11.Controls.Add( this.label24 );
			this.groupBox11.Controls.Add( this.gyScale );
			this.groupBox11.Location = new System.Drawing.Point( 49, 65 );
			this.groupBox11.Name = "groupBox11";
			this.groupBox11.Size = new System.Drawing.Size( 169, 40 );
			this.groupBox11.TabIndex = 31;
			this.groupBox11.TabStop = false;
			this.groupBox11.Text = "GY";
			// 
			// label22
			// 
			this.label22.AutoSize = true;
			this.label22.Location = new System.Drawing.Point( 92, 14 );
			this.label22.Name = "label22";
			this.label22.Size = new System.Drawing.Size( 35, 13 );
			this.label22.TabIndex = 11;
			this.label22.Text = "Offset";
			this.label22.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// gyOffset
			// 
			this.gyOffset.Location = new System.Drawing.Point( 133, 14 );
			this.gyOffset.Name = "gyOffset";
			this.gyOffset.Size = new System.Drawing.Size( 36, 13 );
			this.gyOffset.TabIndex = 10;
			this.gyOffset.Text = "0";
			this.gyOffset.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// label24
			// 
			this.label24.AutoSize = true;
			this.label24.Location = new System.Drawing.Point( 8, 14 );
			this.label24.Name = "label24";
			this.label24.Size = new System.Drawing.Size( 34, 13 );
			this.label24.TabIndex = 8;
			this.label24.Text = "Scale";
			this.label24.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
			// 
			// gyScale
			// 
			this.gyScale.Location = new System.Drawing.Point( 47, 14 );
			this.gyScale.Name = "gyScale";
			this.gyScale.Size = new System.Drawing.Size( 36, 13 );
			this.gyScale.TabIndex = 9;
			this.gyScale.Text = "0";
			this.gyScale.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// groupBox12
			// 
			this.groupBox12.Controls.Add( this.label26 );
			this.groupBox12.Controls.Add( this.gxOffset );
			this.groupBox12.Controls.Add( this.label28 );
			this.groupBox12.Controls.Add( this.gxScale );
			this.groupBox12.Location = new System.Drawing.Point( 49, 19 );
			this.groupBox12.Name = "groupBox12";
			this.groupBox12.Size = new System.Drawing.Size( 169, 40 );
			this.groupBox12.TabIndex = 30;
			this.groupBox12.TabStop = false;
			this.groupBox12.Text = "GX";
			// 
			// label26
			// 
			this.label26.AutoSize = true;
			this.label26.Location = new System.Drawing.Point( 92, 16 );
			this.label26.Name = "label26";
			this.label26.Size = new System.Drawing.Size( 35, 13 );
			this.label26.TabIndex = 7;
			this.label26.Text = "Offset";
			this.label26.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// gxOffset
			// 
			this.gxOffset.Location = new System.Drawing.Point( 133, 16 );
			this.gxOffset.Name = "gxOffset";
			this.gxOffset.Size = new System.Drawing.Size( 36, 13 );
			this.gxOffset.TabIndex = 6;
			this.gxOffset.Text = "0";
			this.gxOffset.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// label28
			// 
			this.label28.AutoSize = true;
			this.label28.Location = new System.Drawing.Point( 8, 16 );
			this.label28.Name = "label28";
			this.label28.Size = new System.Drawing.Size( 34, 13 );
			this.label28.TabIndex = 4;
			this.label28.Text = "Scale";
			this.label28.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
			// 
			// gxScale
			// 
			this.gxScale.Location = new System.Drawing.Point( 47, 16 );
			this.gxScale.Name = "gxScale";
			this.gxScale.Size = new System.Drawing.Size( 36, 13 );
			this.gxScale.TabIndex = 5;
			this.gxScale.Text = "0";
			this.gxScale.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// tpAccelCalibration
			// 
			this.tpAccelCalibration.Controls.Add( this.ocAccelOrient );
			this.tpAccelCalibration.Controls.Add( this.groupBox6 );
			this.tpAccelCalibration.Controls.Add( this.groupBox5 );
			this.tpAccelCalibration.Location = new System.Drawing.Point( 4, 22 );
			this.tpAccelCalibration.Name = "tpAccelCalibration";
			this.tpAccelCalibration.Size = new System.Drawing.Size( 721, 356 );
			this.tpAccelCalibration.TabIndex = 6;
			this.tpAccelCalibration.Text = "Accel Calibration";
			// 
			// groupBox6
			// 
			this.groupBox6.Controls.Add( this.btnUploadAngleCorrection );
			this.groupBox6.Controls.Add( this.udRollCorrection );
			this.groupBox6.Controls.Add( this.label41 );
			this.groupBox6.Controls.Add( this.label42 );
			this.groupBox6.Controls.Add( this.udPitchCorrection );
			this.groupBox6.Location = new System.Drawing.Point( 358, 23 );
			this.groupBox6.Name = "groupBox6";
			this.groupBox6.Size = new System.Drawing.Size( 313, 137 );
			this.groupBox6.TabIndex = 36;
			this.groupBox6.TabStop = false;
			this.groupBox6.Text = "Angular offset correction";
			// 
			// btnUploadAngleCorrection
			// 
			this.btnUploadAngleCorrection.Location = new System.Drawing.Point( 114, 102 );
			this.btnUploadAngleCorrection.Name = "btnUploadAngleCorrection";
			this.btnUploadAngleCorrection.Size = new System.Drawing.Size( 75, 23 );
			this.btnUploadAngleCorrection.TabIndex = 32;
			this.btnUploadAngleCorrection.Text = "Upload";
			this.btnUploadAngleCorrection.UseVisualStyleBackColor = true;
			this.btnUploadAngleCorrection.Click += new System.EventHandler( this.btnUploadAngleCorrection_Click );
			// 
			// udRollCorrection
			// 
			this.udRollCorrection.DecimalPlaces = 2;
			this.udRollCorrection.Increment = new decimal( new int[] {
            1,
            0,
            0,
            65536} );
			this.udRollCorrection.Location = new System.Drawing.Point( 165, 30 );
			this.udRollCorrection.Maximum = new decimal( new int[] {
            30,
            0,
            0,
            0} );
			this.udRollCorrection.Minimum = new decimal( new int[] {
            30,
            0,
            0,
            -2147483648} );
			this.udRollCorrection.Name = "udRollCorrection";
			this.udRollCorrection.Size = new System.Drawing.Size( 76, 20 );
			this.udRollCorrection.TabIndex = 28;
			this.tlToolTip.SetToolTip( this.udRollCorrection, "Use this to trim the roll value in level hover" );
			// 
			// label41
			// 
			this.label41.AutoSize = true;
			this.label41.Location = new System.Drawing.Point( 47, 32 );
			this.label41.Name = "label41";
			this.label41.Size = new System.Drawing.Size( 106, 13 );
			this.label41.TabIndex = 29;
			this.label41.Text = "Roll Angle Correction";
			// 
			// label42
			// 
			this.label42.AutoSize = true;
			this.label42.Location = new System.Drawing.Point( 47, 63 );
			this.label42.Name = "label42";
			this.label42.Size = new System.Drawing.Size( 112, 13 );
			this.label42.TabIndex = 31;
			this.label42.Text = "Pitch Angle Correction";
			// 
			// udPitchCorrection
			// 
			this.udPitchCorrection.DecimalPlaces = 2;
			this.udPitchCorrection.Increment = new decimal( new int[] {
            1,
            0,
            0,
            65536} );
			this.udPitchCorrection.Location = new System.Drawing.Point( 165, 61 );
			this.udPitchCorrection.Maximum = new decimal( new int[] {
            30,
            0,
            0,
            0} );
			this.udPitchCorrection.Minimum = new decimal( new int[] {
            30,
            0,
            0,
            -2147483648} );
			this.udPitchCorrection.Name = "udPitchCorrection";
			this.udPitchCorrection.Size = new System.Drawing.Size( 76, 20 );
			this.udPitchCorrection.TabIndex = 30;
			this.tlToolTip.SetToolTip( this.udPitchCorrection, "Use this to trim the pitch value in level hover" );
			// 
			// groupBox5
			// 
			this.groupBox5.Controls.Add( this.gAccelXCal );
			this.groupBox5.Controls.Add( this.gAccelYCal );
			this.groupBox5.Controls.Add( this.gAccelZCal );
			this.groupBox5.Controls.Add( this.label39 );
			this.groupBox5.Controls.Add( this.label38 );
			this.groupBox5.Controls.Add( this.label37 );
			this.groupBox5.Controls.Add( this.lblAccelCalFinal );
			this.groupBox5.Controls.Add( this.btnAccelCal1 );
			this.groupBox5.Controls.Add( this.lblAccelCal4 );
			this.groupBox5.Controls.Add( this.btnAccelCal2 );
			this.groupBox5.Controls.Add( this.lblAccelCal3 );
			this.groupBox5.Controls.Add( this.btnAccelCal3 );
			this.groupBox5.Controls.Add( this.lblAccelCal2 );
			this.groupBox5.Controls.Add( this.btnAccelCal4 );
			this.groupBox5.Controls.Add( this.lblAccelCal1 );
			this.groupBox5.Controls.Add( this.btnUploadAccelCal );
			this.groupBox5.Location = new System.Drawing.Point( 49, 23 );
			this.groupBox5.Name = "groupBox5";
			this.groupBox5.Size = new System.Drawing.Size( 303, 308 );
			this.groupBox5.TabIndex = 35;
			this.groupBox5.TabStop = false;
			this.groupBox5.Text = "Fixed offset compensation";
			// 
			// label39
			// 
			this.label39.AutoSize = true;
			this.label39.Location = new System.Drawing.Point( 5, 111 );
			this.label39.Name = "label39";
			this.label39.Size = new System.Drawing.Size( 44, 13 );
			this.label39.TabIndex = 15;
			this.label39.Text = "Accel X";
			// 
			// label38
			// 
			this.label38.AutoSize = true;
			this.label38.Location = new System.Drawing.Point( 100, 111 );
			this.label38.Name = "label38";
			this.label38.Size = new System.Drawing.Size( 44, 13 );
			this.label38.TabIndex = 16;
			this.label38.Text = "Accel Y";
			// 
			// label37
			// 
			this.label37.AutoSize = true;
			this.label37.Location = new System.Drawing.Point( 198, 111 );
			this.label37.Name = "label37";
			this.label37.Size = new System.Drawing.Size( 44, 13 );
			this.label37.TabIndex = 17;
			this.label37.Text = "Accel Z";
			// 
			// lblAccelCalFinal
			// 
			this.lblAccelCalFinal.Location = new System.Drawing.Point( 100, 278 );
			this.lblAccelCalFinal.Name = "lblAccelCalFinal";
			this.lblAccelCalFinal.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCalFinal.TabIndex = 27;
			// 
			// btnAccelCal1
			// 
			this.btnAccelCal1.Location = new System.Drawing.Point( 8, 149 );
			this.btnAccelCal1.Name = "btnAccelCal1";
			this.btnAccelCal1.Size = new System.Drawing.Size( 75, 23 );
			this.btnAccelCal1.TabIndex = 18;
			this.btnAccelCal1.Text = "Reading 1";
			this.btnAccelCal1.UseVisualStyleBackColor = true;
			this.btnAccelCal1.Click += new System.EventHandler( this.btnAccelCal1_Click );
			// 
			// lblAccelCal4
			// 
			this.lblAccelCal4.Location = new System.Drawing.Point( 100, 241 );
			this.lblAccelCal4.Name = "lblAccelCal4";
			this.lblAccelCal4.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCal4.TabIndex = 26;
			// 
			// btnAccelCal2
			// 
			this.btnAccelCal2.Location = new System.Drawing.Point( 8, 178 );
			this.btnAccelCal2.Name = "btnAccelCal2";
			this.btnAccelCal2.Size = new System.Drawing.Size( 75, 23 );
			this.btnAccelCal2.TabIndex = 19;
			this.btnAccelCal2.Text = "Reading 2";
			this.btnAccelCal2.UseVisualStyleBackColor = true;
			this.btnAccelCal2.Click += new System.EventHandler( this.btnAccelCal2_Click );
			// 
			// lblAccelCal3
			// 
			this.lblAccelCal3.Location = new System.Drawing.Point( 100, 212 );
			this.lblAccelCal3.Name = "lblAccelCal3";
			this.lblAccelCal3.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCal3.TabIndex = 25;
			// 
			// btnAccelCal3
			// 
			this.btnAccelCal3.Location = new System.Drawing.Point( 8, 207 );
			this.btnAccelCal3.Name = "btnAccelCal3";
			this.btnAccelCal3.Size = new System.Drawing.Size( 75, 23 );
			this.btnAccelCal3.TabIndex = 20;
			this.btnAccelCal3.Text = "Reading 3";
			this.btnAccelCal3.UseVisualStyleBackColor = true;
			this.btnAccelCal3.Click += new System.EventHandler( this.btnAccelCal3_Click );
			// 
			// lblAccelCal2
			// 
			this.lblAccelCal2.Location = new System.Drawing.Point( 100, 183 );
			this.lblAccelCal2.Name = "lblAccelCal2";
			this.lblAccelCal2.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCal2.TabIndex = 24;
			// 
			// btnAccelCal4
			// 
			this.btnAccelCal4.Location = new System.Drawing.Point( 8, 236 );
			this.btnAccelCal4.Name = "btnAccelCal4";
			this.btnAccelCal4.Size = new System.Drawing.Size( 75, 23 );
			this.btnAccelCal4.TabIndex = 21;
			this.btnAccelCal4.Text = "Reading 4";
			this.btnAccelCal4.UseVisualStyleBackColor = true;
			this.btnAccelCal4.Click += new System.EventHandler( this.btnAccelCal4_Click );
			// 
			// lblAccelCal1
			// 
			this.lblAccelCal1.Location = new System.Drawing.Point( 100, 154 );
			this.lblAccelCal1.Name = "lblAccelCal1";
			this.lblAccelCal1.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCal1.TabIndex = 23;
			// 
			// btnUploadAccelCal
			// 
			this.btnUploadAccelCal.Location = new System.Drawing.Point( 8, 273 );
			this.btnUploadAccelCal.Name = "btnUploadAccelCal";
			this.btnUploadAccelCal.Size = new System.Drawing.Size( 75, 23 );
			this.btnUploadAccelCal.TabIndex = 22;
			this.btnUploadAccelCal.Text = "Upload";
			this.btnUploadAccelCal.UseVisualStyleBackColor = true;
			this.btnUploadAccelCal.Click += new System.EventHandler( this.btnUploadAccelCal_Click );
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
			// checkBox4
			// 
			this.checkBox4.AutoSize = true;
			this.checkBox4.Location = new System.Drawing.Point( 13, 135 );
			this.checkBox4.Name = "checkBox4";
			this.checkBox4.Size = new System.Drawing.Size( 46, 17 );
			this.checkBox4.TabIndex = 55;
			this.checkBox4.Text = "Rev";
			this.checkBox4.UseVisualStyleBackColor = true;
			// 
			// checkBox5
			// 
			this.checkBox5.AutoSize = true;
			this.checkBox5.Location = new System.Drawing.Point( 13, 92 );
			this.checkBox5.Name = "checkBox5";
			this.checkBox5.Size = new System.Drawing.Size( 46, 17 );
			this.checkBox5.TabIndex = 54;
			this.checkBox5.Text = "Rev";
			this.checkBox5.UseVisualStyleBackColor = true;
			// 
			// checkBox6
			// 
			this.checkBox6.AutoSize = true;
			this.checkBox6.Location = new System.Drawing.Point( 13, 49 );
			this.checkBox6.Name = "checkBox6";
			this.checkBox6.Size = new System.Drawing.Size( 46, 17 );
			this.checkBox6.TabIndex = 53;
			this.checkBox6.Text = "Rev";
			this.checkBox6.UseVisualStyleBackColor = true;
			// 
			// radioButton1
			// 
			this.radioButton1.AutoSize = true;
			this.radioButton1.Location = new System.Drawing.Point( 8, 253 );
			this.radioButton1.Name = "radioButton1";
			this.radioButton1.Size = new System.Drawing.Size( 100, 17 );
			this.radioButton1.TabIndex = 52;
			this.radioButton1.TabStop = true;
			this.radioButton1.Text = "SBUS Receiver";
			this.radioButton1.UseVisualStyleBackColor = true;
			// 
			// radioButton2
			// 
			this.radioButton2.AutoSize = true;
			this.radioButton2.Location = new System.Drawing.Point( 8, 231 );
			this.radioButton2.Name = "radioButton2";
			this.radioButton2.Size = new System.Drawing.Size( 98, 17 );
			this.radioButton2.TabIndex = 51;
			this.radioButton2.TabStop = true;
			this.radioButton2.Text = "PWM Receiver";
			this.radioButton2.UseVisualStyleBackColor = true;
			// 
			// comboBox1
			// 
			this.comboBox1.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.comboBox1.FormattingEnabled = true;
			this.comboBox1.Location = new System.Drawing.Point( 616, 176 );
			this.comboBox1.Name = "comboBox1";
			this.comboBox1.Size = new System.Drawing.Size( 40, 21 );
			this.comboBox1.TabIndex = 50;
			// 
			// comboBox2
			// 
			this.comboBox2.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.comboBox2.FormattingEnabled = true;
			this.comboBox2.Location = new System.Drawing.Point( 616, 133 );
			this.comboBox2.Name = "comboBox2";
			this.comboBox2.Size = new System.Drawing.Size( 40, 21 );
			this.comboBox2.TabIndex = 49;
			// 
			// comboBox3
			// 
			this.comboBox3.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.comboBox3.FormattingEnabled = true;
			this.comboBox3.Location = new System.Drawing.Point( 616, 90 );
			this.comboBox3.Name = "comboBox3";
			this.comboBox3.Size = new System.Drawing.Size( 40, 21 );
			this.comboBox3.TabIndex = 48;
			// 
			// comboBox4
			// 
			this.comboBox4.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.comboBox4.FormattingEnabled = true;
			this.comboBox4.Location = new System.Drawing.Point( 616, 47 );
			this.comboBox4.Name = "comboBox4";
			this.comboBox4.Size = new System.Drawing.Size( 40, 21 );
			this.comboBox4.TabIndex = 47;
			// 
			// comboBox5
			// 
			this.comboBox5.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.comboBox5.FormattingEnabled = true;
			this.comboBox5.Location = new System.Drawing.Point( 65, 176 );
			this.comboBox5.Name = "comboBox5";
			this.comboBox5.Size = new System.Drawing.Size( 40, 21 );
			this.comboBox5.TabIndex = 46;
			// 
			// comboBox6
			// 
			this.comboBox6.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.comboBox6.FormattingEnabled = true;
			this.comboBox6.Location = new System.Drawing.Point( 65, 133 );
			this.comboBox6.Name = "comboBox6";
			this.comboBox6.Size = new System.Drawing.Size( 40, 21 );
			this.comboBox6.TabIndex = 45;
			// 
			// comboBox7
			// 
			this.comboBox7.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.comboBox7.FormattingEnabled = true;
			this.comboBox7.Location = new System.Drawing.Point( 65, 90 );
			this.comboBox7.Name = "comboBox7";
			this.comboBox7.Size = new System.Drawing.Size( 40, 21 );
			this.comboBox7.TabIndex = 44;
			// 
			// comboBox8
			// 
			this.comboBox8.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.comboBox8.FormattingEnabled = true;
			this.comboBox8.Location = new System.Drawing.Point( 65, 47 );
			this.comboBox8.Name = "comboBox8";
			this.comboBox8.Size = new System.Drawing.Size( 40, 21 );
			this.comboBox8.TabIndex = 43;
			// 
			// vbVoltage
			// 
			this.vbVoltage.FromLeft = true;
			this.vbVoltage.LeftLabel = "Battery Voltage";
			this.vbVoltage.Location = new System.Drawing.Point( 183, 160 );
			this.vbVoltage.MaxValue = 1260;
			this.vbVoltage.MinValue = 900;
			this.vbVoltage.Name = "vbVoltage";
			this.vbVoltage.RightLabel = "0";
			this.vbVoltage.Size = new System.Drawing.Size( 150, 20 );
			this.vbVoltage.TabIndex = 33;
			this.vbVoltage.Value = 1200;
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
			// plotSensors
			// 
			this.plotSensors.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
						| System.Windows.Forms.AnchorStyles.Left)
						| System.Windows.Forms.AnchorStyles.Right)));
			this.plotSensors.BackColor = System.Drawing.Color.Transparent;
			this.plotSensors.BackgroundColorBot = System.Drawing.Color.White;
			this.plotSensors.BackgroundColorTop = System.Drawing.Color.White;
			this.plotSensors.DashedGridColor = System.Drawing.Color.DarkGray;
			this.plotSensors.DoubleBuffering = true;
			this.plotSensors.Location = new System.Drawing.Point( 87, 6 );
			this.plotSensors.Name = "plotSensors";
			this.plotSensors.PlaySpeed = 0.5F;
			this.plotSensors.Size = new System.Drawing.Size( 626, 344 );
			this.plotSensors.SolidGridColor = System.Drawing.Color.DarkGray;
			this.plotSensors.TabIndex = 14;
			// 
			// vbR_Channel8
			// 
			this.vbR_Channel8.FromLeft = true;
			this.vbR_Channel8.LeftLabel = "Aux3";
			this.vbR_Channel8.Location = new System.Drawing.Point( 527, 186 );
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
			this.vbR_Channel7.Location = new System.Drawing.Point( 527, 143 );
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
			this.vbR_Channel6.Location = new System.Drawing.Point( 111, 186 );
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
			this.vbR_Channel5.Location = new System.Drawing.Point( 111, 143 );
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
			this.vbR_RS_XValue.Location = new System.Drawing.Point( 527, 100 );
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
			this.vbR_RS_YValue.Location = new System.Drawing.Point( 527, 57 );
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
			this.vbR_LS_XValue.Location = new System.Drawing.Point( 111, 100 );
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
			this.vbR_LS_YValue.Location = new System.Drawing.Point( 111, 57 );
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
			this.rsR_Right.Location = new System.Drawing.Point( 367, 57 );
			this.rsR_Right.Name = "rsR_Right";
			this.rsR_Right.Size = new System.Drawing.Size( 150, 150 );
			this.rsR_Right.TabIndex = 34;
			this.rsR_Right.Text = "radioStick2";
			// 
			// rsR_Left
			// 
			this.rsR_Left.Location = new System.Drawing.Point( 204, 57 );
			this.rsR_Left.Name = "rsR_Left";
			this.rsR_Left.Size = new System.Drawing.Size( 150, 150 );
			this.rsR_Left.TabIndex = 33;
			this.rsR_Left.Text = "radioStick1";
			// 
			// vbVoltage2
			// 
			this.vbVoltage2.FromLeft = true;
			this.vbVoltage2.LeftLabel = "Battery Voltage";
			this.vbVoltage2.Location = new System.Drawing.Point( 243, 7 );
			this.vbVoltage2.MaxValue = 1260;
			this.vbVoltage2.MinValue = 900;
			this.vbVoltage2.Name = "vbVoltage2";
			this.vbVoltage2.RightLabel = "0";
			this.vbVoltage2.Size = new System.Drawing.Size( 150, 20 );
			this.vbVoltage2.TabIndex = 34;
			this.vbVoltage2.Value = 1200;
			// 
			// gCalibTemp
			// 
			this.gCalibTemp.AverageCount = 128;
			this.gCalibTemp.BackColor = System.Drawing.SystemColors.Control;
			this.gCalibTemp.Location = new System.Drawing.Point( 251, 254 );
			this.gCalibTemp.Name = "gCalibTemp";
			this.gCalibTemp.Range = 8192F;
			this.gCalibTemp.Size = new System.Drawing.Size( 84, 84 );
			this.gCalibTemp.TabIndex = 38;
			this.gCalibTemp.Value = 0F;
			// 
			// gCalibZ
			// 
			this.gCalibZ.AverageCount = 128;
			this.gCalibZ.BackColor = System.Drawing.SystemColors.Control;
			this.gCalibZ.Location = new System.Drawing.Point( 560, 254 );
			this.gCalibZ.Name = "gCalibZ";
			this.gCalibZ.Range = 8192F;
			this.gCalibZ.Size = new System.Drawing.Size( 84, 84 );
			this.gCalibZ.TabIndex = 37;
			this.gCalibZ.Value = 0F;
			// 
			// gCalibY
			// 
			this.gCalibY.AverageCount = 128;
			this.gCalibY.BackColor = System.Drawing.SystemColors.Control;
			this.gCalibY.Location = new System.Drawing.Point( 466, 254 );
			this.gCalibY.Name = "gCalibY";
			this.gCalibY.Range = 8192F;
			this.gCalibY.Size = new System.Drawing.Size( 84, 84 );
			this.gCalibY.TabIndex = 36;
			this.gCalibY.Value = 0F;
			// 
			// gCalibX
			// 
			this.gCalibX.AverageCount = 128;
			this.gCalibX.BackColor = System.Drawing.SystemColors.Control;
			this.gCalibX.Location = new System.Drawing.Point( 372, 254 );
			this.gCalibX.Name = "gCalibX";
			this.gCalibX.Range = 8192F;
			this.gCalibX.Size = new System.Drawing.Size( 84, 84 );
			this.gCalibX.TabIndex = 35;
			this.gCalibX.Value = 0F;
			// 
			// lfGraph
			// 
			this.lfGraph.Location = new System.Drawing.Point( 224, 19 );
			this.lfGraph.Name = "lfGraph";
			this.lfGraph.Size = new System.Drawing.Size( 447, 232 );
			this.lfGraph.TabIndex = 29;
			// 
			// ocAccelOrient
			// 
			this.ocAccelOrient.CubeDepth = 1.25F;
			this.ocAccelOrient.CubeHeight = 0.6F;
			this.ocAccelOrient.CubeWidth = 1.25F;
			this.ocAccelOrient.Location = new System.Drawing.Point( 358, 166 );
			this.ocAccelOrient.Name = "ocAccelOrient";
			this.ocAccelOrient.Size = new System.Drawing.Size( 313, 165 );
			this.ocAccelOrient.TabIndex = 37;
			// 
			// gAccelXCal
			// 
			this.gAccelXCal.AverageCount = 325;
			this.gAccelXCal.Font = new System.Drawing.Font( "Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)) );
			this.gAccelXCal.Location = new System.Drawing.Point( 6, 19 );
			this.gAccelXCal.Name = "gAccelXCal";
			this.gAccelXCal.Range = 32768F;
			this.gAccelXCal.Size = new System.Drawing.Size( 91, 89 );
			this.gAccelXCal.TabIndex = 12;
			this.gAccelXCal.Value = 0F;
			// 
			// gAccelYCal
			// 
			this.gAccelYCal.AverageCount = 325;
			this.gAccelYCal.Font = new System.Drawing.Font( "Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)) );
			this.gAccelYCal.Location = new System.Drawing.Point( 103, 19 );
			this.gAccelYCal.Name = "gAccelYCal";
			this.gAccelYCal.Range = 32768F;
			this.gAccelYCal.Size = new System.Drawing.Size( 91, 89 );
			this.gAccelYCal.TabIndex = 13;
			this.gAccelYCal.Value = 0F;
			// 
			// gAccelZCal
			// 
			this.gAccelZCal.AverageCount = 325;
			this.gAccelZCal.Font = new System.Drawing.Font( "Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)) );
			this.gAccelZCal.Location = new System.Drawing.Point( 200, 19 );
			this.gAccelZCal.Name = "gAccelZCal";
			this.gAccelZCal.Range = 32768F;
			this.gAccelZCal.Size = new System.Drawing.Size( 91, 89 );
			this.gAccelZCal.TabIndex = 14;
			this.gAccelZCal.Value = 0F;
			// 
			// valueBar1
			// 
			this.valueBar1.FromLeft = true;
			this.valueBar1.LeftLabel = "Aux3";
			this.valueBar1.Location = new System.Drawing.Point( 527, 176 );
			this.valueBar1.MaxValue = 1024;
			this.valueBar1.MinValue = -1024;
			this.valueBar1.Name = "valueBar1";
			this.valueBar1.RightLabel = "0";
			this.valueBar1.Size = new System.Drawing.Size( 83, 21 );
			this.valueBar1.TabIndex = 42;
			this.valueBar1.Value = 0;
			// 
			// valueBar2
			// 
			this.valueBar2.FromLeft = true;
			this.valueBar2.LeftLabel = "Aux2";
			this.valueBar2.Location = new System.Drawing.Point( 527, 133 );
			this.valueBar2.MaxValue = 1024;
			this.valueBar2.MinValue = -1024;
			this.valueBar2.Name = "valueBar2";
			this.valueBar2.RightLabel = "0";
			this.valueBar2.Size = new System.Drawing.Size( 83, 21 );
			this.valueBar2.TabIndex = 41;
			this.valueBar2.Value = 0;
			// 
			// valueBar3
			// 
			this.valueBar3.FromLeft = true;
			this.valueBar3.LeftLabel = "Aux1";
			this.valueBar3.Location = new System.Drawing.Point( 111, 176 );
			this.valueBar3.MaxValue = 1024;
			this.valueBar3.MinValue = -1024;
			this.valueBar3.Name = "valueBar3";
			this.valueBar3.RightLabel = "0";
			this.valueBar3.Size = new System.Drawing.Size( 83, 21 );
			this.valueBar3.TabIndex = 40;
			this.valueBar3.Value = 0;
			// 
			// valueBar4
			// 
			this.valueBar4.FromLeft = true;
			this.valueBar4.LeftLabel = "Gear";
			this.valueBar4.Location = new System.Drawing.Point( 111, 133 );
			this.valueBar4.MaxValue = 1024;
			this.valueBar4.MinValue = -1024;
			this.valueBar4.Name = "valueBar4";
			this.valueBar4.RightLabel = "0";
			this.valueBar4.Size = new System.Drawing.Size( 83, 21 );
			this.valueBar4.TabIndex = 39;
			this.valueBar4.Value = 0;
			// 
			// valueBar5
			// 
			this.valueBar5.FromLeft = true;
			this.valueBar5.LeftLabel = "Aileron";
			this.valueBar5.Location = new System.Drawing.Point( 527, 90 );
			this.valueBar5.MaxValue = 1024;
			this.valueBar5.MinValue = -1024;
			this.valueBar5.Name = "valueBar5";
			this.valueBar5.RightLabel = "0";
			this.valueBar5.Size = new System.Drawing.Size( 83, 21 );
			this.valueBar5.TabIndex = 38;
			this.valueBar5.Value = 0;
			// 
			// valueBar6
			// 
			this.valueBar6.FromLeft = true;
			this.valueBar6.LeftLabel = "Elevator";
			this.valueBar6.Location = new System.Drawing.Point( 527, 47 );
			this.valueBar6.MaxValue = 1024;
			this.valueBar6.MinValue = -1024;
			this.valueBar6.Name = "valueBar6";
			this.valueBar6.RightLabel = "0";
			this.valueBar6.Size = new System.Drawing.Size( 83, 21 );
			this.valueBar6.TabIndex = 37;
			this.valueBar6.Value = 0;
			// 
			// valueBar7
			// 
			this.valueBar7.FromLeft = true;
			this.valueBar7.LeftLabel = "Rudder";
			this.valueBar7.Location = new System.Drawing.Point( 111, 90 );
			this.valueBar7.MaxValue = 1024;
			this.valueBar7.MinValue = -1024;
			this.valueBar7.Name = "valueBar7";
			this.valueBar7.RightLabel = "0";
			this.valueBar7.Size = new System.Drawing.Size( 83, 21 );
			this.valueBar7.TabIndex = 36;
			this.valueBar7.Value = 0;
			// 
			// valueBar8
			// 
			this.valueBar8.FromLeft = true;
			this.valueBar8.LeftLabel = "Throttle";
			this.valueBar8.Location = new System.Drawing.Point( 111, 47 );
			this.valueBar8.MaxValue = 1024;
			this.valueBar8.MinValue = -1024;
			this.valueBar8.Name = "valueBar8";
			this.valueBar8.RightLabel = "0";
			this.valueBar8.Size = new System.Drawing.Size( 83, 21 );
			this.valueBar8.TabIndex = 35;
			this.valueBar8.Value = 0;
			// 
			// radioStick1
			// 
			this.radioStick1.Location = new System.Drawing.Point( 367, 47 );
			this.radioStick1.Name = "radioStick1";
			this.radioStick1.Size = new System.Drawing.Size( 150, 150 );
			this.radioStick1.TabIndex = 34;
			this.radioStick1.Text = "radioStick2";
			// 
			// radioStick2
			// 
			this.radioStick2.Location = new System.Drawing.Point( 204, 47 );
			this.radioStick2.Name = "radioStick2";
			this.radioStick2.Size = new System.Drawing.Size( 150, 150 );
			this.radioStick2.TabIndex = 33;
			this.radioStick2.Text = "radioStick1";
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
			this.tpControlSetup.PerformLayout();
			((System.ComponentModel.ISupportInitialize)(this.tbAccelCorrectionFilter)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbThrustCorrection)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbYawSpeed)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbRollPitchSpeed)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbRollPitchAngle)).EndInit();
			this.tpSystemSetup.ResumeLayout( false );
			this.tpSystemSetup.PerformLayout();
			((System.ComponentModel.ISupportInitialize)(this.udVoltageOffset)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udLowVoltageAlarm)).EndInit();
			this.groupBox1.ResumeLayout( false );
			this.groupBox1.PerformLayout();
			((System.ComponentModel.ISupportInitialize)(this.udTestThrottle)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udLowThrottle)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udHighThrottle)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udArmedLowThrottle)).EndInit();
			this.tpGyroCalibration.ResumeLayout( false );
			this.groupBox10.ResumeLayout( false );
			this.groupBox10.PerformLayout();
			this.groupBox11.ResumeLayout( false );
			this.groupBox11.PerformLayout();
			this.groupBox12.ResumeLayout( false );
			this.groupBox12.PerformLayout();
			this.tpAccelCalibration.ResumeLayout( false );
			this.groupBox6.ResumeLayout( false );
			this.groupBox6.PerformLayout();
			((System.ComponentModel.ISupportInitialize)(this.udRollCorrection)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udPitchCorrection)).EndInit();
			this.groupBox5.ResumeLayout( false );
			this.groupBox5.PerformLayout();
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
		private System.Windows.Forms.CheckBox cbRev8;
		private System.Windows.Forms.CheckBox cbRev7;
		private System.Windows.Forms.CheckBox cbRev2;
		private System.Windows.Forms.CheckBox cbRev3;
		private System.Windows.Forms.CheckBox cbRev6;
		private System.Windows.Forms.CheckBox cbRev5;
		private System.Windows.Forms.CheckBox cbRev4;
		private System.Windows.Forms.CheckBox cbRev1;
		private System.Windows.Forms.CheckBox checkBox4;
		private System.Windows.Forms.CheckBox checkBox5;
		private System.Windows.Forms.CheckBox checkBox6;
		private System.Windows.Forms.RadioButton radioButton1;
		private System.Windows.Forms.RadioButton radioButton2;
		private System.Windows.Forms.ComboBox comboBox1;
		private System.Windows.Forms.ComboBox comboBox2;
		private System.Windows.Forms.ComboBox comboBox3;
		private System.Windows.Forms.ComboBox comboBox4;
		private System.Windows.Forms.ComboBox comboBox5;
		private System.Windows.Forms.ComboBox comboBox6;
		private System.Windows.Forms.ComboBox comboBox7;
		private System.Windows.Forms.ComboBox comboBox8;
		private Elev8.Controls.ValueBar valueBar1;
		private Elev8.Controls.ValueBar valueBar2;
		private Elev8.Controls.ValueBar valueBar3;
		private Elev8.Controls.ValueBar valueBar4;
		private Elev8.Controls.ValueBar valueBar5;
		private Elev8.Controls.ValueBar valueBar6;
		private Elev8.Controls.ValueBar valueBar7;
		private Elev8.Controls.ValueBar valueBar8;
		private RadioStick radioStick1;
		private RadioStick radioStick2;
		private System.Windows.Forms.Label label1;
		private System.Windows.Forms.ComboBox cbReceiverType;
		private System.Windows.Forms.Button btnCalibrate;
		private System.Windows.Forms.ToolTip tlToolTip;
		private System.Windows.Forms.TextBox tbCalibrateDocs;
		private GraphLib.PlotterDisplayEx plotSensors;
		private System.Windows.Forms.TabPage tpGyroCalibration;
		private Gauge gCalibTemp;
		private Gauge gCalibZ;
		private Gauge gCalibY;
		private Gauge gCalibX;
		private System.Windows.Forms.Button btnUploadGyroCalibration;
		private System.Windows.Forms.Button btnResetGyroCal;
		private System.Windows.Forms.GroupBox groupBox10;
		private System.Windows.Forms.Label label18;
		private System.Windows.Forms.Label gzOffset;
		private System.Windows.Forms.Label label20;
		private System.Windows.Forms.Label gzScale;
		private System.Windows.Forms.GroupBox groupBox11;
		private System.Windows.Forms.Label label22;
		private System.Windows.Forms.Label gyOffset;
		private System.Windows.Forms.Label label24;
		private System.Windows.Forms.Label gyScale;
		private System.Windows.Forms.GroupBox groupBox12;
		private System.Windows.Forms.Label label26;
		private System.Windows.Forms.Label gxOffset;
		private System.Windows.Forms.Label label28;
		private System.Windows.Forms.Label gxScale;
		private LineFit lfGraph;
		private System.Windows.Forms.TabPage tpAccelCalibration;
		private System.Windows.Forms.GroupBox groupBox6;
		private System.Windows.Forms.Button btnUploadAngleCorrection;
		private System.Windows.Forms.NumericUpDown udRollCorrection;
		private System.Windows.Forms.Label label41;
		private System.Windows.Forms.Label label42;
		private System.Windows.Forms.NumericUpDown udPitchCorrection;
		private System.Windows.Forms.GroupBox groupBox5;
		private Gauge gAccelXCal;
		private Gauge gAccelYCal;
		private Gauge gAccelZCal;
		private System.Windows.Forms.Label label39;
		private System.Windows.Forms.Label label38;
		private System.Windows.Forms.Label label37;
		private System.Windows.Forms.Label lblAccelCalFinal;
		private System.Windows.Forms.Button btnAccelCal1;
		private System.Windows.Forms.Label lblAccelCal4;
		private System.Windows.Forms.Button btnAccelCal2;
		private System.Windows.Forms.Label lblAccelCal3;
		private System.Windows.Forms.Button btnAccelCal3;
		private System.Windows.Forms.Label lblAccelCal2;
		private System.Windows.Forms.Button btnAccelCal4;
		private System.Windows.Forms.Label lblAccelCal1;
		private System.Windows.Forms.Button btnUploadAccelCal;
		private OrientationCube ocAccelOrient;
		private System.Windows.Forms.TrackBar tbRollPitchAngle;
		private System.Windows.Forms.Label label3;
		private System.Windows.Forms.TrackBar tbRollPitchSpeed;
		private System.Windows.Forms.Label label2;
		private System.Windows.Forms.Label lblRollPitchSpeed;
		private System.Windows.Forms.Label lblRollPitchAngle;
		private System.Windows.Forms.Button btnUploadRollPitch;
		private System.Windows.Forms.Label lblYawSpeed;
		private System.Windows.Forms.Label label5;
		private System.Windows.Forms.TrackBar tbYawSpeed;
		private System.Windows.Forms.Button btnControlReset;
		private System.Windows.Forms.NumericUpDown udHighThrottle;
		private System.Windows.Forms.NumericUpDown udArmedLowThrottle;
		private System.Windows.Forms.NumericUpDown udLowThrottle;
		private System.Windows.Forms.Label label7;
		private System.Windows.Forms.Label label6;
		private System.Windows.Forms.Label label4;
		private System.Windows.Forms.NumericUpDown udTestThrottle;
		private System.Windows.Forms.Label label8;
		private System.Windows.Forms.Button btnUploadThrottle;
		private System.Windows.Forms.GroupBox groupBox1;
		private System.Windows.Forms.CheckBox cbUseBatteryMonitor;
		private Elev8.Controls.ValueBar vbVoltage;
		private System.Windows.Forms.Label label9;
		private System.Windows.Forms.NumericUpDown udLowVoltageAlarm;
		private System.Windows.Forms.Label label12;
		private System.Windows.Forms.Label label10;
		private System.Windows.Forms.NumericUpDown udVoltageOffset;
		private Elev8.Controls.ValueBar vbVoltage2;
		private System.Windows.Forms.ComboBox cbDisarmDelay;
		private System.Windows.Forms.Label label15;
		private System.Windows.Forms.Label label16;
		private System.Windows.Forms.ComboBox cbArmingDelay;
		private System.Windows.Forms.CheckBox cbLowVoltageBuzzer;
		private System.Windows.Forms.Label lblThrustCorrection;
		private System.Windows.Forms.Label label17;
		private System.Windows.Forms.TrackBar tbThrustCorrection;
		private System.Windows.Forms.Label lblAccelCorrectionFilter;
		private System.Windows.Forms.Label label13;
		private System.Windows.Forms.TrackBar tbAccelCorrectionFilter;
	}
}

