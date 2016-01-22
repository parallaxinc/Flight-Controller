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
			this.tssGSVersion = new System.Windows.Forms.ToolStripStatusLabel();
			this.tssFCVersion = new System.Windows.Forms.ToolStripStatusLabel();
			this.tmCommTimer = new System.Windows.Forms.Timer( this.components );
			this.msMainMenu = new System.Windows.Forms.MenuStrip();
			this.settingsToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
			this.radioDisplayToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
			this.toolStripMenuItem2 = new System.Windows.Forms.ToolStripSeparator();
			this.miRadioMode1 = new System.Windows.Forms.ToolStripMenuItem();
			this.miRadioMode2 = new System.Windows.Forms.ToolStripMenuItem();
			this.toolStripMenuItem1 = new System.Windows.Forms.ToolStripSeparator();
			this.helpToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
			this.aboutToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
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
			this.tpAccelCalibration = new System.Windows.Forms.TabPage();
			this.groupBox5 = new System.Windows.Forms.GroupBox();
			this.btnUploadAccelCal = new System.Windows.Forms.Button();
			this.lblAccelCal1 = new System.Windows.Forms.Label();
			this.btnAccelCal4 = new System.Windows.Forms.Button();
			this.lblAccelCal2 = new System.Windows.Forms.Label();
			this.btnAccelCal3 = new System.Windows.Forms.Button();
			this.lblAccelCal3 = new System.Windows.Forms.Label();
			this.btnAccelCal2 = new System.Windows.Forms.Button();
			this.lblAccelCal4 = new System.Windows.Forms.Label();
			this.btnAccelCal1 = new System.Windows.Forms.Button();
			this.lblAccelCalFinal = new System.Windows.Forms.Label();
			this.label37 = new System.Windows.Forms.Label();
			this.label38 = new System.Windows.Forms.Label();
			this.label39 = new System.Windows.Forms.Label();
			this.gAccelZCal = new Elev8.Gauge();
			this.gAccelYCal = new Elev8.Gauge();
			this.gAccelXCal = new Elev8.Gauge();
			this.groupBox6 = new System.Windows.Forms.GroupBox();
			this.udPitchCorrection = new System.Windows.Forms.NumericUpDown();
			this.label42 = new System.Windows.Forms.Label();
			this.label41 = new System.Windows.Forms.Label();
			this.udRollCorrection = new System.Windows.Forms.NumericUpDown();
			this.btnUploadAngleCorrection = new System.Windows.Forms.Button();
			this.ocAccelOrient = new Elev8.OrientationCube();
			this.tpGyroCalibration = new System.Windows.Forms.TabPage();
			this.lfGraph = new Elev8.LineFit();
			this.groupBox12 = new System.Windows.Forms.GroupBox();
			this.gxScale = new System.Windows.Forms.Label();
			this.label28 = new System.Windows.Forms.Label();
			this.gxOffset = new System.Windows.Forms.Label();
			this.label26 = new System.Windows.Forms.Label();
			this.groupBox11 = new System.Windows.Forms.GroupBox();
			this.gyScale = new System.Windows.Forms.Label();
			this.label24 = new System.Windows.Forms.Label();
			this.gyOffset = new System.Windows.Forms.Label();
			this.label22 = new System.Windows.Forms.Label();
			this.groupBox10 = new System.Windows.Forms.GroupBox();
			this.gzScale = new System.Windows.Forms.Label();
			this.label20 = new System.Windows.Forms.Label();
			this.gzOffset = new System.Windows.Forms.Label();
			this.label18 = new System.Windows.Forms.Label();
			this.btnResetGyroCal = new System.Windows.Forms.Button();
			this.btnUploadGyroCalibration = new System.Windows.Forms.Button();
			this.gCalibX = new Elev8.Gauge();
			this.gCalibY = new Elev8.Gauge();
			this.gCalibZ = new Elev8.Gauge();
			this.gCalibTemp = new Elev8.Gauge();
			this.tpSystemSetup = new System.Windows.Forms.TabPage();
			this.groupBox1 = new System.Windows.Forms.GroupBox();
			this.udArmedLowThrottle = new System.Windows.Forms.NumericUpDown();
			this.udHighThrottle = new System.Windows.Forms.NumericUpDown();
			this.udLowThrottle = new System.Windows.Forms.NumericUpDown();
			this.label8 = new System.Windows.Forms.Label();
			this.label7 = new System.Windows.Forms.Label();
			this.cbDisarmDelay = new System.Windows.Forms.ComboBox();
			this.udTestThrottle = new System.Windows.Forms.NumericUpDown();
			this.label15 = new System.Windows.Forms.Label();
			this.cbArmingDelay = new System.Windows.Forms.ComboBox();
			this.label4 = new System.Windows.Forms.Label();
			this.label16 = new System.Windows.Forms.Label();
			this.label6 = new System.Windows.Forms.Label();
			this.cbDisableMotors = new System.Windows.Forms.CheckBox();
			this.btnUploadThrottle = new System.Windows.Forms.Button();
			this.btnFactoryDefaultPrefs = new System.Windows.Forms.Button();
			this.groupBox2 = new System.Windows.Forms.GroupBox();
			this.label10 = new System.Windows.Forms.Label();
			this.udVoltageOffset = new System.Windows.Forms.NumericUpDown();
			this.label12 = new System.Windows.Forms.Label();
			this.label9 = new System.Windows.Forms.Label();
			this.udLowVoltageAlarmThreshold = new System.Windows.Forms.NumericUpDown();
			this.cbLowVoltageAlarm = new System.Windows.Forms.CheckBox();
			this.cbUseBatteryMonitor = new System.Windows.Forms.CheckBox();
			this.vbVoltage2 = new Elev8.Controls.ValueBar();
			this.tpControllerSetup = new System.Windows.Forms.TabPage();
			this.groupBox3 = new System.Windows.Forms.GroupBox();
			this.label21 = new System.Windows.Forms.Label();
			this.lblPitchGain = new System.Windows.Forms.Label();
			this.hsYawGain = new System.Windows.Forms.HScrollBar();
			this.lblRollGain = new System.Windows.Forms.Label();
			this.label19 = new System.Windows.Forms.Label();
			this.lblYawGain = new System.Windows.Forms.Label();
			this.label11 = new System.Windows.Forms.Label();
			this.hsAscentGain = new System.Windows.Forms.HScrollBar();
			this.hsRollGain = new System.Windows.Forms.HScrollBar();
			this.label25 = new System.Windows.Forms.Label();
			this.hsPitchGain = new System.Windows.Forms.HScrollBar();
			this.lblAscentGain = new System.Windows.Forms.Label();
			this.cbPitchRollLocked = new System.Windows.Forms.CheckBox();
			this.hsAltiGain = new System.Windows.Forms.HScrollBar();
			this.label27 = new System.Windows.Forms.Label();
			this.lblAltiGain = new System.Windows.Forms.Label();
			this.groupBox4 = new System.Windows.Forms.GroupBox();
			this.cbEnableAdvanced = new System.Windows.Forms.CheckBox();
			this.btnUploadFlightChanges = new System.Windows.Forms.Button();
			this.tpRadioSetup = new System.Windows.Forms.TabPage();
			this.rsR_Left = new Elev8.RadioStick();
			this.rsR_Right = new Elev8.RadioStick();
			this.vbR_LS_YValue = new Elev8.Controls.ValueBar();
			this.vbR_LS_XValue = new Elev8.Controls.ValueBar();
			this.vbR_RS_YValue = new Elev8.Controls.ValueBar();
			this.vbR_RS_XValue = new Elev8.Controls.ValueBar();
			this.vbR_Channel5 = new Elev8.Controls.ValueBar();
			this.vbR_Channel6 = new Elev8.Controls.ValueBar();
			this.vbR_Channel7 = new Elev8.Controls.ValueBar();
			this.vbR_Channel8 = new Elev8.Controls.ValueBar();
			this.cbChannel1 = new System.Windows.Forms.ComboBox();
			this.cbChannel4 = new System.Windows.Forms.ComboBox();
			this.cbChannel5 = new System.Windows.Forms.ComboBox();
			this.cbChannel6 = new System.Windows.Forms.ComboBox();
			this.cbChannel3 = new System.Windows.Forms.ComboBox();
			this.cbChannel2 = new System.Windows.Forms.ComboBox();
			this.cbChannel7 = new System.Windows.Forms.ComboBox();
			this.cbChannel8 = new System.Windows.Forms.ComboBox();
			this.cbRev1 = new System.Windows.Forms.CheckBox();
			this.cbRev4 = new System.Windows.Forms.CheckBox();
			this.cbRev5 = new System.Windows.Forms.CheckBox();
			this.cbRev6 = new System.Windows.Forms.CheckBox();
			this.cbRev3 = new System.Windows.Forms.CheckBox();
			this.cbRev2 = new System.Windows.Forms.CheckBox();
			this.cbRev7 = new System.Windows.Forms.CheckBox();
			this.cbRev8 = new System.Windows.Forms.CheckBox();
			this.cbReceiverType = new System.Windows.Forms.ComboBox();
			this.label1 = new System.Windows.Forms.Label();
			this.btnCalibrate = new System.Windows.Forms.Button();
			this.tbRollPitchAuto = new System.Windows.Forms.TrackBar();
			this.label2 = new System.Windows.Forms.Label();
			this.tbRollPitchManual = new System.Windows.Forms.TrackBar();
			this.label3 = new System.Windows.Forms.Label();
			this.lblRollPitchAngle = new System.Windows.Forms.Label();
			this.lblRollPitchManual = new System.Windows.Forms.Label();
			this.btnUploadRollPitch = new System.Windows.Forms.Button();
			this.tbYawSpeedAuto = new System.Windows.Forms.TrackBar();
			this.label5 = new System.Windows.Forms.Label();
			this.lblYawSpeedAuto = new System.Windows.Forms.Label();
			this.btnControlReset = new System.Windows.Forms.Button();
			this.tbThrustCorrection = new System.Windows.Forms.TrackBar();
			this.label17 = new System.Windows.Forms.Label();
			this.lblThrustCorrection = new System.Windows.Forms.Label();
			this.tbAccelCorrectionFilter = new System.Windows.Forms.TrackBar();
			this.label13 = new System.Windows.Forms.Label();
			this.lblAccelCorrectionFilter = new System.Windows.Forms.Label();
			this.tbYawSpeedManual = new System.Windows.Forms.TrackBar();
			this.label14 = new System.Windows.Forms.Label();
			this.lblYawSpeedManual = new System.Windows.Forms.Label();
			this.tbCalibrateDocs = new System.Windows.Forms.TextBox();
			this.tpSysTest = new System.Windows.Forms.TabPage();
			this.btnMotor1 = new System.Windows.Forms.Button();
			this.btnMotor2 = new System.Windows.Forms.Button();
			this.btnMotor3 = new System.Windows.Forms.Button();
			this.btnMotor4 = new System.Windows.Forms.Button();
			this.textBox3 = new System.Windows.Forms.TextBox();
			this.btnBeeper = new System.Windows.Forms.Button();
			this.btnLED = new System.Windows.Forms.Button();
			this.btnThrottleCalibrate = new System.Windows.Forms.Button();
			this.lblCalibrateDocs = new System.Windows.Forms.Label();
			this.tpSensors = new System.Windows.Forms.TabPage();
			this.plotSensors = new GraphLib.PlotterDisplayEx();
			this.cbGyroX = new System.Windows.Forms.CheckBox();
			this.cbGyroY = new System.Windows.Forms.CheckBox();
			this.cbGyroZ = new System.Windows.Forms.CheckBox();
			this.cbAccelX = new System.Windows.Forms.CheckBox();
			this.cbAccelY = new System.Windows.Forms.CheckBox();
			this.cbAccelZ = new System.Windows.Forms.CheckBox();
			this.cbMagX = new System.Windows.Forms.CheckBox();
			this.cbMagY = new System.Windows.Forms.CheckBox();
			this.cbMagZ = new System.Windows.Forms.CheckBox();
			this.cbAltiEst = new System.Windows.Forms.CheckBox();
			this.cbPitch = new System.Windows.Forms.CheckBox();
			this.cbRoll = new System.Windows.Forms.CheckBox();
			this.cbYaw = new System.Windows.Forms.CheckBox();
			this.cbVoltage = new System.Windows.Forms.CheckBox();
			this.cbAltiBaro = new System.Windows.Forms.CheckBox();
			this.tpStatus = new System.Windows.Forms.TabPage();
			this.aicAltimeter = new Elev8.Altimeter();
			this.aicHeading = new Elev8.HeadingIndicator();
			this.ocOrientation = new Elev8.OrientationCube();
			this.aicAttitude = new Elev8.AttitudeIndicator();
			this.rsLeft = new Elev8.RadioStick();
			this.rsRight = new Elev8.RadioStick();
			this.vbLS_YValue = new Elev8.Controls.ValueBar();
			this.vbLS_XValue = new Elev8.Controls.ValueBar();
			this.vbRS_YValue = new Elev8.Controls.ValueBar();
			this.vbRS_XValue = new Elev8.Controls.ValueBar();
			this.vbChannel5 = new Elev8.Controls.ValueBar();
			this.vbChannel6 = new Elev8.Controls.ValueBar();
			this.vbChannel7 = new Elev8.Controls.ValueBar();
			this.vbChannel8 = new Elev8.Controls.ValueBar();
			this.vbVoltage = new Elev8.Controls.ValueBar();
			this.vbFrontRight = new Elev8.Controls.ValueBar();
			this.vbBackRight = new Elev8.Controls.ValueBar();
			this.vbFrontLeft = new Elev8.Controls.ValueBar();
			this.vbBackLeft = new Elev8.Controls.ValueBar();
			this.vbPitchOut = new Elev8.Controls.ValueBar();
			this.vbYawOut = new Elev8.Controls.ValueBar();
			this.vbRollOut = new Elev8.Controls.ValueBar();
			this.lblCycles = new System.Windows.Forms.Label();
			this.label23 = new System.Windows.Forms.Label();
			this.tcTabs = new System.Windows.Forms.TabControl();
			this.stMainStatus.SuspendLayout();
			this.msMainMenu.SuspendLayout();
			this.tpAccelCalibration.SuspendLayout();
			this.groupBox5.SuspendLayout();
			this.groupBox6.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)(this.udPitchCorrection)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udRollCorrection)).BeginInit();
			this.tpGyroCalibration.SuspendLayout();
			this.groupBox12.SuspendLayout();
			this.groupBox11.SuspendLayout();
			this.groupBox10.SuspendLayout();
			this.tpSystemSetup.SuspendLayout();
			this.groupBox1.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)(this.udArmedLowThrottle)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udHighThrottle)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udLowThrottle)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udTestThrottle)).BeginInit();
			this.groupBox2.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)(this.udVoltageOffset)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.udLowVoltageAlarmThreshold)).BeginInit();
			this.tpControllerSetup.SuspendLayout();
			this.groupBox3.SuspendLayout();
			this.tpRadioSetup.SuspendLayout();
			((System.ComponentModel.ISupportInitialize)(this.tbRollPitchAuto)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbRollPitchManual)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbYawSpeedAuto)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbThrustCorrection)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbAccelCorrectionFilter)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.tbYawSpeedManual)).BeginInit();
			this.tpSysTest.SuspendLayout();
			this.tpSensors.SuspendLayout();
			this.tpStatus.SuspendLayout();
			this.tcTabs.SuspendLayout();
			this.SuspendLayout();
			// 
			// stMainStatus
			// 
			this.stMainStatus.Items.AddRange( new System.Windows.Forms.ToolStripItem[] {
            this.tssStatusText,
            this.tssGSVersion,
            this.tssFCVersion} );
			this.stMainStatus.Location = new System.Drawing.Point( 0, 406 );
			this.stMainStatus.Name = "stMainStatus";
			this.stMainStatus.Size = new System.Drawing.Size( 729, 22 );
			this.stMainStatus.TabIndex = 0;
			this.stMainStatus.Text = "status";
			// 
			// tssStatusText
			// 
			this.tssStatusText.AutoSize = false;
			this.tssStatusText.Name = "tssStatusText";
			this.tssStatusText.Size = new System.Drawing.Size( 250, 17 );
			this.tssStatusText.Text = "Connecting...";
			this.tssStatusText.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
			// 
			// tssGSVersion
			// 
			this.tssGSVersion.AutoSize = false;
			this.tssGSVersion.Name = "tssGSVersion";
			this.tssGSVersion.Size = new System.Drawing.Size( 240, 17 );
			this.tssGSVersion.Text = "GroundStation Version 1.0";
			this.tssGSVersion.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
			// 
			// tssFCVersion
			// 
			this.tssFCVersion.AutoSize = false;
			this.tssFCVersion.Name = "tssFCVersion";
			this.tssFCVersion.Size = new System.Drawing.Size( 150, 17 );
			this.tssFCVersion.Text = "Firmware Version -.-";
			// 
			// tmCommTimer
			// 
			this.tmCommTimer.Enabled = true;
			this.tmCommTimer.Interval = 50;
			this.tmCommTimer.Tick += new System.EventHandler( this.tmCommTimer_Tick );
			// 
			// msMainMenu
			// 
			this.msMainMenu.Items.AddRange( new System.Windows.Forms.ToolStripItem[] {
            this.settingsToolStripMenuItem,
            this.helpToolStripMenuItem} );
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
			// helpToolStripMenuItem
			// 
			this.helpToolStripMenuItem.DropDownItems.AddRange( new System.Windows.Forms.ToolStripItem[] {
            this.aboutToolStripMenuItem} );
			this.helpToolStripMenuItem.Name = "helpToolStripMenuItem";
			this.helpToolStripMenuItem.Size = new System.Drawing.Size( 44, 20 );
			this.helpToolStripMenuItem.Text = "Help";
			// 
			// aboutToolStripMenuItem
			// 
			this.aboutToolStripMenuItem.Name = "aboutToolStripMenuItem";
			this.aboutToolStripMenuItem.Size = new System.Drawing.Size( 107, 22 );
			this.aboutToolStripMenuItem.Text = "About";
			this.aboutToolStripMenuItem.Click += new System.EventHandler( this.aboutToolStripMenuItem_Click );
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
			// valueBar1
			// 
			this.valueBar1.BarColor = System.Drawing.Color.LightGreen;
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
			this.valueBar2.BarColor = System.Drawing.Color.LightGreen;
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
			this.valueBar3.BarColor = System.Drawing.Color.LightGreen;
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
			this.valueBar4.BarColor = System.Drawing.Color.LightGreen;
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
			this.valueBar5.BarColor = System.Drawing.Color.LightGreen;
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
			this.valueBar6.BarColor = System.Drawing.Color.LightGreen;
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
			this.valueBar7.BarColor = System.Drawing.Color.LightGreen;
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
			this.valueBar8.BarColor = System.Drawing.Color.LightGreen;
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
			// btnUploadAccelCal
			// 
			this.btnUploadAccelCal.Location = new System.Drawing.Point( 8, 273 );
			this.btnUploadAccelCal.Name = "btnUploadAccelCal";
			this.btnUploadAccelCal.Size = new System.Drawing.Size( 75, 23 );
			this.btnUploadAccelCal.TabIndex = 22;
			this.btnUploadAccelCal.Text = "Upload";
			this.tlToolTip.SetToolTip( this.btnUploadAccelCal, "Upload new accelerometer calibration data" );
			this.btnUploadAccelCal.UseVisualStyleBackColor = true;
			this.btnUploadAccelCal.Click += new System.EventHandler( this.btnUploadAccelCal_Click );
			// 
			// lblAccelCal1
			// 
			this.lblAccelCal1.Location = new System.Drawing.Point( 100, 154 );
			this.lblAccelCal1.Name = "lblAccelCal1";
			this.lblAccelCal1.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCal1.TabIndex = 23;
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
			// lblAccelCal2
			// 
			this.lblAccelCal2.Location = new System.Drawing.Point( 100, 183 );
			this.lblAccelCal2.Name = "lblAccelCal2";
			this.lblAccelCal2.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCal2.TabIndex = 24;
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
			// lblAccelCal3
			// 
			this.lblAccelCal3.Location = new System.Drawing.Point( 100, 212 );
			this.lblAccelCal3.Name = "lblAccelCal3";
			this.lblAccelCal3.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCal3.TabIndex = 25;
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
			// lblAccelCal4
			// 
			this.lblAccelCal4.Location = new System.Drawing.Point( 100, 241 );
			this.lblAccelCal4.Name = "lblAccelCal4";
			this.lblAccelCal4.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCal4.TabIndex = 26;
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
			// lblAccelCalFinal
			// 
			this.lblAccelCalFinal.Location = new System.Drawing.Point( 100, 278 );
			this.lblAccelCalFinal.Name = "lblAccelCalFinal";
			this.lblAccelCalFinal.Size = new System.Drawing.Size( 147, 18 );
			this.lblAccelCalFinal.TabIndex = 27;
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
			// label38
			// 
			this.label38.AutoSize = true;
			this.label38.Location = new System.Drawing.Point( 100, 111 );
			this.label38.Name = "label38";
			this.label38.Size = new System.Drawing.Size( 44, 13 );
			this.label38.TabIndex = 16;
			this.label38.Text = "Accel Y";
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
			// gAccelZCal
			// 
			this.gAccelZCal.AverageCount = 200;
			this.gAccelZCal.Font = new System.Drawing.Font( "Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)) );
			this.gAccelZCal.Location = new System.Drawing.Point( 200, 19 );
			this.gAccelZCal.Name = "gAccelZCal";
			this.gAccelZCal.Range = 32768F;
			this.gAccelZCal.Size = new System.Drawing.Size( 91, 89 );
			this.gAccelZCal.TabIndex = 14;
			this.gAccelZCal.Value = 0F;
			// 
			// gAccelYCal
			// 
			this.gAccelYCal.AverageCount = 200;
			this.gAccelYCal.Font = new System.Drawing.Font( "Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)) );
			this.gAccelYCal.Location = new System.Drawing.Point( 103, 19 );
			this.gAccelYCal.Name = "gAccelYCal";
			this.gAccelYCal.Range = 32768F;
			this.gAccelYCal.Size = new System.Drawing.Size( 91, 89 );
			this.gAccelYCal.TabIndex = 13;
			this.gAccelYCal.Value = 0F;
			// 
			// gAccelXCal
			// 
			this.gAccelXCal.AverageCount = 200;
			this.gAccelXCal.Font = new System.Drawing.Font( "Microsoft Sans Serif", 6.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)) );
			this.gAccelXCal.Location = new System.Drawing.Point( 6, 19 );
			this.gAccelXCal.Name = "gAccelXCal";
			this.gAccelXCal.Range = 32768F;
			this.gAccelXCal.Size = new System.Drawing.Size( 91, 89 );
			this.gAccelXCal.TabIndex = 12;
			this.gAccelXCal.Value = 0F;
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
			// label42
			// 
			this.label42.AutoSize = true;
			this.label42.Location = new System.Drawing.Point( 47, 63 );
			this.label42.Name = "label42";
			this.label42.Size = new System.Drawing.Size( 112, 13 );
			this.label42.TabIndex = 31;
			this.label42.Text = "Pitch Angle Correction";
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
			// btnUploadAngleCorrection
			// 
			this.btnUploadAngleCorrection.Location = new System.Drawing.Point( 114, 102 );
			this.btnUploadAngleCorrection.Name = "btnUploadAngleCorrection";
			this.btnUploadAngleCorrection.Size = new System.Drawing.Size( 75, 23 );
			this.btnUploadAngleCorrection.TabIndex = 32;
			this.btnUploadAngleCorrection.Text = "Upload";
			this.tlToolTip.SetToolTip( this.btnUploadAngleCorrection, "Upload new pitch and roll angle correction data" );
			this.btnUploadAngleCorrection.UseVisualStyleBackColor = true;
			this.btnUploadAngleCorrection.Click += new System.EventHandler( this.btnUploadAngleCorrection_Click );
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
			// lfGraph
			// 
			this.lfGraph.Location = new System.Drawing.Point( 224, 19 );
			this.lfGraph.Name = "lfGraph";
			this.lfGraph.Size = new System.Drawing.Size( 447, 232 );
			this.lfGraph.TabIndex = 29;
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
			// gxScale
			// 
			this.gxScale.Location = new System.Drawing.Point( 47, 16 );
			this.gxScale.Name = "gxScale";
			this.gxScale.Size = new System.Drawing.Size( 36, 13 );
			this.gxScale.TabIndex = 5;
			this.gxScale.Text = "0";
			this.gxScale.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
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
			// gxOffset
			// 
			this.gxOffset.Location = new System.Drawing.Point( 133, 16 );
			this.gxOffset.Name = "gxOffset";
			this.gxOffset.Size = new System.Drawing.Size( 36, 13 );
			this.gxOffset.TabIndex = 6;
			this.gxOffset.Text = "0";
			this.gxOffset.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
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
			// gyScale
			// 
			this.gyScale.Location = new System.Drawing.Point( 47, 14 );
			this.gyScale.Name = "gyScale";
			this.gyScale.Size = new System.Drawing.Size( 36, 13 );
			this.gyScale.TabIndex = 9;
			this.gyScale.Text = "0";
			this.gyScale.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
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
			// gyOffset
			// 
			this.gyOffset.Location = new System.Drawing.Point( 133, 14 );
			this.gyOffset.Name = "gyOffset";
			this.gyOffset.Size = new System.Drawing.Size( 36, 13 );
			this.gyOffset.TabIndex = 10;
			this.gyOffset.Text = "0";
			this.gyOffset.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
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
			// gzScale
			// 
			this.gzScale.Location = new System.Drawing.Point( 47, 14 );
			this.gzScale.Name = "gzScale";
			this.gzScale.Size = new System.Drawing.Size( 36, 13 );
			this.gzScale.TabIndex = 9;
			this.gzScale.Text = "0";
			this.gzScale.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
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
			// gzOffset
			// 
			this.gzOffset.Location = new System.Drawing.Point( 132, 14 );
			this.gzOffset.Name = "gzOffset";
			this.gzOffset.Size = new System.Drawing.Size( 36, 13 );
			this.gzOffset.TabIndex = 10;
			this.gzOffset.Text = "0";
			this.gzOffset.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
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
			// btnResetGyroCal
			// 
			this.btnResetGyroCal.Location = new System.Drawing.Point( 60, 176 );
			this.btnResetGyroCal.Name = "btnResetGyroCal";
			this.btnResetGyroCal.Size = new System.Drawing.Size( 147, 23 );
			this.btnResetGyroCal.TabIndex = 33;
			this.btnResetGyroCal.Text = "Restart Calibration";
			this.tlToolTip.SetToolTip( this.btnResetGyroCal, "Reset current calibration data (local only, not sent to firmware)" );
			this.btnResetGyroCal.UseVisualStyleBackColor = true;
			this.btnResetGyroCal.Click += new System.EventHandler( this.btnResetCalib_Click );
			// 
			// btnUploadGyroCalibration
			// 
			this.btnUploadGyroCalibration.Location = new System.Drawing.Point( 61, 228 );
			this.btnUploadGyroCalibration.Name = "btnUploadGyroCalibration";
			this.btnUploadGyroCalibration.Size = new System.Drawing.Size( 147, 23 );
			this.btnUploadGyroCalibration.TabIndex = 34;
			this.btnUploadGyroCalibration.Text = "Upload Calibration Settings";
			this.tlToolTip.SetToolTip( this.btnUploadGyroCalibration, "Upload changes on this page to the flight controller" );
			this.btnUploadGyroCalibration.UseVisualStyleBackColor = true;
			this.btnUploadGyroCalibration.Click += new System.EventHandler( this.btnUploadCalibration_Click );
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
			// tpSystemSetup
			// 
			this.tpSystemSetup.Controls.Add( this.groupBox2 );
			this.tpSystemSetup.Controls.Add( this.btnFactoryDefaultPrefs );
			this.tpSystemSetup.Controls.Add( this.btnUploadThrottle );
			this.tpSystemSetup.Controls.Add( this.groupBox1 );
			this.tpSystemSetup.Location = new System.Drawing.Point( 4, 22 );
			this.tpSystemSetup.Name = "tpSystemSetup";
			this.tpSystemSetup.Size = new System.Drawing.Size( 721, 356 );
			this.tpSystemSetup.TabIndex = 4;
			this.tpSystemSetup.Text = "System Setup";
			// 
			// groupBox1
			// 
			this.groupBox1.Controls.Add( this.cbDisableMotors );
			this.groupBox1.Controls.Add( this.label6 );
			this.groupBox1.Controls.Add( this.label16 );
			this.groupBox1.Controls.Add( this.label4 );
			this.groupBox1.Controls.Add( this.cbArmingDelay );
			this.groupBox1.Controls.Add( this.label15 );
			this.groupBox1.Controls.Add( this.udTestThrottle );
			this.groupBox1.Controls.Add( this.cbDisarmDelay );
			this.groupBox1.Controls.Add( this.label7 );
			this.groupBox1.Controls.Add( this.label8 );
			this.groupBox1.Controls.Add( this.udLowThrottle );
			this.groupBox1.Controls.Add( this.udHighThrottle );
			this.groupBox1.Controls.Add( this.udArmedLowThrottle );
			this.groupBox1.Location = new System.Drawing.Point( 8, 14 );
			this.groupBox1.Name = "groupBox1";
			this.groupBox1.Size = new System.Drawing.Size( 202, 214 );
			this.groupBox1.TabIndex = 7;
			this.groupBox1.TabStop = false;
			this.groupBox1.Text = "Motor Outputs";
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
			// label8
			// 
			this.label8.AutoSize = true;
			this.label8.Location = new System.Drawing.Point( 44, 70 );
			this.label8.Name = "label8";
			this.label8.Size = new System.Drawing.Size( 67, 13 );
			this.label8.TabIndex = 6;
			this.label8.Text = "Test Throttle";
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
			// cbDisarmDelay
			// 
			this.cbDisarmDelay.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbDisarmDelay.FormattingEnabled = true;
			this.cbDisarmDelay.Items.AddRange( new object[] {
            "1.00 sec",
            "0.50 sec",
            "0.25 sec",
            "Off"} );
			this.cbDisarmDelay.Location = new System.Drawing.Point( 104, 183 );
			this.cbDisarmDelay.Name = "cbDisarmDelay";
			this.cbDisarmDelay.Size = new System.Drawing.Size( 65, 21 );
			this.cbDisarmDelay.TabIndex = 35;
			this.tlToolTip.SetToolTip( this.cbDisarmDelay, "How long you have to hold the sticks in the disarm position to disarm the motors" );
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
			// label15
			// 
			this.label15.AutoSize = true;
			this.label15.Location = new System.Drawing.Point( 29, 186 );
			this.label15.Name = "label15";
			this.label15.Size = new System.Drawing.Size( 69, 13 );
			this.label15.TabIndex = 36;
			this.label15.Text = "Disarm Delay";
			this.tlToolTip.SetToolTip( this.label15, "How long you have to hold the sticks in the disarm position to disarm the motors" );
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
			this.cbArmingDelay.Location = new System.Drawing.Point( 104, 157 );
			this.cbArmingDelay.Name = "cbArmingDelay";
			this.cbArmingDelay.Size = new System.Drawing.Size( 65, 21 );
			this.cbArmingDelay.TabIndex = 37;
			this.tlToolTip.SetToolTip( this.cbArmingDelay, "How long you have to hold the sticks in the arming position to arm the motors" );
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
			// label16
			// 
			this.label16.AutoSize = true;
			this.label16.Location = new System.Drawing.Point( 29, 160 );
			this.label16.Name = "label16";
			this.label16.Size = new System.Drawing.Size( 69, 13 );
			this.label16.TabIndex = 38;
			this.label16.Text = "Arming Delay";
			this.tlToolTip.SetToolTip( this.label16, "How long you have to hold the sticks in the arming position to arm the motors" );
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
			// cbDisableMotors
			// 
			this.cbDisableMotors.Appearance = System.Windows.Forms.Appearance.Button;
			this.cbDisableMotors.AutoSize = true;
			this.cbDisableMotors.FlatAppearance.CheckedBackColor = System.Drawing.Color.FromArgb( ((int)(((byte)(255)))), ((int)(((byte)(192)))), ((int)(((byte)(192)))) );
			this.cbDisableMotors.Location = new System.Drawing.Point( 104, 118 );
			this.cbDisableMotors.Name = "cbDisableMotors";
			this.cbDisableMotors.Size = new System.Drawing.Size( 87, 23 );
			this.cbDisableMotors.TabIndex = 7;
			this.cbDisableMotors.Text = "Disable Motors";
			this.tlToolTip.SetToolTip( this.cbDisableMotors, "Used to completely disable motor outputs for testing." );
			this.cbDisableMotors.UseVisualStyleBackColor = true;
			this.cbDisableMotors.CheckedChanged += new System.EventHandler( this.cbDisableMotors_CheckedChanged );
			// 
			// btnUploadThrottle
			// 
			this.btnUploadThrottle.Location = new System.Drawing.Point( 259, 190 );
			this.btnUploadThrottle.Name = "btnUploadThrottle";
			this.btnUploadThrottle.Size = new System.Drawing.Size( 130, 23 );
			this.btnUploadThrottle.TabIndex = 4;
			this.btnUploadThrottle.Text = "Upload Changes";
			this.tlToolTip.SetToolTip( this.btnUploadThrottle, "Upload changes on this page to the flight controller" );
			this.btnUploadThrottle.UseVisualStyleBackColor = true;
			this.btnUploadThrottle.Click += new System.EventHandler( this.btnUploadThrottle_Click );
			// 
			// btnFactoryDefaultPrefs
			// 
			this.btnFactoryDefaultPrefs.Location = new System.Drawing.Point( 574, 330 );
			this.btnFactoryDefaultPrefs.Name = "btnFactoryDefaultPrefs";
			this.btnFactoryDefaultPrefs.Size = new System.Drawing.Size( 139, 23 );
			this.btnFactoryDefaultPrefs.TabIndex = 40;
			this.btnFactoryDefaultPrefs.Text = "Factory Default Prefs";
			this.tlToolTip.SetToolTip( this.btnFactoryDefaultPrefs, "Reset ALL SETTINGS to their factory defaults. and send to firmware." );
			this.btnFactoryDefaultPrefs.UseVisualStyleBackColor = true;
			this.btnFactoryDefaultPrefs.Click += new System.EventHandler( this.btnFactoryDefaultPrefs_Click );
			// 
			// groupBox2
			// 
			this.groupBox2.Controls.Add( this.vbVoltage2 );
			this.groupBox2.Controls.Add( this.cbUseBatteryMonitor );
			this.groupBox2.Controls.Add( this.cbLowVoltageAlarm );
			this.groupBox2.Controls.Add( this.udLowVoltageAlarmThreshold );
			this.groupBox2.Controls.Add( this.label9 );
			this.groupBox2.Controls.Add( this.label12 );
			this.groupBox2.Controls.Add( this.udVoltageOffset );
			this.groupBox2.Controls.Add( this.label10 );
			this.groupBox2.Location = new System.Drawing.Point( 216, 14 );
			this.groupBox2.Name = "groupBox2";
			this.groupBox2.Size = new System.Drawing.Size( 200, 159 );
			this.groupBox2.TabIndex = 41;
			this.groupBox2.TabStop = false;
			this.groupBox2.Text = "Battery Monitor";
			// 
			// label10
			// 
			this.label10.AutoSize = true;
			this.label10.Location = new System.Drawing.Point( 36, 95 );
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
			this.udVoltageOffset.Location = new System.Drawing.Point( 116, 93 );
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
			// 
			// label12
			// 
			this.label12.AutoSize = true;
			this.label12.Location = new System.Drawing.Point( 35, 139 );
			this.label12.Name = "label12";
			this.label12.Size = new System.Drawing.Size( 111, 13 );
			this.label12.TabIndex = 13;
			this.label12.Text = "TODO: Ascent limiting";
			this.label12.Visible = false;
			// 
			// label9
			// 
			this.label9.AutoSize = true;
			this.label9.Location = new System.Drawing.Point( 15, 71 );
			this.label9.Name = "label9";
			this.label9.Size = new System.Drawing.Size( 95, 13 );
			this.label9.TabIndex = 10;
			this.label9.Text = "Low Voltage Alarm";
			// 
			// udLowVoltageAlarmThreshold
			// 
			this.udLowVoltageAlarmThreshold.DecimalPlaces = 2;
			this.udLowVoltageAlarmThreshold.Increment = new decimal( new int[] {
            1,
            0,
            0,
            65536} );
			this.udLowVoltageAlarmThreshold.Location = new System.Drawing.Point( 116, 69 );
			this.udLowVoltageAlarmThreshold.Maximum = new decimal( new int[] {
            168,
            0,
            0,
            65536} );
			this.udLowVoltageAlarmThreshold.Minimum = new decimal( new int[] {
            60,
            0,
            0,
            65536} );
			this.udLowVoltageAlarmThreshold.Name = "udLowVoltageAlarmThreshold";
			this.udLowVoltageAlarmThreshold.Size = new System.Drawing.Size( 57, 20 );
			this.udLowVoltageAlarmThreshold.TabIndex = 9;
			this.tlToolTip.SetToolTip( this.udLowVoltageAlarmThreshold, "LED and audio warning of low voltage when battery is below this value" );
			this.udLowVoltageAlarmThreshold.Value = new decimal( new int[] {
            60,
            0,
            0,
            65536} );
			// 
			// cbLowVoltageAlarm
			// 
			this.cbLowVoltageAlarm.AutoSize = true;
			this.cbLowVoltageAlarm.CheckAlign = System.Drawing.ContentAlignment.MiddleRight;
			this.cbLowVoltageAlarm.Location = new System.Drawing.Point( 19, 118 );
			this.cbLowVoltageAlarm.Name = "cbLowVoltageAlarm";
			this.cbLowVoltageAlarm.Size = new System.Drawing.Size( 152, 17 );
			this.cbLowVoltageAlarm.TabIndex = 39;
			this.cbLowVoltageAlarm.Text = "Audible Low Voltage Alarm";
			this.tlToolTip.SetToolTip( this.cbLowVoltageAlarm, "Enable / Disable audible low-voltage buzzer" );
			this.cbLowVoltageAlarm.UseVisualStyleBackColor = true;
			// 
			// cbUseBatteryMonitor
			// 
			this.cbUseBatteryMonitor.AutoSize = true;
			this.cbUseBatteryMonitor.CheckAlign = System.Drawing.ContentAlignment.MiddleRight;
			this.cbUseBatteryMonitor.Location = new System.Drawing.Point( 38, 46 );
			this.cbUseBatteryMonitor.Name = "cbUseBatteryMonitor";
			this.cbUseBatteryMonitor.Size = new System.Drawing.Size( 133, 17 );
			this.cbUseBatteryMonitor.TabIndex = 8;
			this.cbUseBatteryMonitor.Text = "Enable Battery Monitor";
			this.tlToolTip.SetToolTip( this.cbUseBatteryMonitor, "Enable / disable monitoring of battery voltage" );
			this.cbUseBatteryMonitor.UseVisualStyleBackColor = true;
			// 
			// vbVoltage2
			// 
			this.vbVoltage2.BarColor = System.Drawing.Color.LightGreen;
			this.vbVoltage2.FromLeft = true;
			this.vbVoltage2.LeftLabel = "Battery Voltage";
			this.vbVoltage2.Location = new System.Drawing.Point( 19, 17 );
			this.vbVoltage2.MaxValue = 1260;
			this.vbVoltage2.MinValue = 900;
			this.vbVoltage2.Name = "vbVoltage2";
			this.vbVoltage2.RightLabel = "0";
			this.vbVoltage2.Size = new System.Drawing.Size( 150, 20 );
			this.vbVoltage2.TabIndex = 34;
			this.vbVoltage2.Value = 1200;
			// 
			// tpControllerSetup
			// 
			this.tpControllerSetup.Controls.Add( this.btnUploadFlightChanges );
			this.tpControllerSetup.Controls.Add( this.cbEnableAdvanced );
			this.tpControllerSetup.Controls.Add( this.groupBox4 );
			this.tpControllerSetup.Controls.Add( this.groupBox3 );
			this.tpControllerSetup.Location = new System.Drawing.Point( 4, 22 );
			this.tpControllerSetup.Name = "tpControllerSetup";
			this.tpControllerSetup.Size = new System.Drawing.Size( 721, 356 );
			this.tpControllerSetup.TabIndex = 7;
			this.tpControllerSetup.Text = "Flight Control Setup";
			this.tlToolTip.SetToolTip( this.tpControllerSetup, "Gain value specifies how forcefully the controller responds to outside influence." +
					"  Higher numbers mean tighter control, but too high can cause instability." );
			// 
			// groupBox3
			// 
			this.groupBox3.Controls.Add( this.lblAltiGain );
			this.groupBox3.Controls.Add( this.label27 );
			this.groupBox3.Controls.Add( this.hsAltiGain );
			this.groupBox3.Controls.Add( this.cbPitchRollLocked );
			this.groupBox3.Controls.Add( this.lblAscentGain );
			this.groupBox3.Controls.Add( this.hsPitchGain );
			this.groupBox3.Controls.Add( this.label25 );
			this.groupBox3.Controls.Add( this.hsRollGain );
			this.groupBox3.Controls.Add( this.hsAscentGain );
			this.groupBox3.Controls.Add( this.label11 );
			this.groupBox3.Controls.Add( this.lblYawGain );
			this.groupBox3.Controls.Add( this.label19 );
			this.groupBox3.Controls.Add( this.lblRollGain );
			this.groupBox3.Controls.Add( this.hsYawGain );
			this.groupBox3.Controls.Add( this.lblPitchGain );
			this.groupBox3.Controls.Add( this.label21 );
			this.groupBox3.Location = new System.Drawing.Point( 8, 10 );
			this.groupBox3.Name = "groupBox3";
			this.groupBox3.Size = new System.Drawing.Size( 331, 149 );
			this.groupBox3.TabIndex = 14;
			this.groupBox3.TabStop = false;
			this.groupBox3.Text = "Simple Settings";
			// 
			// label21
			// 
			this.label21.AutoSize = true;
			this.label21.Location = new System.Drawing.Point( 20, 88 );
			this.label21.Name = "label21";
			this.label21.Size = new System.Drawing.Size( 53, 13 );
			this.label21.TabIndex = 7;
			this.label21.Text = "Yaw Gain";
			this.tlToolTip.SetToolTip( this.label21, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			// 
			// lblPitchGain
			// 
			this.lblPitchGain.Location = new System.Drawing.Point( 281, 48 );
			this.lblPitchGain.Name = "lblPitchGain";
			this.lblPitchGain.Size = new System.Drawing.Size( 42, 13 );
			this.lblPitchGain.TabIndex = 8;
			this.lblPitchGain.Text = "128";
			this.lblPitchGain.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			this.tlToolTip.SetToolTip( this.lblPitchGain, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			// 
			// hsYawGain
			// 
			this.hsYawGain.Location = new System.Drawing.Point( 73, 88 );
			this.hsYawGain.Maximum = 265;
			this.hsYawGain.Minimum = 1;
			this.hsYawGain.Name = "hsYawGain";
			this.hsYawGain.Size = new System.Drawing.Size( 205, 16 );
			this.hsYawGain.TabIndex = 3;
			this.hsYawGain.TabStop = true;
			this.tlToolTip.SetToolTip( this.hsYawGain, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			this.hsYawGain.Value = 128;
			this.hsYawGain.ValueChanged += new System.EventHandler( this.hsYawGain_ValueChanged );
			// 
			// lblRollGain
			// 
			this.lblRollGain.Location = new System.Drawing.Point( 281, 68 );
			this.lblRollGain.Name = "lblRollGain";
			this.lblRollGain.Size = new System.Drawing.Size( 42, 13 );
			this.lblRollGain.TabIndex = 9;
			this.lblRollGain.Text = "128";
			this.lblRollGain.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			this.tlToolTip.SetToolTip( this.lblRollGain, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			// 
			// label19
			// 
			this.label19.AutoSize = true;
			this.label19.Location = new System.Drawing.Point( 20, 68 );
			this.label19.Name = "label19";
			this.label19.Size = new System.Drawing.Size( 50, 13 );
			this.label19.TabIndex = 5;
			this.label19.Text = "Roll Gain";
			this.tlToolTip.SetToolTip( this.label19, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			// 
			// lblYawGain
			// 
			this.lblYawGain.Location = new System.Drawing.Point( 281, 88 );
			this.lblYawGain.Name = "lblYawGain";
			this.lblYawGain.Size = new System.Drawing.Size( 42, 13 );
			this.lblYawGain.TabIndex = 10;
			this.lblYawGain.Text = "128";
			this.lblYawGain.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			this.tlToolTip.SetToolTip( this.lblYawGain, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			// 
			// label11
			// 
			this.label11.AutoSize = true;
			this.label11.Location = new System.Drawing.Point( 14, 48 );
			this.label11.Name = "label11";
			this.label11.Size = new System.Drawing.Size( 56, 13 );
			this.label11.TabIndex = 4;
			this.label11.Text = "Pitch Gain";
			this.tlToolTip.SetToolTip( this.label11, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			// 
			// hsAscentGain
			// 
			this.hsAscentGain.Location = new System.Drawing.Point( 73, 108 );
			this.hsAscentGain.Maximum = 265;
			this.hsAscentGain.Minimum = 1;
			this.hsAscentGain.Name = "hsAscentGain";
			this.hsAscentGain.Size = new System.Drawing.Size( 205, 16 );
			this.hsAscentGain.TabIndex = 4;
			this.hsAscentGain.TabStop = true;
			this.tlToolTip.SetToolTip( this.hsAscentGain, "The affort applied by the controller to attain the correct rate of ascent / desce" +
					"nt" );
			this.hsAscentGain.Value = 128;
			this.hsAscentGain.ValueChanged += new System.EventHandler( this.hsAscentGain_ValueChanged );
			// 
			// hsRollGain
			// 
			this.hsRollGain.Location = new System.Drawing.Point( 73, 68 );
			this.hsRollGain.Maximum = 265;
			this.hsRollGain.Minimum = 1;
			this.hsRollGain.Name = "hsRollGain";
			this.hsRollGain.Size = new System.Drawing.Size( 205, 16 );
			this.hsRollGain.TabIndex = 2;
			this.hsRollGain.TabStop = true;
			this.tlToolTip.SetToolTip( this.hsRollGain, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			this.hsRollGain.Value = 128;
			this.hsRollGain.ValueChanged += new System.EventHandler( this.hsRollGain_ValueChanged );
			// 
			// label25
			// 
			this.label25.AutoSize = true;
			this.label25.Location = new System.Drawing.Point( 5, 108 );
			this.label25.Name = "label25";
			this.label25.Size = new System.Drawing.Size( 65, 13 );
			this.label25.TabIndex = 12;
			this.label25.Text = "Ascent Gain";
			this.tlToolTip.SetToolTip( this.label25, "The affort applied by the controller to attain the correct rate of ascent / desce" +
					"nt" );
			// 
			// hsPitchGain
			// 
			this.hsPitchGain.Location = new System.Drawing.Point( 73, 48 );
			this.hsPitchGain.Maximum = 265;
			this.hsPitchGain.Minimum = 1;
			this.hsPitchGain.Name = "hsPitchGain";
			this.hsPitchGain.Size = new System.Drawing.Size( 205, 16 );
			this.hsPitchGain.TabIndex = 1;
			this.hsPitchGain.TabStop = true;
			this.tlToolTip.SetToolTip( this.hsPitchGain, "Gain values affect how forcefully the controller responds to outside influence.  " +
					"Higher numbers mean tighter control, but too high can cause instability." );
			this.hsPitchGain.Value = 128;
			this.hsPitchGain.ValueChanged += new System.EventHandler( this.hsPitchGain_ValueChanged );
			// 
			// lblAscentGain
			// 
			this.lblAscentGain.Location = new System.Drawing.Point( 281, 108 );
			this.lblAscentGain.Name = "lblAscentGain";
			this.lblAscentGain.Size = new System.Drawing.Size( 42, 13 );
			this.lblAscentGain.TabIndex = 13;
			this.lblAscentGain.Text = "128";
			this.lblAscentGain.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			this.tlToolTip.SetToolTip( this.lblAscentGain, "The affort applied by the controller to attain the correct rate of ascent / desce" +
					"nt" );
			// 
			// cbPitchRollLocked
			// 
			this.cbPitchRollLocked.AutoSize = true;
			this.cbPitchRollLocked.Location = new System.Drawing.Point( 14, 25 );
			this.cbPitchRollLocked.Name = "cbPitchRollLocked";
			this.cbPitchRollLocked.Size = new System.Drawing.Size( 115, 17 );
			this.cbPitchRollLocked.TabIndex = 0;
			this.cbPitchRollLocked.Text = "Pitch && Roll locked";
			this.tlToolTip.SetToolTip( this.cbPitchRollLocked, "Use the same values for pitch and roll - typically for quads that are X frames, n" +
					"ot H frames." );
			this.cbPitchRollLocked.UseVisualStyleBackColor = true;
			this.cbPitchRollLocked.CheckedChanged += new System.EventHandler( this.cbPitchRollLocked_CheckedChanged );
			// 
			// hsAltiGain
			// 
			this.hsAltiGain.Location = new System.Drawing.Point( 73, 128 );
			this.hsAltiGain.Maximum = 265;
			this.hsAltiGain.Minimum = 1;
			this.hsAltiGain.Name = "hsAltiGain";
			this.hsAltiGain.Size = new System.Drawing.Size( 205, 16 );
			this.hsAltiGain.TabIndex = 5;
			this.hsAltiGain.TabStop = true;
			this.tlToolTip.SetToolTip( this.hsAltiGain, "The affort applied by the controller to attain the correct altitude" );
			this.hsAltiGain.Value = 128;
			this.hsAltiGain.ValueChanged += new System.EventHandler( this.hsAltiGain_ValueChanged );
			// 
			// label27
			// 
			this.label27.AutoSize = true;
			this.label27.Location = new System.Drawing.Point( 3, 128 );
			this.label27.Name = "label27";
			this.label27.Size = new System.Drawing.Size( 67, 13 );
			this.label27.TabIndex = 15;
			this.label27.Text = "Altitude Gain";
			this.tlToolTip.SetToolTip( this.label27, "The affort applied by the controller to attain the correct altitude" );
			// 
			// lblAltiGain
			// 
			this.lblAltiGain.Location = new System.Drawing.Point( 281, 128 );
			this.lblAltiGain.Name = "lblAltiGain";
			this.lblAltiGain.Size = new System.Drawing.Size( 42, 13 );
			this.lblAltiGain.TabIndex = 16;
			this.lblAltiGain.Text = "128";
			this.lblAltiGain.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			this.tlToolTip.SetToolTip( this.lblAltiGain, "The affort applied by the controller to attain the correct altitude" );
			// 
			// groupBox4
			// 
			this.groupBox4.Enabled = false;
			this.groupBox4.Location = new System.Drawing.Point( 8, 173 );
			this.groupBox4.Name = "groupBox4";
			this.groupBox4.Size = new System.Drawing.Size( 705, 180 );
			this.groupBox4.TabIndex = 15;
			this.groupBox4.TabStop = false;
			this.groupBox4.Text = "Advanced Settings";
			this.groupBox4.Visible = false;
			// 
			// cbEnableAdvanced
			// 
			this.cbEnableAdvanced.AutoSize = true;
			this.cbEnableAdvanced.Enabled = false;
			this.cbEnableAdvanced.Location = new System.Drawing.Point( 561, 150 );
			this.cbEnableAdvanced.Name = "cbEnableAdvanced";
			this.cbEnableAdvanced.Size = new System.Drawing.Size( 152, 17 );
			this.cbEnableAdvanced.TabIndex = 1;
			this.cbEnableAdvanced.Text = "Enable Advanced Settings";
			this.cbEnableAdvanced.UseVisualStyleBackColor = true;
			// 
			// btnUploadFlightChanges
			// 
			this.btnUploadFlightChanges.Location = new System.Drawing.Point( 355, 144 );
			this.btnUploadFlightChanges.Name = "btnUploadFlightChanges";
			this.btnUploadFlightChanges.Size = new System.Drawing.Size( 130, 23 );
			this.btnUploadFlightChanges.TabIndex = 0;
			this.btnUploadFlightChanges.Text = "Upload Changes";
			this.tlToolTip.SetToolTip( this.btnUploadFlightChanges, "Upload changes on this page to the flight controller" );
			this.btnUploadFlightChanges.UseVisualStyleBackColor = true;
			this.btnUploadFlightChanges.Click += new System.EventHandler( this.btnUploadFlightChanges_Click );
			// 
			// tpRadioSetup
			// 
			this.tpRadioSetup.BackColor = System.Drawing.SystemColors.Control;
			this.tpRadioSetup.Controls.Add( this.tbCalibrateDocs );
			this.tpRadioSetup.Controls.Add( this.lblYawSpeedManual );
			this.tpRadioSetup.Controls.Add( this.label14 );
			this.tpRadioSetup.Controls.Add( this.tbYawSpeedManual );
			this.tpRadioSetup.Controls.Add( this.lblAccelCorrectionFilter );
			this.tpRadioSetup.Controls.Add( this.label13 );
			this.tpRadioSetup.Controls.Add( this.tbAccelCorrectionFilter );
			this.tpRadioSetup.Controls.Add( this.lblThrustCorrection );
			this.tpRadioSetup.Controls.Add( this.label17 );
			this.tpRadioSetup.Controls.Add( this.tbThrustCorrection );
			this.tpRadioSetup.Controls.Add( this.btnControlReset );
			this.tpRadioSetup.Controls.Add( this.lblYawSpeedAuto );
			this.tpRadioSetup.Controls.Add( this.label5 );
			this.tpRadioSetup.Controls.Add( this.tbYawSpeedAuto );
			this.tpRadioSetup.Controls.Add( this.btnUploadRollPitch );
			this.tpRadioSetup.Controls.Add( this.lblRollPitchManual );
			this.tpRadioSetup.Controls.Add( this.lblRollPitchAngle );
			this.tpRadioSetup.Controls.Add( this.label3 );
			this.tpRadioSetup.Controls.Add( this.tbRollPitchManual );
			this.tpRadioSetup.Controls.Add( this.label2 );
			this.tpRadioSetup.Controls.Add( this.tbRollPitchAuto );
			this.tpRadioSetup.Controls.Add( this.btnCalibrate );
			this.tpRadioSetup.Controls.Add( this.label1 );
			this.tpRadioSetup.Controls.Add( this.cbReceiverType );
			this.tpRadioSetup.Controls.Add( this.cbRev8 );
			this.tpRadioSetup.Controls.Add( this.cbRev7 );
			this.tpRadioSetup.Controls.Add( this.cbRev2 );
			this.tpRadioSetup.Controls.Add( this.cbRev3 );
			this.tpRadioSetup.Controls.Add( this.cbRev6 );
			this.tpRadioSetup.Controls.Add( this.cbRev5 );
			this.tpRadioSetup.Controls.Add( this.cbRev4 );
			this.tpRadioSetup.Controls.Add( this.cbRev1 );
			this.tpRadioSetup.Controls.Add( this.cbChannel8 );
			this.tpRadioSetup.Controls.Add( this.cbChannel7 );
			this.tpRadioSetup.Controls.Add( this.cbChannel2 );
			this.tpRadioSetup.Controls.Add( this.cbChannel3 );
			this.tpRadioSetup.Controls.Add( this.cbChannel6 );
			this.tpRadioSetup.Controls.Add( this.cbChannel5 );
			this.tpRadioSetup.Controls.Add( this.cbChannel4 );
			this.tpRadioSetup.Controls.Add( this.cbChannel1 );
			this.tpRadioSetup.Controls.Add( this.vbR_Channel8 );
			this.tpRadioSetup.Controls.Add( this.vbR_Channel7 );
			this.tpRadioSetup.Controls.Add( this.vbR_Channel6 );
			this.tpRadioSetup.Controls.Add( this.vbR_Channel5 );
			this.tpRadioSetup.Controls.Add( this.vbR_RS_XValue );
			this.tpRadioSetup.Controls.Add( this.vbR_RS_YValue );
			this.tpRadioSetup.Controls.Add( this.vbR_LS_XValue );
			this.tpRadioSetup.Controls.Add( this.vbR_LS_YValue );
			this.tpRadioSetup.Controls.Add( this.rsR_Right );
			this.tpRadioSetup.Controls.Add( this.rsR_Left );
			this.tpRadioSetup.Location = new System.Drawing.Point( 4, 22 );
			this.tpRadioSetup.Name = "tpRadioSetup";
			this.tpRadioSetup.Size = new System.Drawing.Size( 721, 356 );
			this.tpRadioSetup.TabIndex = 3;
			this.tpRadioSetup.Text = "Radio Setup";
			// 
			// rsR_Left
			// 
			this.rsR_Left.Location = new System.Drawing.Point( 204, 57 );
			this.rsR_Left.Name = "rsR_Left";
			this.rsR_Left.Size = new System.Drawing.Size( 150, 150 );
			this.rsR_Left.TabIndex = 33;
			this.rsR_Left.Text = "radioStick1";
			// 
			// rsR_Right
			// 
			this.rsR_Right.Location = new System.Drawing.Point( 367, 57 );
			this.rsR_Right.Name = "rsR_Right";
			this.rsR_Right.Size = new System.Drawing.Size( 150, 150 );
			this.rsR_Right.TabIndex = 34;
			this.rsR_Right.Text = "radioStick2";
			// 
			// vbR_LS_YValue
			// 
			this.vbR_LS_YValue.BarColor = System.Drawing.Color.LightGreen;
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
			// vbR_LS_XValue
			// 
			this.vbR_LS_XValue.BarColor = System.Drawing.Color.LightGreen;
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
			// vbR_RS_YValue
			// 
			this.vbR_RS_YValue.BarColor = System.Drawing.Color.LightGreen;
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
			// vbR_RS_XValue
			// 
			this.vbR_RS_XValue.BarColor = System.Drawing.Color.LightGreen;
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
			// vbR_Channel5
			// 
			this.vbR_Channel5.BarColor = System.Drawing.Color.LightGreen;
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
			// vbR_Channel6
			// 
			this.vbR_Channel6.BarColor = System.Drawing.Color.LightGreen;
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
			// vbR_Channel7
			// 
			this.vbR_Channel7.BarColor = System.Drawing.Color.LightGreen;
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
			// vbR_Channel8
			// 
			this.vbR_Channel8.BarColor = System.Drawing.Color.LightGreen;
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
			// cbChannel1
			// 
			this.cbChannel1.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbChannel1.FormattingEnabled = true;
			this.cbChannel1.Location = new System.Drawing.Point( 59, 57 );
			this.cbChannel1.Name = "cbChannel1";
			this.cbChannel1.Size = new System.Drawing.Size( 46, 21 );
			this.cbChannel1.TabIndex = 4;
			this.cbChannel1.Tag = "1";
			this.tlToolTip.SetToolTip( this.cbChannel1, "Selects the radio channel to use for this flight function" );
			this.cbChannel1.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
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
			this.tlToolTip.SetToolTip( this.cbChannel4, "Selects the radio channel to use for this flight function" );
			this.cbChannel4.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
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
			this.tlToolTip.SetToolTip( this.cbChannel5, "Selects the radio channel to use for this flight function" );
			this.cbChannel5.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
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
			this.tlToolTip.SetToolTip( this.cbChannel6, "Selects the radio channel to use for this flight function" );
			this.cbChannel6.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
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
			this.tlToolTip.SetToolTip( this.cbChannel3, "Selects the radio channel to use for this flight function" );
			this.cbChannel3.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
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
			this.tlToolTip.SetToolTip( this.cbChannel2, "Selects the radio channel to use for this flight function" );
			this.cbChannel2.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
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
			this.tlToolTip.SetToolTip( this.cbChannel7, "Selects the radio channel to use for this flight function" );
			this.cbChannel7.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
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
			this.tlToolTip.SetToolTip( this.cbChannel8, "Selects the radio channel to use for this flight function" );
			this.cbChannel8.SelectedIndexChanged += new System.EventHandler( this.cbChannel_SelectedIndexChanged );
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
			this.tlToolTip.SetToolTip( this.cbRev1, "Reverse this radio channel" );
			this.cbRev1.UseVisualStyleBackColor = true;
			this.cbRev1.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
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
			this.tlToolTip.SetToolTip( this.cbRev4, "Reverse this radio channel" );
			this.cbRev4.UseVisualStyleBackColor = true;
			this.cbRev4.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
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
			this.tlToolTip.SetToolTip( this.cbRev5, "Reverse this radio channel" );
			this.cbRev5.UseVisualStyleBackColor = true;
			this.cbRev5.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
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
			this.tlToolTip.SetToolTip( this.cbRev6, "Reverse this radio channel" );
			this.cbRev6.UseVisualStyleBackColor = true;
			this.cbRev6.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
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
			this.tlToolTip.SetToolTip( this.cbRev3, "Reverse this radio channel" );
			this.cbRev3.UseVisualStyleBackColor = true;
			this.cbRev3.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
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
			this.tlToolTip.SetToolTip( this.cbRev2, "Reverse this radio channel" );
			this.cbRev2.UseVisualStyleBackColor = true;
			this.cbRev2.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
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
			this.tlToolTip.SetToolTip( this.cbRev7, "Reverse this radio channel" );
			this.cbRev7.UseVisualStyleBackColor = true;
			this.cbRev7.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
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
			this.tlToolTip.SetToolTip( this.cbRev8, "Reverse this radio channel" );
			this.cbRev8.UseVisualStyleBackColor = true;
			this.cbRev8.CheckedChanged += new System.EventHandler( this.cbRev_CheckedChanged );
			// 
			// cbReceiverType
			// 
			this.cbReceiverType.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
			this.cbReceiverType.FormattingEnabled = true;
			this.cbReceiverType.Items.AddRange( new object[] {
            "PWM",
            "SBUS",
            "PPM"} );
			this.cbReceiverType.Location = new System.Drawing.Point( 328, 21 );
			this.cbReceiverType.Name = "cbReceiverType";
			this.cbReceiverType.Size = new System.Drawing.Size( 74, 21 );
			this.cbReceiverType.TabIndex = 18;
			this.tlToolTip.SetToolTip( this.cbReceiverType, "Choose the communication type for your receiver (SBUS and PPM = 1 wire, PWM = mul" +
					"tiple wires)" );
			this.cbReceiverType.SelectedIndexChanged += new System.EventHandler( this.cbReceiverType_SelectedIndexChanged );
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
			// tbRollPitchAuto
			// 
			this.tbRollPitchAuto.Location = new System.Drawing.Point( 101, 228 );
			this.tbRollPitchAuto.Maximum = 85;
			this.tbRollPitchAuto.Minimum = 10;
			this.tbRollPitchAuto.Name = "tbRollPitchAuto";
			this.tbRollPitchAuto.Size = new System.Drawing.Size( 110, 45 );
			this.tbRollPitchAuto.TabIndex = 19;
			this.tbRollPitchAuto.TickFrequency = 5;
			this.tlToolTip.SetToolTip( this.tbRollPitchAuto, "Maximum angle the craft will tilt when given a full-stick command" );
			this.tbRollPitchAuto.Value = 30;
			this.tbRollPitchAuto.ValueChanged += new System.EventHandler( this.tbRollPitchAngle_ValueChanged );
			// 
			// label2
			// 
			this.label2.Location = new System.Drawing.Point( 9, 228 );
			this.label2.Name = "label2";
			this.label2.Size = new System.Drawing.Size( 87, 45 );
			this.label2.TabIndex = 66;
			this.label2.Text = "Max Roll / Pitch (Auto Level)";
			this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// tbRollPitchManual
			// 
			this.tbRollPitchManual.LargeChange = 3;
			this.tbRollPitchManual.Location = new System.Drawing.Point( 354, 228 );
			this.tbRollPitchManual.Maximum = 72;
			this.tbRollPitchManual.Minimum = 5;
			this.tbRollPitchManual.Name = "tbRollPitchManual";
			this.tbRollPitchManual.Size = new System.Drawing.Size( 99, 45 );
			this.tbRollPitchManual.TabIndex = 21;
			this.tbRollPitchManual.TickFrequency = 10;
			this.tlToolTip.SetToolTip( this.tbRollPitchManual, "Speed the craft will tilt when given a full-stick command in manual mode (larger " +
					"numbers are faster)" );
			this.tbRollPitchManual.Value = 18;
			this.tbRollPitchManual.ValueChanged += new System.EventHandler( this.tbRollPitchManual_ValueChanged );
			// 
			// label3
			// 
			this.label3.Location = new System.Drawing.Point( 264, 228 );
			this.label3.Name = "label3";
			this.label3.Size = new System.Drawing.Size( 87, 45 );
			this.label3.TabIndex = 68;
			this.label3.Text = "Roll / Pitch Speed (Manual)";
			this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// lblRollPitchAngle
			// 
			this.lblRollPitchAngle.AutoSize = true;
			this.lblRollPitchAngle.Location = new System.Drawing.Point( 214, 244 );
			this.lblRollPitchAngle.Name = "lblRollPitchAngle";
			this.lblRollPitchAngle.Size = new System.Drawing.Size( 40, 13 );
			this.lblRollPitchAngle.TabIndex = 69;
			this.lblRollPitchAngle.Text = "30 deg";
			// 
			// lblRollPitchManual
			// 
			this.lblRollPitchManual.Location = new System.Drawing.Point( 459, 244 );
			this.lblRollPitchManual.Name = "lblRollPitchManual";
			this.lblRollPitchManual.Size = new System.Drawing.Size( 58, 13 );
			this.lblRollPitchManual.TabIndex = 70;
			this.lblRollPitchManual.Text = "180 deg/s";
			// 
			// btnUploadRollPitch
			// 
			this.btnUploadRollPitch.Location = new System.Drawing.Point( 300, 330 );
			this.btnUploadRollPitch.Name = "btnUploadRollPitch";
			this.btnUploadRollPitch.Size = new System.Drawing.Size( 129, 22 );
			this.btnUploadRollPitch.TabIndex = 22;
			this.btnUploadRollPitch.Text = "Upload Changes";
			this.tlToolTip.SetToolTip( this.btnUploadRollPitch, "Upload changes on this page to the flight controller" );
			this.btnUploadRollPitch.UseVisualStyleBackColor = true;
			this.btnUploadRollPitch.Click += new System.EventHandler( this.btnUploadRollPitch_Click );
			// 
			// tbYawSpeedAuto
			// 
			this.tbYawSpeedAuto.LargeChange = 3;
			this.tbYawSpeedAuto.Location = new System.Drawing.Point( 101, 279 );
			this.tbYawSpeedAuto.Maximum = 72;
			this.tbYawSpeedAuto.Minimum = 5;
			this.tbYawSpeedAuto.Name = "tbYawSpeedAuto";
			this.tbYawSpeedAuto.Size = new System.Drawing.Size( 110, 45 );
			this.tbYawSpeedAuto.TabIndex = 3;
			this.tbYawSpeedAuto.TickFrequency = 3;
			this.tlToolTip.SetToolTip( this.tbYawSpeedAuto, "How quickly the craft will change heading.  Larger numbers are faster." );
			this.tbYawSpeedAuto.Value = 18;
			this.tbYawSpeedAuto.ValueChanged += new System.EventHandler( this.tbYawSpeedAuto_ValueChanged );
			// 
			// label5
			// 
			this.label5.Location = new System.Drawing.Point( 23, 279 );
			this.label5.Name = "label5";
			this.label5.Size = new System.Drawing.Size( 73, 45 );
			this.label5.TabIndex = 73;
			this.label5.Text = "Yaw Speed (Auto Level)";
			this.label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// lblYawSpeedAuto
			// 
			this.lblYawSpeedAuto.AutoSize = true;
			this.lblYawSpeedAuto.Location = new System.Drawing.Point( 214, 295 );
			this.lblYawSpeedAuto.Name = "lblYawSpeedAuto";
			this.lblYawSpeedAuto.Size = new System.Drawing.Size( 56, 13 );
			this.lblYawSpeedAuto.TabIndex = 74;
			this.lblYawSpeedAuto.Text = "180 deg/s";
			// 
			// btnControlReset
			// 
			this.btnControlReset.Location = new System.Drawing.Point( 136, 19 );
			this.btnControlReset.Name = "btnControlReset";
			this.btnControlReset.Size = new System.Drawing.Size( 75, 23 );
			this.btnControlReset.TabIndex = 0;
			this.btnControlReset.Text = "Reset";
			this.tlToolTip.SetToolTip( this.btnControlReset, "Reset all channel scale / reverse assignments" );
			this.btnControlReset.UseVisualStyleBackColor = true;
			this.btnControlReset.Click += new System.EventHandler( this.btnControlReset_Click );
			// 
			// tbThrustCorrection
			// 
			this.tbThrustCorrection.LargeChange = 32;
			this.tbThrustCorrection.Location = new System.Drawing.Point( 589, 279 );
			this.tbThrustCorrection.Maximum = 512;
			this.tbThrustCorrection.Name = "tbThrustCorrection";
			this.tbThrustCorrection.Size = new System.Drawing.Size( 83, 45 );
			this.tbThrustCorrection.TabIndex = 77;
			this.tbThrustCorrection.TickFrequency = 32;
			this.tlToolTip.SetToolTip( this.tbThrustCorrection, "Compensates for loss of thrust when the craft is tilted.  (0 is disabled)" );
			this.tbThrustCorrection.Value = 256;
			this.tbThrustCorrection.ValueChanged += new System.EventHandler( this.tbThrustCorrection_ValueChanged );
			// 
			// label17
			// 
			this.label17.Location = new System.Drawing.Point( 520, 279 );
			this.label17.Name = "label17";
			this.label17.Size = new System.Drawing.Size( 66, 45 );
			this.label17.TabIndex = 78;
			this.label17.Text = "Thrust Angle Correction";
			this.label17.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// lblThrustCorrection
			// 
			this.lblThrustCorrection.AutoSize = true;
			this.lblThrustCorrection.Location = new System.Drawing.Point( 678, 295 );
			this.lblThrustCorrection.Name = "lblThrustCorrection";
			this.lblThrustCorrection.Size = new System.Drawing.Size( 34, 13 );
			this.lblThrustCorrection.TabIndex = 79;
			this.lblThrustCorrection.Text = "1.000";
			// 
			// tbAccelCorrectionFilter
			// 
			this.tbAccelCorrectionFilter.LargeChange = 16;
			this.tbAccelCorrectionFilter.Location = new System.Drawing.Point( 589, 228 );
			this.tbAccelCorrectionFilter.Maximum = 256;
			this.tbAccelCorrectionFilter.Name = "tbAccelCorrectionFilter";
			this.tbAccelCorrectionFilter.Size = new System.Drawing.Size( 83, 45 );
			this.tbAccelCorrectionFilter.TabIndex = 80;
			this.tbAccelCorrectionFilter.TickFrequency = 16;
			this.tlToolTip.SetToolTip( this.tbAccelCorrectionFilter, "Smaller numbers mean vertical disturbance correction is softer.  (0 is disabled)" );
			this.tbAccelCorrectionFilter.Value = 16;
			this.tbAccelCorrectionFilter.ValueChanged += new System.EventHandler( this.tbAccelCorrectionFilter_ValueChanged );
			// 
			// label13
			// 
			this.label13.Location = new System.Drawing.Point( 520, 228 );
			this.label13.Name = "label13";
			this.label13.Size = new System.Drawing.Size( 66, 45 );
			this.label13.TabIndex = 81;
			this.label13.Text = "Vertical Disturbance Correction";
			this.label13.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// lblAccelCorrectionFilter
			// 
			this.lblAccelCorrectionFilter.AutoSize = true;
			this.lblAccelCorrectionFilter.Location = new System.Drawing.Point( 678, 244 );
			this.lblAccelCorrectionFilter.Name = "lblAccelCorrectionFilter";
			this.lblAccelCorrectionFilter.Size = new System.Drawing.Size( 34, 13 );
			this.lblAccelCorrectionFilter.TabIndex = 82;
			this.lblAccelCorrectionFilter.Text = "0.062";
			// 
			// tbYawSpeedManual
			// 
			this.tbYawSpeedManual.LargeChange = 3;
			this.tbYawSpeedManual.Location = new System.Drawing.Point( 354, 279 );
			this.tbYawSpeedManual.Maximum = 72;
			this.tbYawSpeedManual.Minimum = 5;
			this.tbYawSpeedManual.Name = "tbYawSpeedManual";
			this.tbYawSpeedManual.Size = new System.Drawing.Size( 99, 45 );
			this.tbYawSpeedManual.TabIndex = 83;
			this.tbYawSpeedManual.TickFrequency = 10;
			this.tlToolTip.SetToolTip( this.tbYawSpeedManual, "How quickly the craft will change heading.  Larger numbers are faster." );
			this.tbYawSpeedManual.Value = 18;
			this.tbYawSpeedManual.ValueChanged += new System.EventHandler( this.tbYawSpeedManual_ValueChanged );
			// 
			// label14
			// 
			this.label14.Location = new System.Drawing.Point( 278, 279 );
			this.label14.Name = "label14";
			this.label14.Size = new System.Drawing.Size( 73, 45 );
			this.label14.TabIndex = 84;
			this.label14.Text = "Yaw Speed (Manual)";
			this.label14.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// lblYawSpeedManual
			// 
			this.lblYawSpeedManual.Location = new System.Drawing.Point( 459, 295 );
			this.lblYawSpeedManual.Name = "lblYawSpeedManual";
			this.lblYawSpeedManual.Size = new System.Drawing.Size( 58, 13 );
			this.lblYawSpeedManual.TabIndex = 85;
			this.lblYawSpeedManual.Text = "180 deg/s";
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
			// 
			// btnMotor2
			// 
			this.btnMotor2.BackgroundImage = global::Elev8.Properties.Resources.Icon_CounterClockwise;
			this.btnMotor2.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
			this.btnMotor2.Location = new System.Drawing.Point( 345, 68 );
			this.btnMotor2.Name = "btnMotor2";
			this.btnMotor2.Size = new System.Drawing.Size( 75, 67 );
			this.btnMotor2.TabIndex = 20;
			this.btnMotor2.Text = "2";
			this.btnMotor2.TextAlign = System.Drawing.ContentAlignment.TopRight;
			this.btnMotor2.UseVisualStyleBackColor = true;
			this.btnMotor2.MouseDown += new System.Windows.Forms.MouseEventHandler( this.btnMotor2_MouseDown );
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
			// 
			// btnMotor4
			// 
			this.btnMotor4.BackgroundImage = global::Elev8.Properties.Resources.Icon_CounterClockwise;
			this.btnMotor4.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
			this.btnMotor4.Location = new System.Drawing.Point( 119, 226 );
			this.btnMotor4.Name = "btnMotor4";
			this.btnMotor4.Size = new System.Drawing.Size( 75, 67 );
			this.btnMotor4.TabIndex = 22;
			this.btnMotor4.Text = "4";
			this.btnMotor4.TextAlign = System.Drawing.ContentAlignment.BottomLeft;
			this.btnMotor4.UseVisualStyleBackColor = true;
			this.btnMotor4.MouseDown += new System.Windows.Forms.MouseEventHandler( this.btnMotor4_MouseDown );
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
			// lblCalibrateDocs
			// 
			this.lblCalibrateDocs.Location = new System.Drawing.Point( 111, 296 );
			this.lblCalibrateDocs.Name = "lblCalibrateDocs";
			this.lblCalibrateDocs.Size = new System.Drawing.Size( 316, 49 );
			this.lblCalibrateDocs.TabIndex = 27;
			this.lblCalibrateDocs.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// tpSensors
			// 
			this.tpSensors.BackColor = System.Drawing.Color.Transparent;
			this.tpSensors.Controls.Add( this.cbAltiBaro );
			this.tpSensors.Controls.Add( this.cbVoltage );
			this.tpSensors.Controls.Add( this.cbYaw );
			this.tpSensors.Controls.Add( this.cbRoll );
			this.tpSensors.Controls.Add( this.cbPitch );
			this.tpSensors.Controls.Add( this.cbAltiEst );
			this.tpSensors.Controls.Add( this.cbMagZ );
			this.tpSensors.Controls.Add( this.cbMagY );
			this.tpSensors.Controls.Add( this.cbMagX );
			this.tpSensors.Controls.Add( this.cbAccelZ );
			this.tpSensors.Controls.Add( this.cbAccelY );
			this.tpSensors.Controls.Add( this.cbAccelX );
			this.tpSensors.Controls.Add( this.cbGyroZ );
			this.tpSensors.Controls.Add( this.cbGyroY );
			this.tpSensors.Controls.Add( this.cbGyroX );
			this.tpSensors.Controls.Add( this.plotSensors );
			this.tpSensors.Location = new System.Drawing.Point( 4, 22 );
			this.tpSensors.Name = "tpSensors";
			this.tpSensors.Padding = new System.Windows.Forms.Padding( 3 );
			this.tpSensors.Size = new System.Drawing.Size( 721, 356 );
			this.tpSensors.TabIndex = 1;
			this.tpSensors.Text = "Sensors";
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
			// cbAltiEst
			// 
			this.cbAltiEst.AutoSize = true;
			this.cbAltiEst.Location = new System.Drawing.Point( 8, 217 );
			this.cbAltiEst.Name = "cbAltiEst";
			this.cbAltiEst.Size = new System.Drawing.Size( 64, 17 );
			this.cbAltiEst.TabIndex = 9;
			this.cbAltiEst.Tag = "11";
			this.cbAltiEst.Text = "Alti (Est)";
			this.cbAltiEst.UseVisualStyleBackColor = true;
			this.cbAltiEst.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbPitch
			// 
			this.cbPitch.AutoSize = true;
			this.cbPitch.Location = new System.Drawing.Point( 8, 237 );
			this.cbPitch.Name = "cbPitch";
			this.cbPitch.Size = new System.Drawing.Size( 50, 17 );
			this.cbPitch.TabIndex = 10;
			this.cbPitch.Tag = "12";
			this.cbPitch.Text = "Pitch";
			this.cbPitch.UseVisualStyleBackColor = true;
			this.cbPitch.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbRoll
			// 
			this.cbRoll.AutoSize = true;
			this.cbRoll.Location = new System.Drawing.Point( 8, 257 );
			this.cbRoll.Name = "cbRoll";
			this.cbRoll.Size = new System.Drawing.Size( 44, 17 );
			this.cbRoll.TabIndex = 11;
			this.cbRoll.Tag = "13";
			this.cbRoll.Text = "Roll";
			this.cbRoll.UseVisualStyleBackColor = true;
			this.cbRoll.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbYaw
			// 
			this.cbYaw.AutoSize = true;
			this.cbYaw.Location = new System.Drawing.Point( 8, 277 );
			this.cbYaw.Name = "cbYaw";
			this.cbYaw.Size = new System.Drawing.Size( 47, 17 );
			this.cbYaw.TabIndex = 12;
			this.cbYaw.Tag = "14";
			this.cbYaw.Text = "Yaw";
			this.cbYaw.UseVisualStyleBackColor = true;
			this.cbYaw.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbVoltage
			// 
			this.cbVoltage.AutoSize = true;
			this.cbVoltage.Location = new System.Drawing.Point( 8, 297 );
			this.cbVoltage.Name = "cbVoltage";
			this.cbVoltage.Size = new System.Drawing.Size( 62, 17 );
			this.cbVoltage.TabIndex = 13;
			this.cbVoltage.Tag = "15";
			this.cbVoltage.Text = "Voltage";
			this.cbVoltage.UseVisualStyleBackColor = true;
			this.cbVoltage.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// cbAltiBaro
			// 
			this.cbAltiBaro.AutoSize = true;
			this.cbAltiBaro.Location = new System.Drawing.Point( 8, 197 );
			this.cbAltiBaro.Name = "cbAltiBaro";
			this.cbAltiBaro.Size = new System.Drawing.Size( 71, 17 );
			this.cbAltiBaro.TabIndex = 15;
			this.cbAltiBaro.Tag = "10";
			this.cbAltiBaro.Text = "Alti (Baro)";
			this.cbAltiBaro.UseVisualStyleBackColor = true;
			this.cbAltiBaro.CheckedChanged += new System.EventHandler( this.cbGraphLegend_CheckedChanged );
			// 
			// tpStatus
			// 
			this.tpStatus.BackColor = System.Drawing.SystemColors.Control;
			this.tpStatus.Controls.Add( this.label23 );
			this.tpStatus.Controls.Add( this.lblCycles );
			this.tpStatus.Controls.Add( this.vbRollOut );
			this.tpStatus.Controls.Add( this.vbYawOut );
			this.tpStatus.Controls.Add( this.vbPitchOut );
			this.tpStatus.Controls.Add( this.vbBackLeft );
			this.tpStatus.Controls.Add( this.vbFrontLeft );
			this.tpStatus.Controls.Add( this.vbBackRight );
			this.tpStatus.Controls.Add( this.vbFrontRight );
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
			// aicAltimeter
			// 
			this.aicAltimeter.Location = new System.Drawing.Point( 15, 6 );
			this.aicAltimeter.Name = "aicAltimeter";
			this.aicAltimeter.Size = new System.Drawing.Size( 150, 150 );
			this.aicAltimeter.TabIndex = 5;
			this.aicAltimeter.Text = "altimeter1";
			// 
			// aicHeading
			// 
			this.aicHeading.Location = new System.Drawing.Point( 351, 6 );
			this.aicHeading.Name = "aicHeading";
			this.aicHeading.Size = new System.Drawing.Size( 150, 150 );
			this.aicHeading.TabIndex = 6;
			this.aicHeading.Text = "headingIndicator1";
			// 
			// ocOrientation
			// 
			this.ocOrientation.CubeDepth = 1.2F;
			this.ocOrientation.CubeHeight = 0.8F;
			this.ocOrientation.CubeWidth = 1.2F;
			this.ocOrientation.Location = new System.Drawing.Point( 507, 6 );
			this.ocOrientation.Name = "ocOrientation";
			this.ocOrientation.Size = new System.Drawing.Size( 206, 124 );
			this.ocOrientation.TabIndex = 3;
			// 
			// aicAttitude
			// 
			this.aicAttitude.Location = new System.Drawing.Point( 183, 6 );
			this.aicAttitude.Name = "aicAttitude";
			this.aicAttitude.Size = new System.Drawing.Size( 150, 150 );
			this.aicAttitude.TabIndex = 4;
			this.aicAttitude.Text = "attitudeIndicatorInstrumentControl1";
			// 
			// rsLeft
			// 
			this.rsLeft.Location = new System.Drawing.Point( 102, 182 );
			this.rsLeft.Name = "rsLeft";
			this.rsLeft.Size = new System.Drawing.Size( 150, 150 );
			this.rsLeft.TabIndex = 7;
			this.rsLeft.Text = "radioStick1";
			// 
			// rsRight
			// 
			this.rsRight.Location = new System.Drawing.Point( 265, 182 );
			this.rsRight.Name = "rsRight";
			this.rsRight.Size = new System.Drawing.Size( 150, 150 );
			this.rsRight.TabIndex = 8;
			this.rsRight.Text = "radioStick2";
			// 
			// vbLS_YValue
			// 
			this.vbLS_YValue.BarColor = System.Drawing.Color.LightGreen;
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
			// vbLS_XValue
			// 
			this.vbLS_XValue.BarColor = System.Drawing.Color.LightGreen;
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
			// vbRS_YValue
			// 
			this.vbRS_YValue.BarColor = System.Drawing.Color.LightGreen;
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
			// vbRS_XValue
			// 
			this.vbRS_XValue.BarColor = System.Drawing.Color.LightGreen;
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
			// vbChannel5
			// 
			this.vbChannel5.BarColor = System.Drawing.Color.LightGreen;
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
			// vbChannel6
			// 
			this.vbChannel6.BarColor = System.Drawing.Color.LightGreen;
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
			// vbChannel7
			// 
			this.vbChannel7.BarColor = System.Drawing.Color.LightGreen;
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
			// vbChannel8
			// 
			this.vbChannel8.BarColor = System.Drawing.Color.LightGreen;
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
			// vbVoltage
			// 
			this.vbVoltage.BarColor = System.Drawing.Color.LightGreen;
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
			// vbFrontRight
			// 
			this.vbFrontRight.BarColor = System.Drawing.Color.Tomato;
			this.vbFrontRight.Font = new System.Drawing.Font( "Microsoft Sans Serif", 8.25F );
			this.vbFrontRight.FromLeft = true;
			this.vbFrontRight.LeftLabel = "";
			this.vbFrontRight.Location = new System.Drawing.Point( 425, 182 );
			this.vbFrontRight.MaxValue = 16000;
			this.vbFrontRight.MinValue = 8000;
			this.vbFrontRight.Name = "vbFrontRight";
			this.vbFrontRight.RightLabel = "FR";
			this.vbFrontRight.Size = new System.Drawing.Size( 83, 18 );
			this.vbFrontRight.TabIndex = 34;
			this.vbFrontRight.Value = 0;
			// 
			// vbBackRight
			// 
			this.vbBackRight.BarColor = System.Drawing.Color.Tomato;
			this.vbBackRight.Font = new System.Drawing.Font( "Microsoft Sans Serif", 8.25F );
			this.vbBackRight.FromLeft = true;
			this.vbBackRight.LeftLabel = "";
			this.vbBackRight.Location = new System.Drawing.Point( 425, 316 );
			this.vbBackRight.MaxValue = 16000;
			this.vbBackRight.MinValue = 8000;
			this.vbBackRight.Name = "vbBackRight";
			this.vbBackRight.RightLabel = "BR";
			this.vbBackRight.Size = new System.Drawing.Size( 83, 18 );
			this.vbBackRight.TabIndex = 35;
			this.vbBackRight.Value = 0;
			// 
			// vbFrontLeft
			// 
			this.vbFrontLeft.BarColor = System.Drawing.Color.Tomato;
			this.vbFrontLeft.Font = new System.Drawing.Font( "Microsoft Sans Serif", 8.25F );
			this.vbFrontLeft.FromLeft = false;
			this.vbFrontLeft.LeftLabel = "FL";
			this.vbFrontLeft.Location = new System.Drawing.Point( 9, 182 );
			this.vbFrontLeft.MaxValue = 16000;
			this.vbFrontLeft.MinValue = 8000;
			this.vbFrontLeft.Name = "vbFrontLeft";
			this.vbFrontLeft.RightLabel = "";
			this.vbFrontLeft.Size = new System.Drawing.Size( 83, 18 );
			this.vbFrontLeft.TabIndex = 36;
			this.vbFrontLeft.Value = 0;
			// 
			// vbBackLeft
			// 
			this.vbBackLeft.BarColor = System.Drawing.Color.Tomato;
			this.vbBackLeft.Font = new System.Drawing.Font( "Microsoft Sans Serif", 8.25F );
			this.vbBackLeft.FromLeft = false;
			this.vbBackLeft.LeftLabel = "BL";
			this.vbBackLeft.Location = new System.Drawing.Point( 9, 316 );
			this.vbBackLeft.MaxValue = 16000;
			this.vbBackLeft.MinValue = 8000;
			this.vbBackLeft.Name = "vbBackLeft";
			this.vbBackLeft.RightLabel = "";
			this.vbBackLeft.Size = new System.Drawing.Size( 83, 18 );
			this.vbBackLeft.TabIndex = 37;
			this.vbBackLeft.Value = 0;
			// 
			// vbPitchOut
			// 
			this.vbPitchOut.BarColor = System.Drawing.Color.Aquamarine;
			this.vbPitchOut.FromLeft = true;
			this.vbPitchOut.LeftLabel = "Pitch Power";
			this.vbPitchOut.Location = new System.Drawing.Point( 532, 206 );
			this.vbPitchOut.MaxValue = 5000;
			this.vbPitchOut.MinValue = -5000;
			this.vbPitchOut.Name = "vbPitchOut";
			this.vbPitchOut.RightLabel = "0";
			this.vbPitchOut.Size = new System.Drawing.Size( 171, 16 );
			this.vbPitchOut.TabIndex = 38;
			this.vbPitchOut.Value = 0;
			// 
			// vbYawOut
			// 
			this.vbYawOut.BarColor = System.Drawing.Color.Aquamarine;
			this.vbYawOut.FromLeft = true;
			this.vbYawOut.LeftLabel = "Yaw Power";
			this.vbYawOut.Location = new System.Drawing.Point( 532, 290 );
			this.vbYawOut.MaxValue = 5000;
			this.vbYawOut.MinValue = -5000;
			this.vbYawOut.Name = "vbYawOut";
			this.vbYawOut.RightLabel = "0";
			this.vbYawOut.Size = new System.Drawing.Size( 171, 16 );
			this.vbYawOut.TabIndex = 39;
			this.vbYawOut.Value = 0;
			// 
			// vbRollOut
			// 
			this.vbRollOut.BarColor = System.Drawing.Color.Aquamarine;
			this.vbRollOut.FromLeft = true;
			this.vbRollOut.LeftLabel = "Roll Power";
			this.vbRollOut.Location = new System.Drawing.Point( 532, 248 );
			this.vbRollOut.MaxValue = 5000;
			this.vbRollOut.MinValue = -5000;
			this.vbRollOut.Name = "vbRollOut";
			this.vbRollOut.RightLabel = "0";
			this.vbRollOut.Size = new System.Drawing.Size( 171, 16 );
			this.vbRollOut.TabIndex = 40;
			this.vbRollOut.Value = 0;
			// 
			// lblCycles
			// 
			this.lblCycles.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
			this.lblCycles.Location = new System.Drawing.Point( 425, 336 );
			this.lblCycles.Name = "lblCycles";
			this.lblCycles.Size = new System.Drawing.Size( 293, 17 );
			this.lblCycles.TabIndex = 41;
			this.lblCycles.Text = "0 cycles";
			this.lblCycles.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
			// 
			// label23
			// 
			this.label23.Image = global::Elev8.Properties.Resources.ParallaxLogo;
			this.label23.Location = new System.Drawing.Point( 517, 133 );
			this.label23.Name = "label23";
			this.label23.Size = new System.Drawing.Size( 198, 58 );
			this.label23.TabIndex = 42;
			// 
			// tcTabs
			// 
			this.tcTabs.Controls.Add( this.tpStatus );
			this.tcTabs.Controls.Add( this.tpSensors );
			this.tcTabs.Controls.Add( this.tpSysTest );
			this.tcTabs.Controls.Add( this.tpRadioSetup );
			this.tcTabs.Controls.Add( this.tpControllerSetup );
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
			// MainForm
			// 
			this.AutoScaleDimensions = new System.Drawing.SizeF( 6F, 13F );
			this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
			this.ClientSize = new System.Drawing.Size( 729, 428 );
			this.Controls.Add( this.tcTabs );
			this.Controls.Add( this.stMainStatus );
			this.Controls.Add( this.msMainMenu );
			this.Icon = ((System.Drawing.Icon)(resources.GetObject( "$this.Icon" )));
			this.MainMenuStrip = this.msMainMenu;
			this.MinimumSize = new System.Drawing.Size( 745, 464 );
			this.Name = "MainForm";
			this.Text = "Elev8 Ground Station";
			this.FormClosing += new System.Windows.Forms.FormClosingEventHandler( this.MainForm_FormClosing );
			this.stMainStatus.ResumeLayout( false );
			this.stMainStatus.PerformLayout();
			this.msMainMenu.ResumeLayout( false );
			this.msMainMenu.PerformLayout();
			this.tpAccelCalibration.ResumeLayout( false );
			this.groupBox5.ResumeLayout( false );
			this.groupBox5.PerformLayout();
			this.groupBox6.ResumeLayout( false );
			this.groupBox6.PerformLayout();
			((System.ComponentModel.ISupportInitialize)(this.udPitchCorrection)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udRollCorrection)).EndInit();
			this.tpGyroCalibration.ResumeLayout( false );
			this.groupBox12.ResumeLayout( false );
			this.groupBox12.PerformLayout();
			this.groupBox11.ResumeLayout( false );
			this.groupBox11.PerformLayout();
			this.groupBox10.ResumeLayout( false );
			this.groupBox10.PerformLayout();
			this.tpSystemSetup.ResumeLayout( false );
			this.groupBox1.ResumeLayout( false );
			this.groupBox1.PerformLayout();
			((System.ComponentModel.ISupportInitialize)(this.udArmedLowThrottle)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udHighThrottle)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udLowThrottle)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udTestThrottle)).EndInit();
			this.groupBox2.ResumeLayout( false );
			this.groupBox2.PerformLayout();
			((System.ComponentModel.ISupportInitialize)(this.udVoltageOffset)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.udLowVoltageAlarmThreshold)).EndInit();
			this.tpControllerSetup.ResumeLayout( false );
			this.tpControllerSetup.PerformLayout();
			this.groupBox3.ResumeLayout( false );
			this.groupBox3.PerformLayout();
			this.tpRadioSetup.ResumeLayout( false );
			this.tpRadioSetup.PerformLayout();
			((System.ComponentModel.ISupportInitialize)(this.tbRollPitchAuto)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbRollPitchManual)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbYawSpeedAuto)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbThrustCorrection)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbAccelCorrectionFilter)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.tbYawSpeedManual)).EndInit();
			this.tpSysTest.ResumeLayout( false );
			this.tpSysTest.PerformLayout();
			this.tpSensors.ResumeLayout( false );
			this.tpSensors.PerformLayout();
			this.tpStatus.ResumeLayout( false );
			this.tcTabs.ResumeLayout( false );
			this.ResumeLayout( false );
			this.PerformLayout();

		}

		#endregion

		private System.Windows.Forms.StatusStrip stMainStatus;
		private System.Windows.Forms.ToolStripStatusLabel tssStatusText;
		private System.Windows.Forms.Timer tmCommTimer;
		private System.Windows.Forms.MenuStrip msMainMenu;
		private System.Windows.Forms.ToolStripMenuItem settingsToolStripMenuItem;
		private System.Windows.Forms.ToolStripMenuItem radioDisplayToolStripMenuItem;
		private System.Windows.Forms.ToolStripSeparator toolStripMenuItem2;
		private System.Windows.Forms.ToolStripMenuItem miRadioMode1;
		private System.Windows.Forms.ToolStripMenuItem miRadioMode2;
		private System.Windows.Forms.ToolStripSeparator toolStripMenuItem1;
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
		private System.Windows.Forms.ToolTip tlToolTip;
		private System.Windows.Forms.ToolStripStatusLabel tssFCVersion;
		private System.Windows.Forms.ToolStripStatusLabel tssGSVersion;
		private System.Windows.Forms.ToolStripMenuItem helpToolStripMenuItem;
		private System.Windows.Forms.ToolStripMenuItem aboutToolStripMenuItem;
		private System.Windows.Forms.TabPage tpAccelCalibration;
		private OrientationCube ocAccelOrient;
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
		private System.Windows.Forms.TabPage tpSystemSetup;
		private System.Windows.Forms.GroupBox groupBox2;
		private Elev8.Controls.ValueBar vbVoltage2;
		private System.Windows.Forms.CheckBox cbUseBatteryMonitor;
		private System.Windows.Forms.CheckBox cbLowVoltageAlarm;
		private System.Windows.Forms.NumericUpDown udLowVoltageAlarmThreshold;
		private System.Windows.Forms.Label label9;
		private System.Windows.Forms.Label label12;
		private System.Windows.Forms.NumericUpDown udVoltageOffset;
		private System.Windows.Forms.Label label10;
		private System.Windows.Forms.Button btnFactoryDefaultPrefs;
		private System.Windows.Forms.Button btnUploadThrottle;
		private System.Windows.Forms.GroupBox groupBox1;
		private System.Windows.Forms.CheckBox cbDisableMotors;
		private System.Windows.Forms.Label label6;
		private System.Windows.Forms.Label label16;
		private System.Windows.Forms.Label label4;
		private System.Windows.Forms.ComboBox cbArmingDelay;
		private System.Windows.Forms.Label label15;
		private System.Windows.Forms.NumericUpDown udTestThrottle;
		private System.Windows.Forms.ComboBox cbDisarmDelay;
		private System.Windows.Forms.Label label7;
		private System.Windows.Forms.Label label8;
		private System.Windows.Forms.NumericUpDown udLowThrottle;
		private System.Windows.Forms.NumericUpDown udHighThrottle;
		private System.Windows.Forms.NumericUpDown udArmedLowThrottle;
		private System.Windows.Forms.TabPage tpControllerSetup;
		private System.Windows.Forms.Button btnUploadFlightChanges;
		private System.Windows.Forms.CheckBox cbEnableAdvanced;
		private System.Windows.Forms.GroupBox groupBox4;
		private System.Windows.Forms.GroupBox groupBox3;
		private System.Windows.Forms.Label lblAltiGain;
		private System.Windows.Forms.Label label27;
		private System.Windows.Forms.HScrollBar hsAltiGain;
		private System.Windows.Forms.CheckBox cbPitchRollLocked;
		private System.Windows.Forms.Label lblAscentGain;
		private System.Windows.Forms.HScrollBar hsPitchGain;
		private System.Windows.Forms.Label label25;
		private System.Windows.Forms.HScrollBar hsRollGain;
		private System.Windows.Forms.HScrollBar hsAscentGain;
		private System.Windows.Forms.Label label11;
		private System.Windows.Forms.Label lblYawGain;
		private System.Windows.Forms.Label label19;
		private System.Windows.Forms.Label lblRollGain;
		private System.Windows.Forms.HScrollBar hsYawGain;
		private System.Windows.Forms.Label lblPitchGain;
		private System.Windows.Forms.Label label21;
		private System.Windows.Forms.TabPage tpRadioSetup;
		private System.Windows.Forms.TextBox tbCalibrateDocs;
		private System.Windows.Forms.Label lblYawSpeedManual;
		private System.Windows.Forms.Label label14;
		private System.Windows.Forms.TrackBar tbYawSpeedManual;
		private System.Windows.Forms.Label lblAccelCorrectionFilter;
		private System.Windows.Forms.Label label13;
		private System.Windows.Forms.TrackBar tbAccelCorrectionFilter;
		private System.Windows.Forms.Label lblThrustCorrection;
		private System.Windows.Forms.Label label17;
		private System.Windows.Forms.TrackBar tbThrustCorrection;
		private System.Windows.Forms.Button btnControlReset;
		private System.Windows.Forms.Label lblYawSpeedAuto;
		private System.Windows.Forms.Label label5;
		private System.Windows.Forms.TrackBar tbYawSpeedAuto;
		private System.Windows.Forms.Button btnUploadRollPitch;
		private System.Windows.Forms.Label lblRollPitchManual;
		private System.Windows.Forms.Label lblRollPitchAngle;
		private System.Windows.Forms.Label label3;
		private System.Windows.Forms.TrackBar tbRollPitchManual;
		private System.Windows.Forms.Label label2;
		private System.Windows.Forms.TrackBar tbRollPitchAuto;
		private System.Windows.Forms.Button btnCalibrate;
		private System.Windows.Forms.Label label1;
		private System.Windows.Forms.ComboBox cbReceiverType;
		private System.Windows.Forms.CheckBox cbRev8;
		private System.Windows.Forms.CheckBox cbRev7;
		private System.Windows.Forms.CheckBox cbRev2;
		private System.Windows.Forms.CheckBox cbRev3;
		private System.Windows.Forms.CheckBox cbRev6;
		private System.Windows.Forms.CheckBox cbRev5;
		private System.Windows.Forms.CheckBox cbRev4;
		private System.Windows.Forms.CheckBox cbRev1;
		private System.Windows.Forms.ComboBox cbChannel8;
		private System.Windows.Forms.ComboBox cbChannel7;
		private System.Windows.Forms.ComboBox cbChannel2;
		private System.Windows.Forms.ComboBox cbChannel3;
		private System.Windows.Forms.ComboBox cbChannel6;
		private System.Windows.Forms.ComboBox cbChannel5;
		private System.Windows.Forms.ComboBox cbChannel4;
		private System.Windows.Forms.ComboBox cbChannel1;
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
		private System.Windows.Forms.TabPage tpSensors;
		private System.Windows.Forms.CheckBox cbAltiBaro;
		private System.Windows.Forms.CheckBox cbVoltage;
		private System.Windows.Forms.CheckBox cbYaw;
		private System.Windows.Forms.CheckBox cbRoll;
		private System.Windows.Forms.CheckBox cbPitch;
		private System.Windows.Forms.CheckBox cbAltiEst;
		private System.Windows.Forms.CheckBox cbMagZ;
		private System.Windows.Forms.CheckBox cbMagY;
		private System.Windows.Forms.CheckBox cbMagX;
		private System.Windows.Forms.CheckBox cbAccelZ;
		private System.Windows.Forms.CheckBox cbAccelY;
		private System.Windows.Forms.CheckBox cbAccelX;
		private System.Windows.Forms.CheckBox cbGyroZ;
		private System.Windows.Forms.CheckBox cbGyroY;
		private System.Windows.Forms.CheckBox cbGyroX;
		private GraphLib.PlotterDisplayEx plotSensors;
		private System.Windows.Forms.TabPage tpStatus;
		private System.Windows.Forms.Label label23;
		private System.Windows.Forms.Label lblCycles;
		private Elev8.Controls.ValueBar vbRollOut;
		private Elev8.Controls.ValueBar vbYawOut;
		private Elev8.Controls.ValueBar vbPitchOut;
		private Elev8.Controls.ValueBar vbBackLeft;
		private Elev8.Controls.ValueBar vbFrontLeft;
		private Elev8.Controls.ValueBar vbBackRight;
		private Elev8.Controls.ValueBar vbFrontRight;
		private Elev8.Controls.ValueBar vbVoltage;
		private Elev8.Controls.ValueBar vbChannel8;
		private Elev8.Controls.ValueBar vbChannel7;
		private Elev8.Controls.ValueBar vbChannel6;
		private Elev8.Controls.ValueBar vbChannel5;
		private Elev8.Controls.ValueBar vbRS_XValue;
		private Elev8.Controls.ValueBar vbRS_YValue;
		private Elev8.Controls.ValueBar vbLS_XValue;
		private Elev8.Controls.ValueBar vbLS_YValue;
		private RadioStick rsRight;
		private RadioStick rsLeft;
		private AttitudeIndicator aicAttitude;
		private OrientationCube ocOrientation;
		private HeadingIndicator aicHeading;
		private Altimeter aicAltimeter;
		private System.Windows.Forms.TabControl tcTabs;
	}
}

