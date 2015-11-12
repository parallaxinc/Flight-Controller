using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Threading;
using System.Text;
using System.Windows.Forms;



namespace Elev8
{
	public partial class MainForm : Form
	{
		Connection_FTDI comm = new Connection_FTDI();
		CommStatus status = CommStatus.Initializing;

		enum RADIO_MODE
		{
			Mode1,		// (Rudder, Elevator)  (Aileron,Throttle)
			Mode2		// (Rudder, Throttle)  (Aileron,Elevator)
		};

		RADIO_MODE RadioMode = RADIO_MODE.Mode2;	// North American default


		public class RadioData
		{
			public short Thro, Aile, Elev, Rudd;
			public short Gear, Aux1, Aux2, Aux3;						// Radio values = 16 bytes
			public short BatteryVolts;                                  // Battery Monitor = 2 bytes
		}


		public struct SensorData
		{
			public short Temp, GyroX, GyroY, GyroZ;
			public short AccelX, AccelY, AccelZ;						// IMU sensors = 20 bytes
			public short MagX, MagY, MagZ;
		};



		public struct ComputedData
		{
			public int Pitch, Roll, Yaw;								// IMU = 12 bytes
			public int Alt, AltTemp, AltiEst;							// Altimeter = 12 bytes

		};

		RadioData radio = new RadioData();
		SensorData sensors = new SensorData();
		Quaternion  q = new Quaternion();
		ComputedData computed = new ComputedData();

		ComboBox[] channelAssignControls = new ComboBox[8];


		public MainForm()
		{
			InitializeComponent();

			channelAssignControls = new ComboBox[] {
				cbChannel1, cbChannel2, cbChannel3, cbChannel4,
				cbChannel5, cbChannel6, cbChannel7, cbChannel8 };

			for(int i = 0; i < 8; i++)
			{
				for(int j = 0; j < 8; j++) {
					if(i == j) {
						channelAssignControls[i].Items.Add( "Channel " + (j + 1).ToString() + '*');
					}
					else {
						channelAssignControls[i].Items.Add( "Channel " + (j + 1).ToString() );
					}
				}

				channelAssignControls[i].SelectedIndex = i;
			}

			SetRadioMode( RadioMode );
			comm.Start();
		}



		private void MainForm_FormClosing( object sender, FormClosingEventArgs e )
		{
			comm.Stop();
		}


		private void tmCommTimer_Tick( object sender, EventArgs e )
		{
			UpdateStatus();
			ProcessPackets();
		}


		void ProcessPackets()
		{
			bool bRadioChanged = false;
			bool bSensorsChanged = false;
			bool bQuatChanged = false;
			bool bComputedChanged = false;

			Packet p;
			do {
				p = comm.GetPacket();
				if(p != null)
				{
					switch( p.mode )
					{
						case 1:	// Radio data
							radio.Thro = p.GetShort();
							radio.Aile = p.GetShort();
							radio.Elev = p.GetShort();
							radio.Rudd = p.GetShort();
							radio.Gear = p.GetShort();
							radio.Aux1 = p.GetShort();
							radio.Aux2 = p.GetShort();
							radio.Aux3 = p.GetShort();
							radio.BatteryVolts = p.GetShort();
							bRadioChanged = true;
							break;

						case 2:	// Sensor values
							sensors.Temp = p.GetShort();
							sensors.GyroX = p.GetShort();
							sensors.GyroY = p.GetShort();
							sensors.GyroZ = p.GetShort();
							sensors.AccelX = p.GetShort();
							sensors.AccelY = p.GetShort();
							sensors.AccelZ = p.GetShort();
							sensors.MagX = p.GetShort();
							sensors.MagY = p.GetShort();
							sensors.MagZ = p.GetShort();
							bSensorsChanged = true;
							break;

						case 3:	// Quaternion
							q.x = p.GetFloat();
							q.y = p.GetFloat();
							q.z = p.GetFloat();
							q.w = p.GetFloat();
							bQuatChanged = true;
							break;

						case 4:	// Compute values
							computed.Pitch = p.GetInt();
							computed.Roll = p.GetInt();
							computed.Yaw = p.GetInt();

							computed.Alt = p.GetInt();
							computed.AltTemp = p.GetInt();
							computed.AltiEst = p.GetInt();
							bComputedChanged = true;
							break;
					}
				}
			} while(p != null);


			if(bRadioChanged)
			{
				if(tcTabs.SelectedTab == tpStatus)
				{
					if(RadioMode == RADIO_MODE.Mode2)	// North American
					{
						rsLeft.SetParameters( radio.Rudd, radio.Thro );
						rsRight.SetParameters( radio.Aile, radio.Elev );

						vbLS_YValue.RightLabel = radio.Thro.ToString();
						vbLS_YValue.Value = radio.Thro;

						vbRS_YValue.RightLabel = radio.Elev.ToString();
						vbRS_YValue.Value = radio.Elev;

					}
					else if(RadioMode == RADIO_MODE.Mode1)	// European
					{
						rsLeft.SetParameters( radio.Rudd, radio.Elev );
						rsRight.SetParameters( radio.Aile, radio.Thro );

						vbLS_YValue.RightLabel = radio.Elev.ToString();
						vbLS_YValue.Value = radio.Elev;

						vbRS_YValue.RightLabel = radio.Thro.ToString();
						vbRS_YValue.Value = radio.Thro;
					}

					vbLS_XValue.RightLabel = radio.Rudd.ToString();
					vbLS_XValue.Value = radio.Rudd;

					vbRS_XValue.RightLabel = radio.Aile.ToString();
					vbRS_XValue.Value = radio.Aile;


					vbChannel5.RightLabel = radio.Gear.ToString();
					vbChannel5.Value = radio.Gear;

					vbChannel6.RightLabel = radio.Aux1.ToString();
					vbChannel6.Value = radio.Aux1;

					vbChannel7.RightLabel = radio.Aux2.ToString();
					vbChannel7.Value = radio.Aux2;

					vbChannel8.RightLabel = radio.Aux3.ToString();
					vbChannel8.Value = radio.Aux3;
				}
				else if(tcTabs.SelectedTab == tpControlSetup)
				{
					if(RadioMode == RADIO_MODE.Mode2)	// North American
					{
						rsR_Left.SetParameters( radio.Rudd, radio.Thro );
						rsR_Right.SetParameters( radio.Aile, radio.Elev );

						vbR_LS_YValue.RightLabel = radio.Thro.ToString();
						vbR_LS_YValue.Value = radio.Thro;

						vbR_RS_YValue.RightLabel = radio.Elev.ToString();
						vbR_RS_YValue.Value = radio.Elev;

					}
					else if(RadioMode == RADIO_MODE.Mode1)	// European
					{
						rsR_Left.SetParameters( radio.Rudd, radio.Elev );
						rsR_Right.SetParameters( radio.Aile, radio.Thro );

						vbR_LS_YValue.RightLabel = radio.Elev.ToString();
						vbR_LS_YValue.Value = radio.Elev;

						vbR_RS_YValue.RightLabel = radio.Thro.ToString();
						vbR_RS_YValue.Value = radio.Thro;
					}

					vbR_LS_XValue.RightLabel = radio.Rudd.ToString();
					vbR_LS_XValue.Value = radio.Rudd;

					vbR_RS_XValue.RightLabel = radio.Aile.ToString();
					vbR_RS_XValue.Value = radio.Aile;


					vbR_Channel5.RightLabel = radio.Gear.ToString();
					vbR_Channel5.Value = radio.Gear;

					vbR_Channel6.RightLabel = radio.Aux1.ToString();
					vbR_Channel6.Value = radio.Aux1;

					vbR_Channel7.RightLabel = radio.Aux2.ToString();
					vbR_Channel7.Value = radio.Aux2;

					vbR_Channel8.RightLabel = radio.Aux3.ToString();
					vbR_Channel8.Value = radio.Aux3;
				}
			}

			if(bQuatChanged) {
				ocOrientation.Quat = q;
			}

			if(bComputedChanged)
			{
				aicAttitude.SetAttitudeIndicatorParameters(
					(double)computed.Pitch / (3768.0 / 10.0),
					(double)computed.Roll / (-32768.0 / 90.0) );

				aicAltimeter.SetAlimeterParameters( (float)computed.AltiEst / 1000.0f );
				aicHeading.SetHeadingIndicatorParameters( (computed.Yaw & ((1 << 17) - 1)) / (65536 / 180) );
			}
		}


		void UpdateStatus()
		{
			CommStatus newStat = comm.Status;
			if(newStat != status)
			{
				status = newStat;
				string msg;

				switch(status)
				{
					case CommStatus.Initializing:
						msg = "Initializing";
						break;

					case CommStatus.NoDevice:
						msg = "No FTDI devices found";
						break;

					case CommStatus.NoElev8:
						msg = "No Elev8-FC connected";
						break;

					case CommStatus.Connected:
						msg = "Connected";
						break;

					default:
						msg = "";
						break;
				}

				tssStatusText.Text = msg;
			}
		}


		private void miRadioMode1_Click( object sender, EventArgs e )
		{
			SetRadioMode( RADIO_MODE.Mode1 );
		}

		private void miRadioMode2_Click( object sender, EventArgs e )
		{
			SetRadioMode( RADIO_MODE.Mode2 );
		}


		void SetRadioMode( RADIO_MODE mode )
		{
			RadioMode = mode;
			miRadioMode1.Checked = RadioMode == RADIO_MODE.Mode1;
			miRadioMode2.Checked = RadioMode == RADIO_MODE.Mode2;

			if(RadioMode == RADIO_MODE.Mode1)
			{
				vbLS_YValue.LeftLabel = "Elevator";
				vbRS_YValue.LeftLabel = "Throttle";
			}
			else if(RadioMode == RADIO_MODE.Mode2)
			{
				vbLS_YValue.LeftLabel = "Throttle";
				vbRS_YValue.LeftLabel = "Elevator";
			}
		}

		private void cbChannel1_SelectedIndexChanged(object sender, EventArgs e)
		{
		}
	}
}
