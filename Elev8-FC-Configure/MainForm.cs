using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace Elev8
{
    public partial class MainForm : Form
    {
		SerialPort comm = null;
		bool Active = false;

		int Thro = 0;
		int Aile = 0;
		int Elev = 0;
		int Rudd = 0;
		int Aux1 = 0;
		int Aux2 = 0;
		int Aux3 = 0;
		int Aux4 = 0;

		int GyroTemp;
		int GyroX, GyroY, GyroZ;
		int AccelX, AccelY, AccelZ;
		int MagX, MagY, MagZ;
		int Alt, AltTemp;
		int Pitch, Roll, Yaw;


		byte[] txBuffer = new byte[32];
		byte[] rxBuffer = new byte[32];

		const int QSize = 1024;
		byte[] rxQueue = new byte[QSize];
		int QHead = 0;
		int QTail = 0;

		byte[] OutputMode = { 0, 1, 2, 3, 2, 4, 5, 2};			// None=0, Radio=1, Sensors=2, Motor=3, Sensors=2, IMU=4, IMUComp=5
		byte[] PacketSizes = { 0, 16+3, 28+3, 0, 22+3, 12+3 };	// None=0, Radio=11, Sensors=31, Motor=0, IMU=25, IMUComp=12  (bytes)
		int SampleCounter = 0;


		enum Mode
		{
			None,
			RadioTest,
			SensorTest,
			MotorTest,
			GyroCalibration,
			IMUTest,
			IMUCompare,
			VibrationTest,
		};

		Mode currentMode = Mode.None;
		int CalibrationCycle = 0;



		public MainForm()
        {
            InitializeComponent();

			gGyroTemp.displayOffset = 25.0f;
			gGyroTemp.displayScale = 1.0f / 16.0f;	// 16 deg C per bit
			gGyroTemp.displayPostfix = " *C";

			gHeading.displayScale = 0.1f;	// We're going to feed it +/- 1800 units, or 10 units per degree
			gHeading.GaugeCircle = 1.0f;	// want this to use the full 360 degrees of the gauge, unlike a normal analog gauge

			Connect();
        }


		void Connect()
		{
			string[] ports = SerialPort.GetPortNames();
			string FoundPort = null;

			foreach(string port in ports)
			{
				if(port.StartsWith( "COM" ))
				{
					int index = int.Parse( port.Substring( 3 ) );
					if(index > 4) {
						FoundPort = port;
						try {
							// Open the serial port at 115200 baud, no parity, 8 bits, one stop bit
							comm = new SerialPort( FoundPort, 115200, Parity.None, 8, StopBits.One );
							comm.Open();

							txBuffer[0] = (byte)0xff;	// Send 0xff to the Prop to see if it replies

							for(int i = 0; i < 20 && comm.BytesToRead == 0; i++)	// Keep pinging until it replies, or we give up
							{
								comm.Write( txBuffer, 0, 1 );
								System.Threading.Thread.Sleep( 50 );
							}

							if( comm.BytesToRead > 0 && comm.ReadByte() == 0x55 )	// If it comes back with 0x55 it's the one we want
								break;
						}

						catch(Exception)
						{
							// Keep trying - this one didn't work
							comm = null;
						}
					}
				}
			}

			Active = true;
			if( comm != null ) {
				//currentMode = (Mode)(tcMainTabs.SelectedIndex + 1);
				//txBuffer[0] = (byte)currentMode;
				//comm.Write( txBuffer, 0, 1 );	// Which mode are we in?
			}

			// Start my 'tick timer' - It's set to tick every 20 milliseconds
			// (used to check the comm port periodically instead of using a thread)
			tickTimer.Start();
		}

		private void SetActive( bool NewActive )
		{
			if(NewActive == true && comm == null) {
				Connect();
			}

			if(Active == NewActive) return;
			if(NewActive == false) {

				currentMode = Mode.None;
				if(comm != null && comm.IsOpen)
				{
					txBuffer[0] = OutputMode[ (byte)currentMode ];
					comm.Write( txBuffer, 0, 1 );	// Which mode are we in?

					comm.Close();
				}
			}
			else {
				if(comm != null) {
					comm.Open();
				}
				else {
					Connect();
				}
			}

			Active = NewActive;
		}

		private void MainForm_Activated( object sender, EventArgs e )
		{
			SetActive( true );
		}

		private void MainForm_Deactivate( object sender, EventArgs e )
		{
			SetActive( false );
		}


		int GetCommWord()
		{
			int val = (int)BitConverter.ToInt16( rxQueue, QTail );
			QTail += 2;

			return val;
		}

		int GetCommTriple()
		{
			int val = (rxQueue[QTail++] << 16);
			val |= (rxQueue[QTail++] << 8);
			val |= rxQueue[QTail++];
			return val;
		}


		float GetCommFloat()
		{
			float Result = BitConverter.ToSingle( rxQueue, QTail );
			QTail += 4;
			return Result;
		}

		int GetCommLong()
		{
			int Result = BitConverter.ToInt32( rxQueue, QTail );
			QTail += 4;
			return Result;
		}


		private void tickTimer_Tick( object sender, EventArgs e )
		{
			if(Active == false || comm == null || comm.IsOpen == false ) return;

			try
			{
				int QAvail = QSize - QHead;								// How much room is available at the end of the data queue?
				int bytesAvail = comm.BytesToRead;						// How much data is available from the serial port?
				if(bytesAvail > QAvail) bytesAvail = QAvail;			// Pick the smaller of the two values for how much to read

				if( bytesAvail > 0 ) {
					QHead += comm.Read( rxQueue, QHead, bytesAvail );	// Read from the serial port into the data queue buffer
				}

				do {
					if( QTail < (QHead-3) ) {							// Keep going as long as we can figure out what the next packet type should be
						if( rxQueue[QTail] != 0x77 )					// Check for our two signature bytes (0x77, 0x77)
							QTail++;
						else if( rxQueue[QTail+1] != 0x77 )				// If we don't see them, just consume data until we do
							QTail++;
						else
						{
							byte packetType = rxQueue[QTail + 2];		// Packet type is the 4th byte
							if( packetType > (byte)Mode.IMUCompare )	// If it's out of range, this is a bad packet, so skip what we've read and move on
								QTail += 3;
							else {
								byte packetSize = PacketSizes[ packetType ];	// Figure out how big the packet size is
								if((QHead - QTail) >= packetSize)		// Have we got enough data to process the packet?
								{
									ProcessPacket( packetType );		// Yep - do it
								}
								else
									break;	// Get out of the loop because we're out of data and need more
							}
						}
					}
				} while( QTail < (QHead-3) );	// Keep going until we're out of data


				// Move any data we have remaining in the queue back to the beginning.  This means we don't
				// have to deal with wrapping around at the end

				int BytesToMove = QHead - QTail;
				for( int i=0; i<BytesToMove; i++ ) {
					rxQueue[i] = rxQueue[QTail + i];
				}

				// Reset the QTail back to zero (start of the buffer) and adjust the QHead to how many bytes we have
				QTail = 0;
				QHead = BytesToMove;


				// Check to see if the user switched to a new tab, and update the flight controller if they did
				// This is done here because it's safe - it means we don't have to worry about locking the serial
				// port object or anything weird because we only use it in one place

				Mode tempMode = (Mode)(tcMainTabs.SelectedIndex+1);
				if(tempMode != currentMode) {

					if(currentMode == Mode.GyroCalibration) {
						txBuffer[0] = 0x11;
						comm.Write( txBuffer, 0, 1 );	// Reset to previous drift values
					}

					currentMode = tempMode;
					txBuffer[0] = OutputMode[ (byte)currentMode ];
					comm.Write( txBuffer, 0, 1 );	// Which mode are we in?

					if(currentMode == Mode.GyroCalibration) {
						txBuffer[0] = 0x10;
						comm.Write( txBuffer, 0, 1 );	// Zero drift calibration values
					}
				}
			}

			catch( Exception )
			{
				comm = null;
			}
		}


		private void ProcessPacket( byte packetType )
		{
			QTail += 3;	// Skip the header signature and the packet type values

			// Figure out what mode we're in and read the complete packet
			switch( packetType )
			{
				case 0x01:	// Receiver test packet - currently 4 words of data
					Thro = GetCommWord();
					Aile = GetCommWord();
					Elev = GetCommWord();
					Rudd = GetCommWord();
					Aux1 = GetCommWord();
					Aux2 = GetCommWord();
					Aux3 = GetCommWord();
					Aux4 = GetCommWord();

					if(currentMode == Mode.RadioTest)
					{
						rsLeft.XValue = (float)Rudd;
						rsLeft.YValue = (float)Thro;

						rsRight.XValue = (float)Aile;
						rsRight.YValue = (float)Elev;

						lblAux1.Text = Aux1.ToString();
						lblAux2.Text = Aux2.ToString();
						lblAux3.Text = Aux3.ToString();
						lblAux4.Text = Aux4.ToString();
					}
					//GotPacket = true;
					break;


				case 0x02:	// Sensor test packet
					GyroTemp = GetCommWord();
					GyroX = GetCommWord();
					GyroY = GetCommWord();
					GyroZ = GetCommWord();

					AccelX = GetCommWord();
					AccelY = GetCommWord();
					AccelZ = GetCommWord();

					MagX = GetCommWord();
					MagY = GetCommWord();
					MagZ = GetCommWord();

					Alt = GetCommLong();
					AltTemp = GetCommLong();

					// If we're on the sensor test tab, update those UI controls
					if(currentMode == Mode.SensorTest)
					{
						gGyroX.Value = (float)GyroX;
						gGyroY.Value = (float)GyroY;
						gGyroZ.Value = (float)GyroZ;

						gAccelX.Value = (float)AccelX;
						gAccelY.Value = (float)AccelY;
						gAccelZ.Value = (float)AccelZ;
						gGyroTemp.Value = (float)GyroTemp;

						gMagX.Value = (float)MagX;
						gMagY.Value = (float)MagY;
						gMagZ.Value = (float)MagZ;

						// Compute a heading from the magnetometer readings (not tilt compensated)
						//gHeading.Value = (float)(Math.Atan2( MagX, MagY ) * 1800.0/Math.PI);
						gHeading.Value = ComputeTiltCompensatedHeading();

						double pressure = (double)Alt / 4096.0;

						float altitudeFeet = (1 - (float)Math.Pow( pressure / 1013.25, 0.190284 )) * 145366.45f;
						float altTempDegrees = 42.5f + (float)AltTemp / 480.0f;

						lblAltimeter.Text = string.Format( "{0:0.00}hPa ({1:0.0} ft)", pressure, altitudeFeet );
						lblAltimeterTemp.Text = string.Format( "{0:0.0}*C", altTempDegrees );
					}


					// If we're on the sensor calibration tab, update the graph / line fit controls
					if(currentMode == Mode.GyroCalibration)
					{
						LineFit.Sample lfSample = new LineFit.Sample();
						lfSample.t = GyroTemp;
						lfSample.x = GyroX;
						lfSample.y = GyroY;
						lfSample.z = GyroZ;
						SampleCounter++;

						bool DoRedraw = (tcMainTabs.SelectedIndex == 3) & ((SampleCounter & 7) == 7);

						lfGraph.AddSample( lfSample, DoRedraw );

						int scaleX = (int)(1.0 / lfGraph.dSlope.x + 0.5);
						int scaleY = (int)(1.0 / lfGraph.dSlope.y + 0.5);
						int scaleZ = (int)(1.0 / lfGraph.dSlope.z + 0.5);
						int offsetX = (int)(lfGraph.dIntercept.x + 0.5);
						int offsetY = (int)(lfGraph.dIntercept.y + 0.5);
						int offsetZ = (int)(lfGraph.dIntercept.z + 0.5);

						if(Math.Abs( scaleX ) < 1024.0f)
							gxScale.Text = scaleX.ToString();
						else
							gxScale.Text = "0";

						if(Math.Abs( scaleY ) < 1024.0f)
							gyScale.Text = scaleY.ToString();
						else
							gyScale.Text = "0";

						if(Math.Abs( scaleZ ) < 1024.0f)
							gzScale.Text = scaleZ.ToString();
						else
							gzScale.Text = "0";

						gxOffset.Text = offsetX.ToString();
						gyOffset.Text = offsetY.ToString();
						gzOffset.Text = offsetZ.ToString();
					}


					if(currentMode == Mode.VibrationTest)
					{
						int[] gySample = new int[3];
						gySample[0] = GyroX;
						gySample[1] = GyroY;
						gySample[2] = GyroZ;
						grGyro.AddSample( gySample, true );

						int[] accSample = new int[3];
						accSample[0] = AccelX;
						accSample[1] = AccelY;
						accSample[2] = AccelZ;
						grAccel.AddSample( accSample, true );

						SampleCounter++;

						if((SampleCounter & 31) == 31)
						{
							grGyro.UpdateStats();
							lblGXMin.Text = grGyro.Mins[0].ToString();
							lblGXMax.Text = grGyro.Maxs[0].ToString();
							lblGXAvg.Text = grGyro.Avgs[0].ToString( "00.0000" );
							lblGXVar.Text = grGyro.Vars[0].ToString( "0.00000" );

							lblGYMin.Text = grGyro.Mins[1].ToString();
							lblGYMax.Text = grGyro.Maxs[1].ToString();
							lblGYAvg.Text = grGyro.Avgs[1].ToString( "00.0000" );
							lblGYVar.Text = grGyro.Vars[1].ToString( "0.00000" );

							lblGZMin.Text = grGyro.Mins[2].ToString();
							lblGZMax.Text = grGyro.Maxs[2].ToString();
							lblGZAvg.Text = grGyro.Avgs[2].ToString( "00.0000" );
							lblGZVar.Text = grGyro.Vars[2].ToString( "0.00000" );


							grAccel.UpdateStats();
							lblAXMin.Text = grAccel.Mins[0].ToString();
							lblAXMax.Text = grAccel.Maxs[0].ToString();
							lblAXAvg.Text = grAccel.Avgs[0].ToString( "00.0000" );
							lblAXVar.Text = grAccel.Vars[0].ToString( "0.00000" );

							lblAYMin.Text = grAccel.Mins[1].ToString();
							lblAYMax.Text = grAccel.Maxs[1].ToString();
							lblAYAvg.Text = grAccel.Avgs[1].ToString( "00.0000" );
							lblAYVar.Text = grAccel.Vars[1].ToString( "0.00000" );

							lblAZMin.Text = grAccel.Mins[2].ToString();
							lblAZMax.Text = grAccel.Maxs[2].ToString();
							lblAZAvg.Text = grAccel.Avgs[2].ToString( "00.0000" );
							lblAZVar.Text = grAccel.Vars[2].ToString( "0.00000" );
						}
					}
					break;


				case 0x04:	// IMU Test - Update the orientation quaternion
					Quaternion q = new Quaternion();
					q.x = GetCommFloat();
					q.y = GetCommFloat();
					q.z = GetCommFloat();
					q.w = GetCommFloat();

					Pitch = GetCommWord();
					Roll = GetCommWord();
					Yaw = GetCommWord();

					ocOrientation.Quat = q;
					gPitch.Value = Pitch;
					gRoll.Value = Roll;
					gYaw.Value = Yaw;
					break;

				case 0x05:	// IMU Comparison - Test different orientation update methods
					GyroX = GetCommWord();
					GyroY = GetCommWord();
					GyroZ = GetCommWord();

					AccelX = GetCommWord();
					AccelY = GetCommWord();
					AccelZ = GetCommWord();

					ComputeQuaternionOrientations();

					ocCompQ1.Quat = Q1;
					ocCompQ2.Quat = Q2;
					break;

			}
		}

		private float ComputeTiltCompensatedHeading()
		{
			// Compute pitch and roll from the current accelerometer vector - only accurate if stationary
			Vector v = new Vector( AccelX, AccelY, AccelZ );
			v = v.Normalize();

			float accPitch = (float)Math.Asin( -v.x );
			float accRoll =  (float)Math.Asin( v.y / Math.Cos(accPitch) );

			// Technically we should also calibrate the min/max readings from the mag first - this may not be accurate otherwise

			float Mxh = (float)(MagX * Math.Cos( accPitch ) + MagZ * Math.Sin( accPitch ));
			float Myh = (float)(MagX * Math.Sin( accRoll ) * Math.Sin( accPitch ) + MagY * Math.Cos( accRoll ) - MagZ * Math.Sin( accRoll ) * Math.Cos( accPitch ));

			float Heading = (float)(Math.Atan2( Mxh, Myh ) * 1800.0 / Math.PI);
			return Heading;
		}

		private void tcMainTabs_SelectedIndexChanged( object sender, EventArgs e )
		{
			// I used to handle the mode switch here, but it was causing problems
			// so I moved it into the timer-tick handler

			if(comm != null) {
				//currentMode = (Mode)(tcMainTabs.SelectedIndex+1);
				//txBuffer[0] = (byte)currentMode;
				//comm.Write( txBuffer, 0, 1 );	// Which mode are we in?
			}
		}

		private void TestMotor( int MotorIndex )
		{
			if(comm != null) {
				txBuffer[0] = (byte)(MotorIndex | 8);
				comm.Write( txBuffer, 0, 1 );
			}
		}


		private void btnMotor1_Click( object sender, EventArgs e ) {
			TestMotor( 0 );
		}

		private void btnMotor2_Click( object sender, EventArgs e ) {
			TestMotor( 1 );
		}

		private void btnMotor3_Click( object sender, EventArgs e ) {
			TestMotor( 2 );
		}

		private void btnMotor4_Click( object sender, EventArgs e ) {
			TestMotor( 3 );
		}

		private void btnBeeper_Click( object sender, EventArgs e ) {
			TestMotor( 4 );
		}

		private void btnLED_Click( object sender, EventArgs e ) {
			TestMotor( 5 );
		}

		private void btnResetCalib_Click( object sender, EventArgs e )
		{
			lfGraph.Reset();
		}


		private void btnUploadCalibration_Click( object sender, EventArgs e )
		{
			// Upload calibration data
			txBuffer[0] = 0x12;

			int scaleX = (int)(1.0 / lfGraph.dSlope.x + 0.5);
			int scaleY = (int)(1.0 / lfGraph.dSlope.y + 0.5);
			int scaleZ = (int)(1.0 / lfGraph.dSlope.z + 0.5);
			int offsetX = (int)(lfGraph.dIntercept.x + 0.5);
			int offsetY = (int)(lfGraph.dIntercept.y + 0.5);
			int offsetZ = (int)(lfGraph.dIntercept.z + 0.5);

			txBuffer[1] = (byte)(scaleX >> 8);
			txBuffer[2] = (byte)(scaleX >> 0);
			txBuffer[3] = (byte)(scaleY >> 8);
			txBuffer[4] = (byte)(scaleY >> 0);
			txBuffer[5] = (byte)(scaleZ >> 8);
			txBuffer[6] = (byte)(scaleZ >> 0);
			txBuffer[7] = (byte)(offsetX >> 8);
			txBuffer[8] = (byte)(offsetX >> 0);
			txBuffer[9] = (byte)(offsetY >> 8);
			txBuffer[10] = (byte)(offsetY >> 0);
			txBuffer[11] = (byte)(offsetZ >> 8);
			txBuffer[12] = (byte)(offsetZ >> 0);

			comm.Write( txBuffer, 0, 13 );
		}


		private void btnThrottleCalibrate_Click( object sender, EventArgs e )
		{
			switch(CalibrationCycle)
			{
				case 0:
					TestMotor( 6 );
					lblCalibrateDocs.Text = "Throttle calibration has started.  Be sure your flight battery is UNPLUGGED, then press the Throttle Calibration button again";
					CalibrationCycle = 1;

					// TODO - Should probably disable all other buttons, and make an abort button visible
					break;

				case 1:
					txBuffer[0] = (byte)0xFF;
					comm.Write( txBuffer, 0, 1 );
					lblCalibrateDocs.Text = "Plug in your flight battery and wait for the ESCs to beep twice, then press the Throttle Calibration button again";
					CalibrationCycle = 2;
					break;

				case 2:
					txBuffer[0] = (byte)Mode.MotorTest;
					comm.Write( txBuffer, 0, 1 );
					lblCalibrateDocs.Text = "Calibration complete";
					lblCalibrateDocs.Update();
					System.Threading.Thread.Sleep( 1000 * 3 );
					lblCalibrateDocs.Text = "";
					CalibrationCycle = 0;

					// TODO: Re-enable all other buttons, hide the abort button

					break;
			}
		}

		private void MainForm_FormClosing( object sender, FormClosingEventArgs e )
		{
			tickTimer.Enabled = false;

			try
			{
				currentMode = Mode.None;
				txBuffer[0] = (byte)(currentMode);
				comm.Write( txBuffer, 0, 1 );
			}

			catch(Exception)
			{
			}
		}

		private void btnMotor1_MouseDown( object sender, MouseEventArgs e )
		{
			((Button)sender).Capture = true;
			TestMotor( 0 );
		}

		private void btnMotor2_MouseDown( object sender, MouseEventArgs e )
		{
			((Button)sender).Capture = true;
			TestMotor( 1 );
		}

		private void btnMotor3_MouseDown( object sender, MouseEventArgs e )
		{
			((Button)sender).Capture = true;
			TestMotor( 2 );
		}

		private void btnMotor4_MouseDown( object sender, MouseEventArgs e )
		{
			((Button)sender).Capture = true;
			TestMotor( 3 );
		}

		private void btnMotor_MouseUp( object sender, MouseEventArgs e )
		{
			((Button)sender).Capture = false;
			TestMotor( 7 );	// Turn off all motors
		}


		float errCorrX1 = 0;
		float errCorrY1 = 0;
		float errCorrZ1 = 0;

		float errCorrX2 = 0;
		float errCorrY2 = 0;
		float errCorrZ2 = 0;

		int GZeroX = 0, GZeroY = 0, GZeroZ = 0;
		int GZeroCount = 0;

		const float GyroToDeg = 1000.0f / (17.5f * 4);
		const float RadToDeg = 180.0f / 3.141592654f;
		const float UpdateRate = 200.0f;
		const float GyroInvScale = GyroToDeg * RadToDeg * UpdateRate;
		const float GyroScale = 1.0f / GyroInvScale;

		const float AccToG = 1000.0f / 0.122f;					// Accelerometer per G @ 4g sensitivity = 0.122 mg/bit 
		const float AccScale = 1.0f / AccToG;

		Quaternion Q1 = new Quaternion( 1, 0, 0, 0 );
		Quaternion Q2 = new Quaternion( 1, 0, 0, 0 );
		Quaternion qdot = new Quaternion();

		Matrix m = new Matrix();
		Vector acc = new Vector();


		// Test different methods of updating orientation to see what works best
		void ComputeQuaternionOrientations()
		{
			if(GZeroCount < 64)
			{
				GZeroX += GyroX;
				GZeroY += GyroY;
				GZeroZ += GyroZ;

				GZeroCount++;
				if(GZeroCount == 64)
				{
					GZeroX /= 64;
					GZeroY /= 64;
					GZeroZ /= 64;
				}
				return;
			}

			ComputeQuatOriginalMethod();	// Original, as implemented on the Prop
			ComputeQuatAlternateMethod();	// Testing alternate
		}


		void ComputeQuatOriginalMethod()
		{
			float rx = (float)(GyroX - GZeroX) *  GyroScale + errCorrX1;
			float ry = (float)(GyroZ - GZeroZ) * -GyroScale + errCorrY1;
			float rz = (float)(GyroY - GZeroY) * -GyroScale + errCorrZ1;

			float rMag = (float)(Math.Sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) * 0.5);

			float cosr = (float)Math.Cos(rMag);
			float sinr = (float)Math.Sin(rMag) / rMag;

			qdot.w = -(rx * Q1.x + ry * Q1.y + rz * Q1.z) * 0.5f;
			qdot.x =  (rx * Q1.w + rz * Q1.y - ry * Q1.z) * 0.5f;
			qdot.y =  (ry * Q1.w - rz * Q1.x + rx * Q1.z) * 0.5f;
			qdot.z =  (rz * Q1.w + ry * Q1.x - rx * Q1.y) * 0.5f;

			Q1.w = cosr * Q1.w + sinr * qdot.w;
			Q1.x = cosr * Q1.x + sinr * qdot.x;
			Q1.y = cosr * Q1.y + sinr * qdot.y;
			Q1.z = cosr * Q1.z + sinr * qdot.z;

			Q1 = Q1.Normalize();

			// Convert to matrix form
			m.From(Q1);

			// Compute the difference between the accelerometer vector and the matrix Y (up) vector
			acc = new Vector( -AccelX, AccelZ, AccelY );
			rMag = acc.Length * AccScale;

			acc = acc.Normalize();
			float accWeight = 1.0f - Math.Min( Math.Abs( 2.0f - rMag * 2.0f ), 1.0f );

			float errDiffX = acc.y * m.m[1,2] - acc.z * m.m[1,1];
			float errDiffY = acc.z * m.m[1,0] - acc.x * m.m[1,2];
			float errDiffZ = acc.x * m.m[1,1] - acc.y * m.m[1,0];

			accWeight *= 1.0f / 512.0f;
			errCorrX1 = errDiffX * accWeight;
			errCorrY1 = errDiffY * accWeight;
			errCorrZ1 = errDiffZ * accWeight;
		}


		float lastGx = 0;
		float lastGy = 0;
		float lastGz = 0;

		void ComputeQuatAlternateMethod()
		{
			// Trapezoidal integration of gyro readings
			float rx = (float)((GyroX+lastGx)*0.5f - GZeroX) *  GyroScale + errCorrX2;
			float ry = (float)((GyroZ+lastGz)*0.5f - GZeroZ) * -GyroScale + errCorrY2;
			float rz = (float)((GyroY+lastGy)*0.5f - GZeroY) * -GyroScale + errCorrZ2;

			lastGx = GyroX;
			lastGy = GyroY;
			lastGz = GyroZ;

			float rMag = (float)(Math.Sqrt(rx * rx + ry * ry + rz * rz + 0.0000000001) * 0.5);

			float cosr = (float)Math.Cos(rMag);
			float sinr = (float)Math.Sin(rMag) / rMag;

			qdot.w = -(rx * Q2.x + ry * Q2.y + rz * Q2.z) * 0.5f;
			qdot.x =  (rx * Q2.w + rz * Q2.y - ry * Q2.z) * 0.5f;
			qdot.y =  (ry * Q2.w - rz * Q2.x + rx * Q2.z) * 0.5f;
			qdot.z =  (rz * Q2.w + ry * Q2.x - rx * Q2.y) * 0.5f;

			Q2.w = cosr * Q2.w + sinr * qdot.w;
			Q2.x = cosr * Q2.x + sinr * qdot.x;
			Q2.y = cosr * Q2.y + sinr * qdot.y;
			Q2.z = cosr * Q2.z + sinr * qdot.z;

			Q2 = Q2.Normalize();

			// Convert to matrix form
			m.From(Q2);

			// Compute the difference between the accelerometer vector and the matrix Y (up) vector
			acc = new Vector( -AccelX, AccelZ, AccelY );
			rMag = acc.Length * AccScale;

			acc = acc.Normalize();
			float accWeight = 1.0f - Math.Min( Math.Abs( 2.0f - rMag * 2.0f ), 1.0f );
			// accWeight *= accWeight * 4.0f;

			float errDiffX = acc.y * m.m[1,2] - acc.z * m.m[1,1];
			float errDiffY = acc.z * m.m[1,0] - acc.x * m.m[1,2];
			float errDiffZ = acc.x * m.m[1,1] - acc.y * m.m[1,0];

			accWeight *= 1.0f / 512.0f;
			errCorrX2 = errDiffX * accWeight;
			errCorrY2 = errDiffY * accWeight;
			errCorrZ2 = errDiffZ * accWeight;

			// At this point, errCorr represents a very small correction rotation vector, but in the WORLD frame
			// Rotate it into the current BODY frame

			//Vector errVect = new Vector( errCorrX2, errCorrY2, errCorrZ2 );
			//errVect = m.Transpose().Mul( errVect );
			//errCorrX2 = errVect.x;
			//errCorrY2 = errVect.y;
			//errCorrZ2 = errVect.z;
		}

	}
}
