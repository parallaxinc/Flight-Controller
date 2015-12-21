using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using Elev8;

namespace FilterSample
{
	public partial class Main : Form
	{
		SerialPort port = null;
		bool connected = false;
		byte[] rxBuf = new byte[64];
		int bufPos = 0;
		StringBuilder str = new StringBuilder();

		int GyroY, AccelX, AccelZ;
		int GyroZero = 0;
		int GyroZeroStep = 0;
		int stage = 0;

		float GyroDegrees = 0.0f;		// Unfiltered version using gyro only readings
		float FilteredDegrees = 0.0f;


		public Main()
		{
			InitializeComponent();

			Connect();
			ShowFilterValue();

			tmTick.Enabled = true;
		}


		void Connect()
		{
			string[] names = SerialPort.GetPortNames();

			List<string> Ports = new List<string>();
			foreach (string name in names)
			{
				if (name != "COM1" && name != "COM2" && name != "COM3" && name != "COM4")
					Ports.Add(name);
			}

			for (int i = 0; i < Ports.Count && connected == false; i++)
			{
				port = new SerialPort(Ports[i], 115200, Parity.None, 8, StopBits.One);
				port.Open();

				System.Threading.Thread.Sleep(50);

				int bytesAvail = port.BytesToRead;
				if (bytesAvail > 0)
				{
					while (bytesAvail > 0)
					{
						int toRead = Math.Min(bytesAvail, rxBuf.Length);
						int bytesRead = port.Read(rxBuf, 0, toRead );

						if (bytesRead == 0) {
							bytesAvail = 0;
							break;
						}

						for (int j = 0; j < bytesRead; j++)
						{
							if (rxBuf[j] == '$')
							{
								connected = true;
								bytesAvail = 0;
								break;
							}
						}
					}
				}
			}
		}


		private void tmTick_Tick(object sender, EventArgs e)
		{
			if (connected == false) return;
			ReadData();
		}


		void ReadData()
		{
			while (port.BytesToRead > 0)
			{
				int bytesAvail = port.BytesToRead;
				int bufRemain = rxBuf.Length - bufPos;
				int bytesToRead = Math.Min(bytesAvail, bufRemain);
				bufPos += port.Read(rxBuf, bufPos, bytesToRead);

				int procPos = 0;
				while (procPos < bufPos)
				{
					ProcessChar( (char)rxBuf[procPos++] );
				}
				bufPos = 0;
			}
		}

		void ProcessChar( char c )
		{
			switch (stage)
			{
				case 0:
					if (c == '$')	//when we find the $ (packet start), begin building the first number
					{
						str.Clear();
						stage = 1;
					}
					break;

				case 1:
					if (c == ' ')	// When we encounter a space, we're moving on to the next number
					{
						GyroY = int.Parse(str.ToString());	// Parse the string into a digit
						str.Clear();						// Reset the string
						stage = 2;							// Advance to the next number
					}
					else str.Append(c);	// still building this one - just append
					break;

				case 2:
					if (c == ' ')	// When we encounter a space, we're moving on to the next number
					{
						AccelX = int.Parse(str.ToString());	// Parse the string into a digit
						str.Clear();						// Reset the string
						stage = 3;							// Advance to the next number
					}
					else str.Append(c);	// still building this one - just append
					break;

				case 3:
					if (c == ' ')	// hit the final space - should now have both numbers
					{
						AccelZ = int.Parse(str.ToString());	// parse the second number out of the string
						str.Clear();

						UpdateComplimentaryFilter();

						stage = 0;
					}
					else str.Append(c);		// Still building up the second number
					break;
			}
		}

		// Keep the value within +/- 180 degrees
		float RangeCheck(float val)
		{
			if (val < -180.0f) {
				val += 360.0f;
			}
			else if (val > 180.0f) {
				val -= 360.0f;
			}
			return val;
		}

		void UpdateComplimentaryFilter()
		{
			// Compute an average of the first 16 gyro values to get a good zero reading
			if( GyroZeroStep < 16 )
			{
				GyroZero += GyroY;
				GyroZeroStep++;
				if (GyroZeroStep == 16) {
					GyroZero /= 16;
				}
				return;
			}

			int FilterVal = hsStrength.Value;	// this is a value from 0 to 100

			float AccelRad = (float)Math.Atan2(AccelX, AccelZ);
			float AccelDegrees = (float)(AccelRad * 180.0 / Math.PI);

			// GyroStep is how much we've moved in this 100th of a second, in degrees
			float GyroStep = (float)((GyroY - GyroZero) * (70.0 / 1000.0) * (1.0 / 100.0));

			GyroDegrees += GyroStep;
			GyroDegrees = RangeCheck(GyroDegrees);

			FilteredDegrees += GyroStep;
			FilteredDegrees = RangeCheck(FilteredDegrees);

			// If one value is 179 degrees, and the other is -179 degrees
			// they're actually only 2 degrees apart.  Figure out and correct for this if necessary.
			float diff = Math.Abs(FilteredDegrees - AccelDegrees);
			if (diff > 180.0f)
			{
				if (FilteredDegrees < 0) FilteredDegrees += 360.0f;
				else FilteredDegrees -= 360.0f;
			}

			float GyroMix = (float)FilterVal / 100.0f;
			float AccelMix = 1.0f - GyroMix;

			// This is the complimentary filter part:
			FilteredDegrees = (FilteredDegrees * GyroMix) + (AccelDegrees * AccelMix);

			FilteredDegrees = RangeCheck(FilteredDegrees);	// Make sure the result is in the +/- 180 range

			gaugeAccel.Value = (float)AccelDegrees;
			gaugeGyro.Value = (float)GyroDegrees;
			gaugeFiltered.Value = (float)FilteredDegrees;
		}

		private void hsStrength_ValueChanged(object sender, EventArgs e)
		{
			ShowFilterValue();
		}


		void ShowFilterValue()
		{
			int val = hsStrength.Value;

			lblStrength.Text = String.Format("Mix: {0}% accel, {1}% gyro",
				val, 100-val);
		}
	}
}
