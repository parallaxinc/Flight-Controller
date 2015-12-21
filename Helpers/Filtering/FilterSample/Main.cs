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

		int GyroY, GyroYFilt;
		int stage = 0;
		int count = 0;

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
					if (c == ' ')	// hit the final space - should now have both numbers
					{
						GyroYFilt = int.Parse(str.ToString());	// parse the second number out of the string
						str.Clear();

						int[] sample = new int[3];				// Make a new sample to add to the graph

						sample[0] = GyroY;
						sample[1] = GyroYFilt;					// Fill in the sample data (expects 3 values, so we duplicate one of them)
						sample[2] = sample[1];

						grGraph.AddSample(sample, true);		// Add to the graph
						count++;
						if (count == 20)
						{
							grGraph.UpdateStats();				// adjust the graph range every 20 samples
							count = 0;
						}
						stage = 0;
					}
					else str.Append(c);		// Still building up the second number
					break;
			}
		}


		private void hsStrength_ValueChanged(object sender, EventArgs e)
		{
			UpdateFilterValue();
			ShowFilterValue();
		}


		void UpdateFilterValue()
		{
			int val = hsStrength.Value;
			byte[] txBuf = new byte[2];
			txBuf[0] = (byte)('0' + val);
			port.Write(txBuf, 0, 1);
		}

		void ShowFilterValue()
		{
			int val = hsStrength.Value;

			int filt = 13 + val * 27;

			lblStrength.Text = String.Format("Filter: {0} / 256 = {1:0.###}",
				filt, (float)filt / 256.0f);
		}
	}
}
