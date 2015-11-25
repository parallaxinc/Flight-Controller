using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;


namespace Elev8
{
	public class Connection_Serial : Connection
	{
		public event ConnectionEvent ConnectionStarted;
		public event ConnectionEvent ConnectionEnded;

		SerialPort serial = null;
		volatile bool quit = false;
		volatile bool connected = false;
		volatile bool active = true;

		CommStatus commStat = CommStatus.NoDevice;

		byte[] txBuffer = new byte[32];
		byte[] rxBuffer = new byte[128];


		Packet[] packetsArray = new Packet[256];
		Packet currentPacket = null;
		int head = 0;
		int tail = 0;
		

		Thread thread = null;


		public bool Active {
			set { active = value; }
		}


		public bool Connected {
			get { return connected; }
		}


		public CommStatus Status {
			get { return commStat; }
		}


		public void Start()
		{
			quit = false;
			connected = false;
			thread = new Thread( new ThreadStart(CommThread) );
			thread.Start();
		}

		public void Stop()
		{
			quit = true;
			thread.Join();
			thread = null;
		}



		private void CommThread()
		{
			do
			{
				Thread.Sleep( 10 );
				if(active == false) {
					continue;
				}

				if(!Connected)
				{
					AttemptConnect();
				}
				else
				{
					int bytesRead = 0;
					lock(serial)
					{
						// read any available data
						int bytesAvail = serial.BytesToRead;
						if( serial.IsOpen == false )
						{
							// If we got an error, the port has likely been closed / unplugged - go back to waiting
							serial.Close();
							serial.Dispose();
							serial = null;
							connected = false;
							if(ConnectionEnded != null) {
								ConnectionEnded();
							}
							continue;
						}

						if(bytesAvail > 0)
						{
							int toRead = Math.Min( bytesAvail, rxBuffer.Length );
							if(toRead > 0) {
								bytesRead = serial.Read( rxBuffer, 0, toRead );
							}
						}
					}

					for( int i=0; i<bytesRead; i++) {
						ProcessByte( rxBuffer[i] );
					}
				}
			} while(!quit);

			Disconnect();
		}


		int sigByteIndex = 0;
		int packetByteIndex = 0;

		private void ProcessByte( byte b )
		{
			if(sigByteIndex < 2)
			{
				if(b == 0x77)
				{
					sigByteIndex++;
					packetByteIndex = 0;
					return;
				}
				else
				{
					sigByteIndex = 0;
					packetByteIndex = 0;
					return;
				}
			}

			switch( packetByteIndex )
			{
				case 0:
					currentPacket = new Packet();
					currentPacket.mode = b;
					packetByteIndex++;
					return;

				case 1:
					currentPacket.len = b;
					currentPacket.data = new byte[currentPacket.len];
					packetByteIndex++;
					return;

				default:
					currentPacket.data[packetByteIndex-2] = b;
					break;
			}

			packetByteIndex++;
			if(packetByteIndex == (currentPacket.data.Length + 2))
			{
				sigByteIndex = 0;
				packetByteIndex = 0;
				packetsArray[head] = currentPacket;

				lock(packetsArray)
				{
					head = (head+1) % packetsArray.Length;

					if(tail == head) {
						tail = (tail + 1) % packetsArray.Length;		// Throw away oldest data if we fill the buffer
					}
				}
			}
		}


		public Packet GetPacket()
		{
			lock(packetsArray)
			{
				if(head == tail) return null;

				Packet p = packetsArray[tail];
				tail = (tail + 1) % packetsArray.Length;
				return p;
			}
		}


		public void Send( byte[] bytes , int count )
		{
			if(connected == false) return;
			lock(serial)
			{
				serial.Write( bytes, 0, count );
			}
		}


		private void AttemptConnect()
		{
			connected = false;

			string[] names = SerialPort.GetPortNames();

			List<string> Ports = new List<string>();
			foreach(string name in names)
			{
				if(name != "COM1" && name != "COM2" && name != "COM3" && name != "COM4")
					Ports.Add( name );
			}

			// Check status
			if(Ports.Count == 0)
			{
				commStat = CommStatus.NoDevice;
				return;
			}

			commStat = CommStatus.NoElev8;

			try
			{

				bool FoundElev8 = false;
				for(int i = 0; i < Ports.Count && FoundElev8 == false; i++)
				{
					serial = new SerialPort( Ports[i], 115200, Parity.None, 8, StopBits.One );
					serial.Open();

					txBuffer[0] = (byte)0;		// Set it to MODE_None to stop it writing (reset in case it was disconnected)
					txBuffer[1] = (byte)0xff;	// Send 0xff to the Prop to see if it replies

					for(int j = 0; j < 10 && FoundElev8 == false; j++)	// Keep pinging until it replies, or we give up
					{
						serial.Write( txBuffer, 0, 2 );
						System.Threading.Thread.Sleep( 50 );

						int bytesAvail = serial.BytesToRead;
						if(bytesAvail > 0)
						{
							int bytesRead = serial.Read( rxBuffer, 0, 1 );			// If it comes back with 0xE8 it's the one we want
							if(bytesRead == 1 && rxBuffer[0] == 0xE8)
							{
								FoundElev8 = true;
								commStat = CommStatus.Connected;
								break;
							}
						}
					}

					if(FoundElev8) {
						connected = true;
						txBuffer[0] = 2;	// MODE_Sensors
						serial.Write( txBuffer, 0, 1 );

						if(ConnectionStarted != null) {
							ConnectionStarted();
						}
						break;
					}
					else {
						serial.Close();
						serial.Dispose();
						serial = null;
					}
				}
			}

			catch(Exception)
			{
				return;
			}
		}


		void Disconnect()
		{
			if( serial.IsOpen ) {
				serial.Close();
				serial.Dispose();
				serial = null;
				connected = false;
			}
		}
	}
}
