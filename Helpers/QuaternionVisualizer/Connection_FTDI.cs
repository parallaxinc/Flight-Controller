using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

using FTD2XX_NET;


namespace Elev8
{
	public class Connection_FTDI : Connection
	{
		public event ConnectionEvent ConnectionStarted;
		public event ConnectionEvent ConnectionEnded;

		FTDI ftdi = null;
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
					uint bytesRead = 0;
					lock(ftdi)
					{
						// read any available data
						uint bytesAvail = 0;
						FTDI.FT_STATUS stat = ftdi.GetRxBytesAvailable( ref bytesAvail );				// How much data is available from the serial port?
						if(stat == FTDI.FT_STATUS.FT_IO_ERROR)
						{
							// If we got an error, the port has likely been closed / unplugged - go back to waiting
							ftdi.Close();
							connected = false;
							if(ConnectionEnded != null) {
								ConnectionEnded();
							}
							continue;
						}

						if(bytesAvail > 0)
						{
							uint toRead = Math.Min( bytesAvail, (uint)rxBuffer.Length );
							if(toRead > 0) {
								ftdi.Read( rxBuffer, toRead, ref bytesRead );
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
			lock(ftdi)
			{
				uint written = 0;
				ftdi.Write( bytes, count, ref written );
			}
		}


		private void AttemptConnect()
		{
			connected = false;

			UInt32 DeviceCount = 0;
			FTDI.FT_STATUS ftStatus = FTDI.FT_STATUS.FT_OK;

			// Create new instance of the FTDI device class
			ftdi = new FTDI();

			// Determine the number of FTDI devices connected to the machine
			ftStatus = ftdi.GetNumberOfDevices( ref DeviceCount );

			// Check status
			if(ftStatus != FTDI.FT_STATUS.FT_OK || DeviceCount == 0)
			{
				commStat = CommStatus.NoDevice;
				return;
			}

			commStat = CommStatus.NoElev8;

			// Allocate storage for device info list
			FTDI.FT_DEVICE_INFO_NODE[] DeviceList = new FTDI.FT_DEVICE_INFO_NODE[DeviceCount];

			try
			{
				// Populate our device list
				ftStatus = ftdi.GetDeviceList( DeviceList );

				bool FoundElev8 = false;
				for(int i = 0; i < DeviceCount && FoundElev8 == false; i++)
				{
					if(DeviceList[i].Type != FTDI.FT_DEVICE.FT_DEVICE_X_SERIES) continue;

					ftStatus = ftdi.OpenBySerialNumber( DeviceList[i].SerialNumber );
					if(ftStatus == FTDI.FT_STATUS.FT_OK)
					{
						string portName;
						ftdi.GetCOMPort( out portName );
						if(portName == null || portName == "")
						{
							ftdi.Close();
							continue;
						}

						ftdi.SetBaudRate( 115200 );
						ftdi.SetDataCharacteristics( 8, 1, 0 );
						ftdi.SetFlowControl( FTDI.FT_FLOW_CONTROL.FT_FLOW_NONE, 0, 0 );


						txBuffer[0] = (byte)0;		// Set it to MODE_None to stop it writing (reset in case it was disconnected)
						txBuffer[1] = (byte)0xff;	// Send 0xff to the Prop to see if it replies
						uint written = 0;

						for(int j = 0; j < 10 && FoundElev8 == false; j++)	// Keep pinging until it replies, or we give up
						{
							ftdi.Write( txBuffer, 2, ref written );
							System.Threading.Thread.Sleep( 50 );

							uint bytesAvail = 0;
							ftdi.GetRxBytesAvailable( ref bytesAvail );				// How much data is available from the serial port?
							if(bytesAvail > 0)
							{
								uint bytesRead = 0;
								ftdi.Read( rxBuffer, 1, ref bytesRead );			// If it comes back with 0xE8 it's the one we want
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
							written = 0;
							ftdi.Write( txBuffer, 1, ref written );

							if(ConnectionStarted != null) {
								ConnectionStarted();
							}
							break;
						}
						else {
							ftdi.Close();
						}
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
			if( ftdi.IsOpen ) {
				ftdi.Close();
				connected = false;
			}
		}
	}
}
