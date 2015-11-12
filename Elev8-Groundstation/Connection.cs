using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;


namespace Elev8
{
	public enum CommStatus
	{
		Initializing,
		NoDevice,
		NoElev8,
		Connected,
	}

	public class Packet
	{
		public byte mode, len;
		public byte[] data;
		public byte index = 0;


		public byte GetByte()
		{
			return data[index++];
		}

		public short GetShort()
		{
			short val = (short)BitConverter.ToInt16( data, index );
			index += 2;
			return val;
		}

		public float GetFloat()
		{
			float Result = BitConverter.ToSingle( data, index );
			index += 4;
			return Result;
		}

		public int GetInt()
		{
			int Result = BitConverter.ToInt32( data, index );
			index += 4;
			return Result;
		}
	}


	// Declare the interface that will be used for any Connection objects (raw serial, or FDTI)

	public interface Connection
	{
		bool Active {
			set;
		}

		bool Connected {
			get;
		}

		CommStatus Status
		{
			get;
		}


		void Start();
		void Stop();

		Packet GetPacket();

		void Send( byte[] bytes , int Count );
	}
}
