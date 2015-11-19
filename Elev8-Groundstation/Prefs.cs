using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;

namespace Elev8
{
	[StructLayout( LayoutKind.Sequential, Pack = 1 )]
	public struct PREFS
	{
		public int DriftScaleX,  DriftScaleY,  DriftScaleZ;
		public int DriftOffsetX, DriftOffsetY, DriftOffsetZ;
		public int AccelOffsetX, AccelOffsetY, AccelOffsetZ;
		public int MagOfsX, MagScaleX, MagOfsY, MagScaleY, MagOfsZ, MagScaleZ;

		public float RollCorrectSin, RollCorrectCos;
		public float PitchCorrectSin, PitchCorrectCos;

		public char UseSBUS;
		public char UsePing;
		public char UseBattMon;
		public char DisableMotors;

		public char LowVoltageAlarm;
		public char LowVoltageAscentLimit;
		public char unused2;
		public char unused3;

		public short ThrustCorrectionScale;  // 0 to 256  =  0 to 1
		public short AccelCorrectionFilter;  // 0 to 256  =  0 to 1

		public short MaxRollPitch;
		public short RollPitchSpeed;
		public short YawSpeed;

		public short VoltageOffset;
		public short LowVoltageAlarmThreshold;  // default is 1050 (10.50v)

		public short ArmDelay;
		public short DisarmDelay;

		public short ThrottleTest;     // Typically the same as MinThrottleArmed, unless MinThrottleArmed is too low for movement

		public short MinThrottle;      // Minimum motor output value
		public short MaxThrottle;      // Maximum motor output value
		public short CenterThrottle;   // Mid-point motor output value
		public short MinThrottleArmed; // Minimum throttle output value when armed - MUST be equal or greater than MinThrottle

		public char ThroChannel;      // Radio inputs to use for each value
		public char AileChannel;
		public char ElevChannel;
		public char RuddChannel;
		public char GearChannel;
		public char Aux1Channel;
		public char Aux2Channel;
		public char Aux3Channel;

		public short ThroScale;
		public short AileScale;
		public short ElevScale;
		public short RuddScale;
		public short GearScale;
		public short Aux1Scale;
		public short Aux2Scale;
		public short Aux3Scale;

		public short ThroCenter;
		public short AileCenter;
		public short ElevCenter;
		public short RuddCenter;
		public short GearCenter;
		public short Aux1Center;
		public short Aux2Center;
		public short Aux3Center;


		public int Checksum;


		public short GetChannelScale( int i )
		{
			switch(i)
			{
				case 0: return ThroScale;
				case 1: return AileScale;
				case 2: return ElevScale;
				case 3: return RuddScale;
				case 4: return GearScale;
				case 5: return Aux1Scale;
				case 6: return Aux2Scale;
				case 7: return Aux3Scale;
				default: return 1024;
			}
		}

		public void SetChannelScale( int i , short val )
		{
			switch(i)
			{
				case 0: ThroScale = val; break;
				case 1: AileScale = val; break;
				case 2: ElevScale = val; break;
				case 3: RuddScale = val; break;
				case 4: GearScale = val; break;
				case 5: Aux1Scale = val; break;
				case 6: Aux2Scale = val; break;
				case 7: Aux3Scale = val; break;
			}
		}

		public void SetChannelCenter( int i, short val )
		{
			switch(i)
			{
				case 0: ThroCenter = val; break;
				case 1: AileCenter = val; break;
				case 2: ElevCenter = val; break;
				case 3: RuddCenter = val; break;
				case 4: GearCenter = val; break;
				case 5: Aux1Center = val; break;
				case 6: Aux2Center = val; break;
				case 7: Aux3Center = val; break;
			}
		}

		public void SetChannelIndex( int i, char Index )
		{
			switch(i)
			{
				case 0: ThroChannel = Index; break;
				case 1: AileChannel = Index; break;
				case 2: ElevChannel = Index; break;
				case 3: RuddChannel = Index; break;
				case 4: GearChannel = Index; break;
				case 5: Aux1Channel = Index; break;
				case 6: Aux2Channel = Index; break;
				case 7: Aux3Channel = Index; break;
			}
		}


		public static Byte[] SerializeMessage<T>( T msg ) where T : struct
		{
			int objsize = Marshal.SizeOf( typeof( T ) );
			Byte[] ret = new Byte[objsize];
			IntPtr buff = Marshal.AllocHGlobal( objsize );
			Marshal.StructureToPtr( msg, buff, true );
			Marshal.Copy( buff, ret, 0, objsize );
			Marshal.FreeHGlobal( buff );
			return ret;
		}

		public static T DeserializeMsg<T>( Byte[] data ) where T : struct
		{
			int objsize = Marshal.SizeOf( typeof( T ) );
			IntPtr buff = Marshal.AllocHGlobal( objsize );
			Marshal.Copy( data, 0, buff, objsize );
			T retStruct = (T)Marshal.PtrToStructure( buff, typeof( T ) );
			Marshal.FreeHGlobal( buff );
			return retStruct;
		}


		public byte[] ToBytes()
		{
			byte[] byteArray = SerializeMessage<PREFS>( this );
			return byteArray;
		}


		public void FromBytes( byte[] byteArray )
		{
			try
			{
				object d = DeserializeMsg<PREFS>( byteArray );
				this = (PREFS)d;
			}

			catch(Exception)
			{
			}
		}


		public int CalculateChecksum()
		{
			byte[] bytes = ToBytes();
			UInt32[] longs = new UInt32[bytes.Length / 4];

			int longCount = (bytes.Length/4) - 1;	// Subtract one to remove the checksum value

			for(int i = 0; i < longCount; i++)
			{
				longs[i] = BitConverter.ToUInt32( bytes, i*4 );
			}


			UInt32 r = 0x55555555;            //Start with a strange, known value
			for(int i = 0; i < longCount; i++)
			{
				r = (r << 7) | (r >> (32 - 7));
				r = r ^ longs[i];     //Jumble the bits, XOR in the prefs value
			}
			return (int)r;
		}

	};

}
