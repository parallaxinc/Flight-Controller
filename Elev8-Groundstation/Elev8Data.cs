using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Elev8
{

	public class RadioData
	{
		public short Thro, Aile, Elev, Rudd;
		public short Gear, Aux1, Aux2, Aux3;						// Radio values = 16 bytes
		public short BatteryVolts;                                  // Battery Monitor = 2 bytes

		// Array index operator, allowing access to the channels by index value
		public short this[int i]
		{
			get
			{
				switch(i)
				{
					case 0: return Thro;
					case 1: return Aile;
					case 2: return Elev;
					case 3: return Rudd;
					case 4: return Gear;
					case 5: return Aux1;
					case 6: return Aux2;
					case 7: return Aux3;
					case 8: return BatteryVolts;
					default: return 0;
				}
			}

			set
			{
				switch(i)
				{
					case 0: Thro = value; break;
					case 1: Aile = value; break;
					case 2: Elev = value; break;
					case 3: Rudd = value; break;
					case 4: Gear = value; break;
					case 5: Aux1 = value; break;
					case 6: Aux2 = value; break;
					case 7: Aux3 = value; break;
					case 8: BatteryVolts = value; break;
				}
			}
		}


		public void ReadFrom( Packet p )
		{
			Thro = p.GetShort();
			Aile = p.GetShort();
			Elev = p.GetShort();
			Rudd = p.GetShort();
			Gear = p.GetShort();
			Aux1 = p.GetShort();
			Aux2 = p.GetShort();
			Aux3 = p.GetShort();
			BatteryVolts = p.GetShort();
		}
	}

	public class MotorData
	{
		public short FL, FR, BR, BL;	// front-left, front-right, back-right, back-left motor outputs

		public void ReadFrom( Packet p )
		{
			FL = p.GetShort();
			FR = p.GetShort();
			BR = p.GetShort();
			BL = p.GetShort();
		}
	}


	public class SensorData
	{
		public short Temp, GyroX, GyroY, GyroZ;
		public short AccelX, AccelY, AccelZ;						// IMU sensors = 20 bytes
		public short MagX, MagY, MagZ;


		public void ReadFrom( Packet p )
		{
			Temp = p.GetShort();
			GyroX = p.GetShort();
			GyroY = p.GetShort();
			GyroZ = p.GetShort();
			AccelX = p.GetShort();
			AccelY = p.GetShort();
			AccelZ = p.GetShort();
			MagX = p.GetShort();
			MagY = p.GetShort();
			MagZ = p.GetShort();
		}
	};

	public class DebugValues
	{
		public int LoopCycles;
		public float DebugFloat;

		public void ReadFrom( Packet p )
		{
			LoopCycles = p.GetInt();
			DebugFloat = p.GetFloat();
		}
	};


	public class ComputedData
	{
		public int Pitch, Roll, Yaw;								// IMU = 12 bytes
		public int Alt, AltTemp, AltiEst;							// Altimeter = 12 bytes


		public void ReadFrom( Packet p )
		{
			Pitch = p.GetInt();
			Roll = p.GetInt();
			Yaw = p.GetInt();

			Alt = p.GetInt();
			AltTemp = p.GetInt();
			AltiEst = p.GetInt();
		}
	};

}
