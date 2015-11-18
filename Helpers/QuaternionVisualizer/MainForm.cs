using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using Elev8;

namespace QuaternionVisualizer
{
	public partial class MainForm : Form
	{
		Connection_FTDI comm = new Connection_FTDI();

		//CommStatus status = CommStatus.Initializing;

		RadioData radio = new RadioData();
		SensorData sensors = new SensorData();
		Quaternion q = new Quaternion();
		ComputedData computed = new ComputedData();

		byte[] txBuffer = new byte[10];

		Point mousePt;
		PointF originalRot = new PointF();
		bool dragging = false;

		Quaternion desired = new Quaternion();


		public MainForm()
		{
			InitializeComponent();
			comm.ConnectionStarted += new ConnectionEvent( comm_ConnectionStarted );

			comm.Start();
		}


		void comm_ConnectionStarted()
		{
			//txBuffer[0] = 0x2;
			//comm.Send( txBuffer, 1 );	// Tell the Elev8 to start sending data
		}


		private void tmCommTimer_Tick( object sender, EventArgs e )
		{
			ProcessPackets();
		}


		void ProcessPackets()
		{
			//bool bRadioChanged = false;
			//bool bSensorsChanged = false;
			bool bQuatChanged = false;
			//bool bComputedChanged = false;

			Packet p;
			do {
				p = comm.GetPacket();
				if(p != null)
				{
					switch( p.mode )
					{
						case 1:	// Radio data
							radio.ReadFrom( p );

							lblCycleCount.Text = string.Format( "{0} cycles", radio.CycleCount );

							//bRadioChanged = true;
							//UpdateRadio();
							break;

						case 2:	// Sensor values
							sensors.ReadFrom( p );

							//bSensorsChanged = true;
							break;

						case 3:	// Quaternion
							q.x = p.GetFloat();
							q.y = p.GetFloat();
							q.z = p.GetFloat();
							q.w = p.GetFloat();

							//if(radio.Gear > -300 && radio.Gear < 300)
							{
								ocCube.Quat = q;
							}

							bQuatChanged = true;
							break;

						case 4:	// Compute values
							computed.ReadFrom(p);
							//bComputedChanged = true;

							//SampleIndex = (SampleIndex + 1) % NumGraphDisplaySamples;
							break;
					}
				}
			} while(p != null);


			if(bQuatChanged)
			{
				float vx = ocCube.Velocities.x;
				float vy = ocCube.Velocities.y;
				float vz = ocCube.Velocities.z;

				lblAngles.Text = string.Format(
					"rx: {0:0.000},  ry: {1:0.000},  rz: {2:0.000}", vx, vy, vz );
			}
		}


		private void MainForm_FormClosing( object sender, FormClosingEventArgs e )
		{
			comm.Stop();
		}


		private void ocCube_MouseDown( object sender, MouseEventArgs e )
		{
			dragging = true;
			mousePt = new Point( e.X, e.Y );
			originalRot = new PointF( ocCube.rx, ocCube.ry );
		}


		private void ocCube_MouseUp( object sender, MouseEventArgs e )
		{
			dragging = false;
		}

		private void ocCube_MouseMove( object sender, MouseEventArgs e )
		{
			if(dragging == false) return;

			PointF mouseDelta = new PointF( e.X - mousePt.X, e.Y - mousePt.Y );

			float ry = originalRot.Y - mouseDelta.X * 0.01f;
			float rx = originalRot.X - mouseDelta.Y * 0.01f;

			ocCube.rx = rx;
			ocCube.ry = ry;
			ocCube.Invalidate();
		}

		static float heading = 0.0f;


		private void UpdateRadio()
		{
			const float RadioScale = 1024.0f;
			const float Deg2Rad = 3.141592654f / 180.0f;

			float MaxRate = 180.0f;	// degrees per second
			float UpdateRate = 250.0f / 8.0f;

			float RateScale = ((MaxRate / UpdateRate) / RadioScale) * Deg2Rad * 0.5f;	// Quaternions use half-angles, so mult everything by 0.5

			if(radio.Gear < -300)
			{
				// Manual mode

				float xrot = (float)radio.Elev * RateScale;	// Individual scalars for channel sensitivity
				float yrot = (float)radio.Rudd * RateScale;
				float zrot = (float)radio.Aile * -RateScale;
	
				// Take Aile/Elev/Rudd and create an incremental quaternion
				Quaternion qrot = new Quaternion( 0, xrot, yrot, zrot );

				// rotate the current desired orientation by it
				desired = desired + (desired * qrot);	// Overall scale (meant as time increment)
				desired = desired.Normalize();

				// Need to extract "heading" from the IMU in case we switch to auto again

				ocCube.Quat = desired;
			}
			else if(radio.Gear > 300)
			{
				// Auto-level

				float MaxBank = 45.0f;
				float ScaleMult = (MaxBank / RadioScale) * Deg2Rad * 0.5f;	// Quaternions use half-angles, so mult everything by 0.5

				heading += (float)radio.Rudd * RateScale;

				float rx = (float)radio.Elev *  ScaleMult;	// Individual scalars for channel sensitivity
				float ry = heading;
				float rz = (float)radio.Aile * -ScaleMult;

				// Convert Aile/Elev/heading directly into desired orientation quaternion

				float csx = (float)Math.Cos(rx);
				float csy = (float)Math.Cos(ry);
				float csz = (float)Math.Cos(rz);
				float snx = (float)Math.Sin(rx);
				float sny = (float)Math.Sin(ry);
				float snz = (float)Math.Sin(rz);


				//Quaternion qz = new Quaternion( csz, 0, 0, snz );
				//Quaternion qy = new Quaternion( csy, 0, sny, 0 );
				//Quaternion qx = new Quaternion( csx, snx, 0, 0 );
				//desired = qy * qz * qx;


				// Simplifies to:

				float snycsx = sny * csx;
				float snysnx = sny * snx;
				float csycsz = csy * csz;
				float csysnz = csy * snz;

				desired.x =  snycsx * snz + csycsz * snx;
				desired.y =  snycsx * csz + csysnz * snx;
				desired.z =  csysnz * csx - snysnx * csz;
				desired.w =  csycsz * csx - snysnx * snz;

				ocCube.Quat = desired;
			}

			// Compute rotation from current orientation to desired

			// Vector Axis;
			// Quaternion qrot = follow.To( quat );
			// float angle = qrot.ToAngleAxis( out Axis );

			// Vector Velocities = Axis * angle;

			// apply through PIDs
		}
	}
}
