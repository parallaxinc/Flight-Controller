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

							//bRadioChanged = true;
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

							ocCube.Quat = q;

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
	}
}
