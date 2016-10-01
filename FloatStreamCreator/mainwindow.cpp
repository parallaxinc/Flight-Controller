#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>

#include "functioncompiler.h"
#include "functionstream.h"
#include <QFile>

#include "quatutil.h"
#include <math.h>

static char beatString[] = "BEAT";


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	pointsUsed = 0;
	pointIndex = 0;

	labelStatus = new QLabel(this);
	labelGSVersion = new QLabel(this);
	labelFWVersion = new QLabel(this);

	// set text for the label
	labelStatus->setText("Connecting...");
	labelStatus->setContentsMargins( 5, 1, 5, 1 );
	labelGSVersion->setText("FloatCompiler Version 1.0.0");
	labelFWVersion->setText( "Firmware Version -.-.-");

	// add the controls to the status bar
	ui->statusBar->addPermanentWidget(labelStatus, 1);
	ui->statusBar->addPermanentWidget(labelGSVersion, 1);
	ui->statusBar->addPermanentWidget(labelFWVersion, 1);

	ui->statusBar->setStyleSheet( "QStatusBar::item { border: 0px solid black }; ");

	labelStatus->setFrameStyle(QFrame::NoFrame);
	labelGSVersion->setFrameStyle(QFrame::NoFrame);
	labelFWVersion->setFrameStyle(QFrame::NoFrame);

	this->startTimer(25, Qt::PreciseTimer);		// 40 updates / sec
	comm.StartConnection();
	Heartbeat = 0;

	//on_btnCompile_clicked();
}

MainWindow::~MainWindow()
{
	comm.StopConnection();
	delete ui;
}

void MainWindow::timerEvent(QTimerEvent * e)
{
	(void)e;	// prevent unused parameter warning
	UpdateStatus();

	Heartbeat++;
	if( Heartbeat >= 20 ) {
		Heartbeat = 0;
		SendCommand( beatString );	// Send the connection heartbeat
	}

	ProcessPackets();
}

void MainWindow::UpdateStatus(void)
{
	CommStatus newStat = comm.Status();
	if(newStat != stat)
	{
		stat = newStat;
		switch( stat )
		{
		case CS_Initializing:
			labelStatus->setText("Initializing");
			break;

		case CS_NoDevice:
			labelStatus->setText("No serial devices found");
			break;

		case CS_NoElev8:
			labelStatus->setText("No Elev-8 found");
			break;

		case CS_Connected:
			labelStatus->setText("Connected");
			break;
		}
	}
}

void MainWindow::SendCommand( const char * command )
{
	comm.Send( (quint8*)command, 4 );
}

void MainWindow::SendCommand( QString command )
{
	QByteArray arr = command.toLocal8Bit();
	comm.Send( (quint8*)arr.constData(), arr.length() );
}


const float PI = 3.141592654f;

void MainWindow::ProcessPackets(void)
{
	bool bRadioChanged = false;
	bool bDebugChanged = false;
	bool bSensorsChanged = false;
	bool bQuatChanged = false;
	bool bTargetQuatChanged = false;
	bool bMotorsChanged = false;
	bool bComputedChanged = false;

	packet * p;
	do {
		p = comm.GetPacket();
		if(p != 0)
		{
			switch( p->mode )
			{
				case 1:	// Radio data
					packedRadio.ReadFrom( p );
					radio = packedRadio;

					//AddGraphSample( 16, (float)radio.BatteryVolts );
					bRadioChanged = true;
					break;

				case 2:	// Sensor values
					sensors.ReadFrom( p );

					QuatUpdate( sensors );
					Quat_GetQuaternion( cq );

					ui->Orientation_display->setQuat2(cq);

					{
						QPointF pt;
						pt.setX(sensors.MagX);
						pt.setY(sensors.MagY);
						ui->xy_mag->AddSample(pt, true);

						pt.setY(sensors.MagZ);
						ui->xz_mag->AddSample(pt, true);

						pt.setX(sensors.MagY);
						ui->yz_mag->AddSample(pt, true);
					}

					UpdateMagSphere();

					//AddGraphSample( 0, sensors.GyroX );
					//AddGraphSample( 1, sensors.GyroY );
					//AddGraphSample( 2, sensors.GyroZ );
					//AddGraphSample( 3, sensors.AccelX );
					//AddGraphSample( 4, sensors.AccelY );
					//AddGraphSample( 5, sensors.AccelZ );
					//AddGraphSample( 6, sensors.MagX );
					//AddGraphSample( 7, sensors.MagY );
					//AddGraphSample( 8, sensors.MagZ );
					//AddGraphSample( 9, sensors.Temp );

					//ahrs.Update( sensors , (1.0/250.f) * 8.0f , false );

					bSensorsChanged = true;
					break;

				case 3:	// Quaternion
					{
					q.setX(      p->GetFloat() );
					q.setY(      p->GetFloat() );
					q.setZ(      p->GetFloat() );
					q.setScalar( p->GetFloat() );

					// TEST CODE
					//q = ahrs.quat;

					QMatrix3x3 m;
					m = QuatToMatrix( q );

					float roll = asinf(  m(1, 0) );
					float pitch = asinf( m(1, 2) );
					//float yaw = -atan2f( m(2, 0), m(2, 2) );

					float xh = sensors.MagX * cosf(pitch) + sensors.MagZ * sinf(pitch);
					float yh = sensors.MagX * sinf(roll) * sinf(pitch) + sensors.MagY * cosf(roll) - sensors.MagZ * sinf(roll)*cosf(pitch);

					float heading = atan2(yh, xh);

					ui->Heading_display->setHeading( heading * (180.0/PI) );

					//AddGraphSample( 13, (float)pitch );
					//AddGraphSample( 14, (float)roll );
					//AddGraphSample( 15, (float)yaw );

					bQuatChanged = true;
					}
					break;

				case 4:	// Compute values
					computed.ReadFrom( p );
					bComputedChanged = true;

					//AddGraphSample( 10, (float)computed.Alt );
					//AddGraphSample( 11, (float)computed.AltiEst );
					//AddGraphSample( 12, (float)computed.GroundHeight );
					break;

				case 5:	// Motor values
					motors.ReadFrom( p );
					bMotorsChanged = true;
					break;

				case 6:	// Control quaternion
					//cq.setX      ( p->GetFloat() );
					//cq.setY      ( p->GetFloat() );
					//cq.setZ      ( p->GetFloat() );
					//cq.setScalar ( p->GetFloat() );
					//bTargetQuatChanged = true;

					// this is actually the last packet sent by the quad, so use this to advance the sample index
					//SampleIndex++;
					break;


				case 7:	// Debug data
					debugData.ReadFrom( p );
					bDebugChanged = true;
					break;

				case 0x18:	// Settings
					{
						//PREFS tempPrefs;
						//memset( &tempPrefs, 0, sizeof(tempPrefs) );
						//
						//quint32 toCopy = p->len;
						//if( sizeof(tempPrefs) < toCopy )
						//	toCopy = sizeof(tempPrefs);
						//
						//memcpy( &tempPrefs, p->data, toCopy );
						//
						//if( Prefs_CalculateChecksum( tempPrefs ) == tempPrefs.Checksum ) {
						//	//PrefsReceived = true;	// Global indicator of valid prefs
						//	bPrefsChanged = true;	// local indicator, just to set up the UI
						//	prefs = tempPrefs;
						//}
						//else {
						//	SendCommand( "QPRF" );	// reqeust them again because the checksum failed
						//}
					}
					break;
			}
			delete p;
		}
	} while(p != 0);

	if( bRadioChanged )
	{
	}

	if( bMotorsChanged )
	{
	}

	if(bQuatChanged) {
		ui->Orientation_display->setQuat(q);

		//QMatrix3x3 m;
		//m = QuatToMatrix( q );

		//float roll =   asin( m(1,0) ) * (180.0f/PI);
		//float pitch =  asin( m(1,2) ) * (180.0f/PI);
		//float yaw =  -atan2( m(2,0), m(2,2) ) * (180.0f/PI);

		//ui->lblRoll->setText( QString::number( roll, 'f', 1) );
		//ui->lblPitch->setText( QString::number( pitch, 'f', 1) );
		//ui->lblYaw->setText( QString::number( yaw, 'f', 1) );


		// Use the matrix and magnetometer here to experiment

		//ui->Horizon_display->setAngles( roll, pitch );
		//ui->Heading_display->setHeading(yaw);
	}

	if( bSensorsChanged )
	{
	}

	if(bTargetQuatChanged)
	{
		//ui->Orientation_display->setQuat2(cq);
	}

	if( bDebugChanged )
	{
		int verHigh = debugData.Version >> 8;
		int verMid  = (debugData.Version >> 4) & 15;
		int verLow  = (debugData.Version >> 0) & 15;

		labelFWVersion->setText( QString( "Firmware Version %1.%2.%3" ).arg(verHigh).arg(verMid).arg(verLow) );

		ui->lblCycles->setText( QString(
			"CPU time (uS): %1 (min), %2 (max), %3 (avg)" ).arg( debugData.MinCycles * 64/80 ).arg( debugData.MaxCycles * 64/80 ).arg( debugData.AvgCycles * 64/80 ) );
	}

	if( bComputedChanged ) {
		//ui->Altimeter_display->setAltitude( computed.AltiEst / 1000.0f );
		//
		//ui->rollPowerVal->setValue( computed.Roll );
		//ui->rollPowerVal->setRightLabel( computed.Roll );
		//
		//ui->pitchPowerVal->setValue( computed.Pitch );
		//ui->pitchPowerVal->setRightLabel( computed.Pitch );
		//
		//ui->yawPowerVal->setValue( computed.Yaw );
		//ui->yawPowerVal->setRightLabel( computed.Yaw );
		//
		//if( ui->tabWidget->currentWidget() == ui->tpSensors )
		//{
		//	ui->lblAltPressure->setText( QString::number(computed.Alt / 1000.0f, 'f', 2) );
		//	ui->lblAltiEst->setText( QString::number(computed.AltiEst / 1000.0f, 'f', 2) );
		//	ui->lblGroundHeight->setText( QString::number(computed.GroundHeight) );
		//}
	}
}


void MainWindow::on_btnCompile_clicked()
{
	// Browse for the file to compile
	QString filepath = "F:/GitHub/Flight-Controller/FloatStreamCreator/functionstream.cpp";
	QString inputsPath = "F:/GitHub/Flight-Controller/FloatStreamCreator/inputoutputlist.h";

	QByteArray contents;
	QByteArray inputs;

	QFile f( filepath );
	if( f.open(QFile::ReadOnly | QFile::Text) )
	{
		contents = f.readAll();
		f.close();
	}

	QFile f2( inputsPath );
	if( f2.open(QFile::ReadOnly | QFile::Text) )
	{
		inputs = f2.readAll();
		f2.close();
	}

	if( contents.length() > 0 && inputs.length() > 0 )
	{
		// Compile it
		FunctionCompiler comp;
		QString outputPrefix = "F:/GitHub/Flight-Controller/Firmware-C/QuatIMU";
		comp.Compile( contents , inputs, outputPrefix );
	}
}

void MainWindow::UpdateMagSphere(void)
{
	pts[pointsUsed] = QVector3D( sensors.MagX, sensors.MagY, sensors.MagZ );
	if( pointsUsed < 4096 ) pointsUsed++;
	pointIndex = (pointIndex+1) & 4095;

	if( (pointIndex & 15) != 15) return;

	//
	// Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
	//
	//    P is a structure that has been computed with the data earlier.
	//    P.npoints is the number of elements; the length of X,Y,Z are identical.
	//    P's members are logically named.
	//
	//    X[n] is the x component of point n
	//    Y[n] is the y component of point n
	//    Z[n] is the z component of point n
	//
	//    A is the x coordiante of the sphere
	//    B is the y coordiante of the sphere
	//    C is the z coordiante of the sphere
	//    Rsq is the radius squared of the sphere.
	//
	// This method should converge; maybe 5-100 iterations or more.
	//
	// Credit where due, this code was taken from:
	// http://imaginaryz.blogspot.com/2011/04/least-squares-fit-sphere-to-3d-data.html

	float npoints = (float)pointsUsed;

	float Xsum =     0.0f, Ysum =     0.0, Zsum =     0.0;
	float Xsumsq =   0.0f, Ysumsq =   0.0, Zsumsq =   0.0;
	float Xsumcube = 0.0f, Ysumcube = 0.0, Zsumcube = 0.0;
	float XYsum =    0.0f, XZsum =    0.0, YZsum =    0.0;
	float X2Ysum =   0.0f, X2Zsum =   0.0, Y2Xsum =   0.0;
	float Y2Zsum =   0.0f, Z2Xsum =   0.0, Z2Ysum =   0.0;

	for( int i=0; i<pointsUsed; i++ )
	{
		Xsum     += pts[i].x();
		Ysum     += pts[i].y();
		Zsum     += pts[i].z();
		Xsumsq   += pts[i].x() * pts[i].x();
		Ysumsq   += pts[i].y() * pts[i].y();
		Zsumsq   += pts[i].z() * pts[i].z();
		Xsumcube += pts[i].x() * pts[i].x() * pts[i].x();
		Ysumcube += pts[i].y() * pts[i].y() * pts[i].y();
		Zsumcube += pts[i].z() * pts[i].z() * pts[i].z();
		XYsum    += pts[i].x() * pts[i].y();
		XZsum    += pts[i].x() * pts[i].z();
		YZsum    += pts[i].y() * pts[i].z();
		X2Ysum   += pts[i].x() * pts[i].x() * pts[i].y();
		X2Zsum   += pts[i].x() * pts[i].x() * pts[i].z();
		Y2Xsum   += pts[i].y() * pts[i].y() * pts[i].x();
		Y2Zsum   += pts[i].y() * pts[i].y() * pts[i].z();
		Z2Xsum   += pts[i].z() * pts[i].z() * pts[i].x();
		Z2Ysum   += pts[i].z() * pts[i].z() * pts[i].y();
	}


	float Xn = Xsum/npoints;        //sum( X[n] )
	float Xn2 = Xsumsq/npoints;    //sum( X[n]^2 )
	float Xn3 = Xsumcube/npoints;    //sum( X[n]^3 )
	float Yn = Ysum/npoints;        //sum( Y[n] )
	float Yn2 = Ysumsq/npoints;    //sum( Y[n]^2 )
	float Yn3 = Ysumcube/npoints;    //sum( Y[n]^3 )
	float Zn = Zsum/npoints;        //sum( Z[n] )
	float Zn2 = Zsumsq/npoints;    //sum( Z[n]^2 )
	float Zn3 = Zsumcube/npoints;    //sum( Z[n]^3 )

	float XY = XYsum/npoints;        //sum( X[n] * Y[n] )
	float XZ = XZsum/npoints;        //sum( X[n] * Z[n] )
	float YZ = YZsum/npoints;        //sum( Y[n] * Z[n] )
	float X2Y = X2Ysum/npoints;    //sum( X[n]^2 * Y[n] )
	float X2Z = X2Zsum/npoints;    //sum( X[n]^2 * Z[n] )
	float Y2X = Y2Xsum/npoints;    //sum( Y[n]^2 * X[n] )
	float Y2Z = Y2Zsum/npoints;    //sum( Y[n]^2 * Z[n] )
	float Z2X = Z2Xsum/npoints;    //sum( Z[n]^2 * X[n] )
	float Z2Y = Z2Ysum/npoints;    //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	float F0 = Xn2 + Yn2 + Zn2;
	float F1 = 0.5f*F0;
	float F2 = -8.0f*(Xn3 + Y2X + Z2X);
	float F3 = -8.0f*(X2Y + Yn3 + Z2Y);
	float F4 = -8.0f*(X2Z + Y2Z + Zn3);

	//Set initial conditions:
	float A = Xn;
	float B = Yn;
	float C = Zn;

	//First iteration computation:
	float A2 = A*A;
	float B2 = B*B;
	float C2 = C*C;
	float QS = A2 + B2 + C2;
	float QB = - 2.0f*(A*Xn + B*Yn + C*Zn);

	//Set initial conditions:
	float Rsq = F0 + QB + QS;

	//First iteration computation:
	float Q0 = 0.5*(QS - Rsq);
	float Q1 = F1 + Q0;
	float Q2 = 8.0f*( QS - Rsq + QB + F0 );
	float aA,aB,aC,nA,nB,nC,dA,dB,dC;

	//Iterate N times, ignore stop condition.
	int n = 0;
	int N = 100;
	float Nstop = 0.5f;

	while( n != N )
	{
		n++;

		//Compute denominator:
		aA = Q2 + 16.0f*(A2 - 2.0f*A*Xn + Xn2);
		aB = Q2 + 16.0f*(B2 - 2.0f*B*Yn + Yn2);
		aC = Q2 + 16.0f*(C2 - 2.0f*C*Zn + Zn2);
		aA = (aA == 0) ? 1.0f : aA;
		aB = (aB == 0) ? 1.0f : aB;
		aC = (aC == 0) ? 1.0f : aC;

		//Compute next iteration
		nA = A - ((F2 + 16.0f*( B*XY + C*XZ + Xn*(-A2 - Q0) + A*(Xn2 + Q1 - C*Zn - B*Yn) ) )/aA);
		nB = B - ((F3 + 16.0f*( A*XY + C*YZ + Yn*(-B2 - Q0) + B*(Yn2 + Q1 - A*Xn - C*Zn) ) )/aB);
		nC = C - ((F4 + 16.0f*( A*XZ + B*YZ + Zn*(-C2 - Q0) + C*(Zn2 + Q1 - A*Xn - B*Yn) ) )/aC);

		//Check for stop condition
		dA = (nA - A);
		dB = (nB - B);
		dC = (nC - C);
		if( (dA*dA + dB*dB + dC*dC) <= Nstop ){ break; }

		//Compute next iteration's values
		A = nA;
		B = nB;
		C = nC;
		A2 = A*A;
		B2 = B*B;
		C2 = C*C;
		QS = A2 + B2 + C2;
		QB = - 2.0f*(A*Xn + B*Yn + C*Zn);
		Rsq = F0 + QB + QS;
		Q0 = 0.5f*(QS - Rsq);
		Q1 = F1 + Q0;
		Q2 = 8.0f*( QS - Rsq + QB + F0 );
	}

	double R = sqrt(Rsq);
	ui->xy_mag->SetCircle( A, B, R );
	ui->yz_mag->SetCircle( B, C, R );
	ui->xz_mag->SetCircle( A, C, R );
}
