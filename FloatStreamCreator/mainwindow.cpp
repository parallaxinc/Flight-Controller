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

	ui->vb_mx->setOrigin(ValueBar_Widget::Center);
	ui->vb_mx->setMinMax( -2000, 2000 );
	ui->vb_mx->setRightLabel( "MX" );
	ui->vb_mx->setBarColor( QColor::fromRgb(255,128,128) );

	ui->vb_my->setOrigin(ValueBar_Widget::Center);
	ui->vb_my->setMinMax( -2000, 2000 );
	ui->vb_my->setRightLabel( "MY" );
	ui->vb_my->setBarColor( QColor::fromRgb(128,255,128) );

	ui->vb_mz->setOrigin(ValueBar_Widget::Center);
	ui->vb_mz->setMinMax( -2000, 2000 );
	ui->vb_mz->setRightLabel( "MZ" );
	ui->vb_mz->setBarColor( QColor::fromRgb(128,128,255) );

	QVector3D v(0.0f, 1.0f, 0.0f);
	ui->Orientation_display->extraVects.append(v);
	ui->Orientation_display->extraVects.append(v);
	ui->Orientation_display->extraVects.append(v);

	ui->Orientation_display->extraVects.append(v);
	ui->Orientation_display->extraVects.append(v);
	ui->Orientation_display->extraVects.append(v);

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

					ui->Orientation_display->setQuat(cq);

					ui->vb_mx->setValue( sensors.MagX );
					ui->vb_my->setValue( sensors.MagY );
					ui->vb_mz->setValue( sensors.MagZ );

					{
						QPointF pt;
						pt.setX(sensors.MagX);
						pt.setY(sensors.MagY);
						ui->xy_mag->AddSample(pt, true);

						pt.setY(sensors.MagZ);
						ui->xz_mag->AddSample(pt, true);

						pt.setX(sensors.MagY);
						ui->yz_mag->AddSample(pt, true);

						QVector3D av(sensors.AccelX / 8192.0f, sensors.AccelZ / 8192.0f, sensors.AccelY / 8192.0f);
						QVector3D mv(sensors.MagX / 2048.0f, sensors.MagZ / 2048.0f, sensors.MagY / 2048.0f);

						//ui->Orientation_display->extraVects[3] = av;
						//ui->Orientation_display->extraVects[4] = mv;
					}

					UpdateMagSphere();


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

					// Row vects = world frame in local space
					// Col vects = local orient in world space

					ui->Orientation_display->extraVects[0] = QVector3D(m(0,0), m(1,0), m(2,0));
					ui->Orientation_display->extraVects[1] = QVector3D(m(0,1), m(1,1), m(2,1));
					ui->Orientation_display->extraVects[2] = QVector3D(m(0,2), m(1,2), m(2,2));


					float mx = sensors.MagX;
					float my = sensors.MagY;
					float mz = sensors.MagZ;

					float pitch = asinf( -m(1, 0) );
					float cosPitch = cosf(pitch);
					float sinPitch = sinf(pitch);

					float roll  = asinf( m(1, 2) / cosPitch );
					float cosRoll = cosf(roll);
					float sinRoll = sinf(roll);

					float mxSinPitch = mx * sinPitch;
					float mzCosPitch = mz * cosPitch;

					float xh =  mx * cosPitch + mz * sinPitch;
					float yh =  mxSinPitch * sinRoll + my * cosRoll - mzCosPitch * sinRoll;
					//float zh = -mxSinPitch * cosRoll + my * sinRoll + mzCosPitch * cosRoll;

					float magNorth = atan2(-xh, yh);
					ui->Heading_display->setHeading( magNorth * (180.0/PI) );

					// This is the "north" vector, in the local space of the flight controller
					//ui->Orientation_display->extraVects[3] = QVector3D( xh, 0.0f, yh ).normalized() * 0.7f;

					QVector3D hv;
					Quat_GetHeadingVect( hv );
					ui->Orientation_display->extraVects[3] = hv;


					// According to madgwick, mag vector can be rotated into local frame and planarized by:
					// EHxyz = Quat * magNorm * QuatConjugate (ie, S,-V)
					// horzMag = EHxyz( sqrt(x*x+y*y), 0, z )
					// However, Quat * vect is non-trivial, and is a lot of mults.  Might be faster with sins/etc.  Worth timing.


					// Do I need to rotate this back into quad frame to compute a diff vector to it, or do I just need "heading"?
					// I think I just need "heading"...


					bQuatChanged = true;
					}
					break;

				case 4:	// Compute values
					computed.ReadFrom( p );
					bComputedChanged = true;
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
		//ui->Orientation_display->setQuat(q);

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
	// mag offset 560, 746, -206 : Elev8
	//            228  985  -47  : alt board

	pts[pointsUsed] = QVector3D( sensors.MagX, sensors.MagY, sensors.MagZ );
	if( pointsUsed < 512 ) pointsUsed++;
	pointIndex = (pointIndex+1) & 511;

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

	float Xn  =   0.0f, Yn  =   0.0, Zn  =   0.0;
	float Xn2 =   0.0f, Yn2 =   0.0, Zn2 =   0.0;
	float Xn3 =   0.0f, Yn3 =   0.0, Zn3 =   0.0;
	float XY  =   0.0f, XZ  =   0.0, YZ  =   0.0;
	float X2Y =   0.0f, X2Z =   0.0, Y2X =   0.0;
	float Y2Z =   0.0f, Z2X =   0.0, Z2Y =   0.0;

	for( int i=0; i<pointsUsed; i++ )
	{
		Xn    += pts[i].x();
		Yn    += pts[i].y();
		Zn    += pts[i].z();
		Xn2   += pts[i].x() * pts[i].x();
		Yn2   += pts[i].y() * pts[i].y();
		Zn2   += pts[i].z() * pts[i].z();
		Xn3   += pts[i].x() * pts[i].x() * pts[i].x();
		Yn3   += pts[i].y() * pts[i].y() * pts[i].y();
		Zn3   += pts[i].z() * pts[i].z() * pts[i].z();
		XY    += pts[i].x() * pts[i].y();
		XZ    += pts[i].x() * pts[i].z();
		YZ    += pts[i].y() * pts[i].z();
		X2Y   += pts[i].x() * pts[i].x() * pts[i].y();
		X2Z   += pts[i].x() * pts[i].x() * pts[i].z();
		Y2X   += pts[i].y() * pts[i].y() * pts[i].x();
		Y2Z   += pts[i].y() * pts[i].y() * pts[i].z();
		Z2X   += pts[i].z() * pts[i].z() * pts[i].x();
		Z2Y   += pts[i].z() * pts[i].z() * pts[i].y();
	}


	Xn  /= npoints;		//sum( X[n] )
	Xn2 /= npoints;		//sum( X[n]^2 )
	Xn3 /= npoints;		//sum( X[n]^3 )
	Yn  /= npoints;		//sum( Y[n] )
	Yn2 /= npoints;		//sum( Y[n]^2 )
	Yn3 /= npoints;		//sum( Y[n]^3 )
	Zn  /= npoints;		//sum( Z[n] )
	Zn2 /= npoints;		//sum( Z[n]^2 )
	Zn3 /= npoints;		//sum( Z[n]^3 )

	XY  /= npoints;		//sum( X[n] * Y[n] )
	XZ  /= npoints;		//sum( X[n] * Z[n] )
	YZ  /= npoints;		//sum( Y[n] * Z[n] )
	X2Y /= npoints;		//sum( X[n]^2 * Y[n] )
	X2Z /= npoints;		//sum( X[n]^2 * Z[n] )
	Y2X /= npoints;		//sum( Y[n]^2 * X[n] )
	Y2Z /= npoints;		//sum( Y[n]^2 * Z[n] )
	Z2X /= npoints;		//sum( Z[n]^2 * X[n] )
	Z2Y /= npoints;		//sum( Z[n]^2 * Y[n] )

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
	float Nstop = 0.1f;

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
		if( (dA*dA + dB*dB + dC*dC) <= Nstop ){
			break;
		}

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

	ui->lblCenter->setText( QString( "Center %1 %2 %3" ).arg(A).arg(B).arg(C) );
}

void MainWindow::on_elevationRot_valueChanged(int value)
{
	ui->Orientation_display->setElevation( (float)value );
}

void MainWindow::on_headingRot_valueChanged(int value)
{
	ui->Orientation_display->setHeading( (float)value );
}
