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

					//QMatrix3x3 m;
					//m = QuatToMatrix( q );

					//double roll = asin(  m(1, 0) ) * (180.0 / PI) * 100.0;
					//double pitch = asin( m(1, 2) ) * (180.0 / PI) * 100.0;
					//double yaw = -atan2( m(2, 0), m(2, 2) ) * (180.0 / PI) * 100.0;

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
		QString outputPrefix = "F:/GitHub/Flight-Controller/Firmware-C/QuatIMU_";
		comp.Compile( contents , inputs, outputPrefix );
	}
}
