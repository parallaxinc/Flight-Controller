#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QCoreApplication>
#include "aboutbox.h"
#include "quatutil.h"
#include <math.h>

static char beatString[] = "BEAT";


MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
    ui(new Ui::MainWindow)
{
	ThrottleCalibrationCycle = 0;
	CalibrateControlsStep = 0;
	CalibrateTimer = 0;
	RadioMode = 2;		// Mode 2 by default - check to see if there's a config file with a different setting
	currentMode = None;

    ui->setupUi(this);

	QFont smallFont = ui->btnReceiverReset->font();
	smallFont.setPointSize(10);

	// Do this here because Mac font defaults are not the same point size as PC
	ui->btnReceiverReset->setFont(smallFont);
	ui->btnReceiverCalibrate->setFont(smallFont);
	ui->label_2->setFont(smallFont);
	ui->cbReceiverType->setFont(smallFont);
	ui->cbR_Channel1->setFont(smallFont);
	ui->cbR_Channel2->setFont(smallFont);
	ui->cbR_Channel3->setFont(smallFont);
	ui->cbR_Channel4->setFont(smallFont);
	ui->cbR_Channel5->setFont(smallFont);
	ui->cbR_Channel6->setFont(smallFont);
	ui->cbR_Channel7->setFont(smallFont);
	ui->cbR_Channel8->setFont(smallFont);
	ui->btnUploadRadioChanges->setFont(smallFont);
	ui->btnDisableMotors->setFont(smallFont);
	ui->btnUploadSystemSetup->setFont(smallFont);


	ui->lblRadioCalibrateDocs->setVisible(false);	// Hide this until calibration mode
	ui->lblRadioCalibrateDocs->setStyleSheet("QLabel { background-color : orange; color : black; }");

	for( int i=0; i<4; i++ ) {
		accXCal[i] = accYCal[i] = accZCal[i] = 0.f;
	}

	m_sSettingsFile = QApplication::applicationDirPath() + "/GroundStation_settings.ini";
	loadSettings();

    connect( &comm, SIGNAL(connectionMade()), this, SLOT(on_connectionMade()));


    QString style = QString("QPushButton{\
                            background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 white, stop: 1 lightgrey);\
							border-style: solid; border-color: grey; border-width: 1px; border-radius: 4px;\
                            }\
                            QPushButton:hover{\
                            background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 white, stop: 1 lightblue);\
							border-style: solid; border-color: grey; border-width: 1px; border-radius: 4px;\
                            }\
                            QPushButton:pressed{\
                            background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 lightblue, stop: 1 white);\
							border-style: solid; border-color: grey; border-width: 1px; border-radius: 4px;\
                            }\
                        ");

	// For the motor disable button
	QString style2 = QString("QPushButton{\
						background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 white, stop: 1 lightgrey);\
						border-style: solid; border-color: grey; border-width: 1px; border-radius: 2px;\
						}\
						QPushButton:hover{\
						background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 white, stop: 1 lightblue);\
						border-style: solid; border-color: grey; border-width: 1px; border-radius: 2px;\
						}\
						QPushButton:pressed{\
						background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 white, stop: 1 pink);\
						border-style: solid; border-color: grey; border-width: 1px; border-radius: 2px;\
						}\
						QPushButton:checked{\
						background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ff8080, stop: 1 #ffd0d0);\
						border-style: solid; border-color: grey; border-width: 1px; border-radius: 2px;\
						}\
					");

	InternalChange = true;

    ui->motor_FL_val->setFromLeft(false);
    ui->motor_BL_val->setFromLeft(false);

    ui->motor_FL_val->setMinMax( 8000, 16000 );
    ui->motor_FL_val->setLeftLabel( "FL" );
    ui->motor_FL_val->setBarColor( QColor::fromRgb(255,160,128) );

    ui->motor_FR_val->setMinMax( 8000, 16000 );
    ui->motor_FR_val->setRightLabel( "FR" );
    ui->motor_FR_val->setBarColor( QColor::fromRgb(255,160,128) );

    ui->motor_BR_val->setMinMax( 8000, 16000 );
    ui->motor_BR_val->setRightLabel( "BR" );
    ui->motor_BR_val->setBarColor( QColor::fromRgb(255,160,128) );

    ui->motor_BL_val->setMinMax( 8000, 16000 );
    ui->motor_BL_val->setLeftLabel( "BL" );
    ui->motor_BL_val->setBarColor( QColor::fromRgb(255,160,128) );

    ui->btnMotorTest_FL->setStyleSheet( style );
    ui->btnMotorTest_FR->setStyleSheet( style );
    ui->btnMotorTest_BL->setStyleSheet( style );
    ui->btnMotorTest_BR->setStyleSheet( style );
    ui->btnBeeper->setStyleSheet( style );
    ui->btnLED->setStyleSheet( style );
    ui->btnThrottleCalibrate->setStyleSheet( style );

	ui->btnDisableMotors->setStyleSheet(style2);

    ui->rollPowerVal->setLeftLabel( "Roll Power" );
    ui->pitchPowerVal->setLeftLabel( "Pitch Power" );
    ui->yawPowerVal->setLeftLabel( "Yaw Power" );

    ui->batteryVal->setLeftLabel( "Battery Voltage" );
    ui->batteryVal->setMinMax( 900, 1260 );

    ui->channel1Val->setLeftLabel("Throtle");
    ui->channel2Val->setLeftLabel("Aileron");
    ui->channel3Val->setLeftLabel("Elevator");
    ui->channel4Val->setLeftLabel("Rudder");
    ui->channel5Val->setLeftLabel("Gear");
    ui->channel6Val->setLeftLabel("Aux1");
    ui->channel7Val->setLeftLabel("Aux2");
    ui->channel8Val->setLeftLabel("Aux3");
    ui->vbR_Channel1->setLeftLabel("Throtle");
    ui->vbR_Channel2->setLeftLabel("Aileron");
    ui->vbR_Channel3->setLeftLabel("Elevator");
    ui->vbR_Channel4->setLeftLabel("Rudder");
    ui->vbR_Channel5->setLeftLabel("Gear");
    ui->vbR_Channel6->setLeftLabel("Aux1");
    ui->vbR_Channel7->setLeftLabel("Aux2");
    ui->vbR_Channel8->setLeftLabel("Aux3");

	FillChannelComboBox( ui->cbR_Channel1, RadioMode == 2 ? 1 : 3 );
    FillChannelComboBox( ui->cbR_Channel2, 2 );
	FillChannelComboBox( ui->cbR_Channel3, RadioMode == 2 ? 3 : 1 );
    FillChannelComboBox( ui->cbR_Channel4, 4 );
    FillChannelComboBox( ui->cbR_Channel5, 5 );
    FillChannelComboBox( ui->cbR_Channel6, 6 );
    FillChannelComboBox( ui->cbR_Channel7, 7 );
    FillChannelComboBox( ui->cbR_Channel8, 8 );

    ui->rollPowerVal->setMinMax( -5000, 5000 );
    ui->rollPowerVal->setBarColor( QColor::fromRgb(128,255,255) );
    ui->pitchPowerVal->setMinMax( -5000, 5000 );
    ui->pitchPowerVal->setBarColor( QColor::fromRgb(128,255,255) );
    ui->yawPowerVal->setMinMax( -5000, 5000 );
    ui->yawPowerVal->setBarColor( QColor::fromRgb(128,255,255) );


    ui->cbReceiverType->addItem(QString("PWM"));
    ui->cbReceiverType->addItem(QString("S-Bus"));
    ui->cbReceiverType->addItem(QString("PPM"));

	ui->cbArmingDelay->addItem(QString("1.00 sec"));
	ui->cbArmingDelay->addItem(QString("0.50 sec"));
	ui->cbArmingDelay->addItem(QString("0.25 sec"));
	ui->cbArmingDelay->addItem(QString("Off"));

	ui->cbDisarmDelay->addItem(QString("1.00 sec"));
	ui->cbDisarmDelay->addItem(QString("0.50 sec"));
	ui->cbDisarmDelay->addItem(QString("0.25 sec"));
	ui->cbDisarmDelay->addItem(QString("Off"));

	ui->vbVoltage2->setLeftLabel("Battery Voltage");
	ui->vbVoltage2->setMinMax( 900, 1260 );

	ui->gCalibTemp->setRange(8192);
	ui->gCalibX->setRange(8192);
	ui->gCalibY->setRange(8192);
	ui->gCalibZ->setRange(8192);

	ui->gAccelXCal->setRange(32768);
	ui->gAccelYCal->setRange(32768);
	ui->gAccelZCal->setRange(32768);

	SetRadioMode(RadioMode);

	labelStatus = new QLabel(this);
	labelGSVersion = new QLabel(this);
	labelFWVersion = new QLabel(this);

    // set text for the label
	labelStatus->setText("Connecting...");
	labelStatus->setContentsMargins( 5, 1, 5, 1 );
	labelGSVersion->setText("GroundStation Version 1.0.3");
	labelFWVersion->setText( "Firmware Version -.--");

	// add the controls to the status bar
	ui->statusBar->addPermanentWidget(labelStatus, 1);
	ui->statusBar->addPermanentWidget(labelGSVersion, 1);
	ui->statusBar->addPermanentWidget(labelFWVersion, 1);

	ui->statusBar->setStyleSheet( "QStatusBar::item { border: 0px solid black }; ");

	labelStatus->setFrameStyle(QFrame::NoFrame);
	labelGSVersion->setFrameStyle(QFrame::NoFrame);
	labelFWVersion->setFrameStyle(QFrame::NoFrame);

	const char* graphNames[] = {"GyroX", "GyroY", "GyroZ", "AccelX", "AccelY", "AccelZ", "MagX", "MagY", "MagZ", "GyroTemp",
							   "AltiRaw", "AltiEst", "LaserHeight",
							   "Pitch", "Roll", "Yaw", "Voltage"};
	const QColor graphColors[] = { Qt::red, Qt::green, Qt::blue,
									QColor(255,160,160), QColor(160, 255, 160), QColor(160,160,255),
									QColor(255, 96, 96), QColor( 96, 255,  96), QColor( 96, 96,255), QColor(192,64,64),
								   Qt::gray, Qt::black, QColor(255,128,0),
								   QColor(255,0,255), QColor(0,255,255), QColor(255,255,128), QColor(255,0,255)
								 };


	SampleIndex = 0;
	sg = ui->sensorGraph;
	sg->legend->setVisible(true);
	sg->setAutoAddPlottableToLegend(true);
	sg->xAxis->setRange(0, 2000);
	sg->yAxis->setRange(-2048, 2048);
	for( int i=0; i<17; i++ )
	{
		graphs[i] = sg->addGraph();
		graphs[i]->setName(graphNames[i]);
		graphs[i]->setPen( QPen(graphColors[i]) );
	}

	sg->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

	InternalChange = false;

	this->startTimer(25, Qt::PreciseTimer);		// 40 updates / sec

	comm.StartConnection();

    Heartbeat = 0;
}


MainWindow::~MainWindow()
{
    comm.StopConnection();
    delete ui;
}

void MainWindow::loadSettings(void)
{
	QSettings settings(m_sSettingsFile, QSettings::IniFormat);

	QVariant radioVar( RadioMode );
	QVariant rMode = settings.value("RadioMode", radioVar );

	RadioMode = rMode.toInt();
}

void MainWindow::saveSettings(void)
{
	QSettings settings(m_sSettingsFile, QSettings::IniFormat);
	settings.setValue( "RadioMode", RadioMode );
}


void MainWindow::on_actionReset_Flight_Controller_triggered()
{
	comm.Reset();
}

void MainWindow::on_actionRadio_Mode_1_triggered() {
	SetRadioMode(1);
}

void MainWindow::on_actionRadio_Mode_2_triggered() {
	SetRadioMode(2);
}

void MainWindow::on_actionRestore_Factory_Defaults_triggered()
{
	SendCommand( "WIPE" );	// Reset prefs
	SendCommand( "QPRF" );	// Query prefs (forces to be applied to UI)
}



void MainWindow::FillChannelComboBox(QComboBox *cb, int defaultIndex )
{
	cb->clear();
    for( int i=0; i<8; i++)
    {
        QString str = QString("Ch %1").arg(i+1);
        if( (i+1) == defaultIndex ) {
            str.append('*');
        }
        QVariant var(i);
        cb->addItem( str, var );
    }
    cb->setCurrentIndex( defaultIndex-1 );
}

void MainWindow::on_connectionMade()
{
	SendCommand( "QPRF" );
}

void MainWindow::timerEvent(QTimerEvent * e)
{
	(void)e;	// prevent unused parameter warning
	UpdateStatus();

	Heartbeat++;
	if( Heartbeat >= 20 && ThrottleCalibrationCycle == 0 )	// don't send heartbeat during throttle calibration
	{
		Heartbeat = 0;
		SendCommand( beatString );	// Send the connection heartbeat
	}

	ProcessPackets();
	CheckCalibrateControls();
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

void MainWindow::SetRadioMode(int mode)
{
	if( mode != 1 && mode != 2 ) return;

	RadioMode = mode;
	ui->actionRadio_Mode_1->setChecked( mode == 1 );
	ui->actionRadio_Mode_2->setChecked( mode == 2 );

	bool bCacheInternal = InternalChange;
	InternalChange = true;
	FillChannelComboBox( ui->cbR_Channel1, RadioMode == 2 ? 1 : 3 );
	FillChannelComboBox( ui->cbR_Channel3, RadioMode == 2 ? 3 : 1 );

	if(RadioMode == 1)
	{
		ui->channel1Val->setLeftLabel("Elevator");
		ui->channel3Val->setLeftLabel("Throttle");
		ui->vbR_Channel1->setLeftLabel("Elevator");
		ui->vbR_Channel3->setLeftLabel("Throttle");

		ui->cbR_Channel1->setCurrentIndex( prefs.ElevChannel );
		ui->cbR_Channel3->setCurrentIndex( prefs.ThroChannel );
	}
	else if(RadioMode == 2)
	{
		ui->channel1Val->setLeftLabel("Throttle");
		ui->channel3Val->setLeftLabel("Elevator");
		ui->vbR_Channel1->setLeftLabel("Throttle");
		ui->vbR_Channel3->setLeftLabel("Elevator");

		ui->cbR_Channel1->setCurrentIndex( prefs.ThroChannel );
		ui->cbR_Channel3->setCurrentIndex( prefs.ElevChannel );
	}
	InternalChange = bCacheInternal;

	if( InternalChange ) return;

	// Save radio mode to prefs file
	saveSettings();
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
    bool bPrefsChanged = false;

    packet * p;
    do {
        p = comm.GetPacket();
        if(p != 0)
        {
            switch( p->mode )
            {
                case 1:	// Radio data
                    radio.ReadFrom( p );

					graphs[16]->addData( SampleIndex, (float)radio.BatteryVolts );
                    bRadioChanged = true;
                    break;

                case 2:	// Sensor values
                    sensors.ReadFrom( p );

					graphs[0]->addData( SampleIndex, sensors.GyroX );
					graphs[1]->addData( SampleIndex, sensors.GyroY);
					graphs[2]->addData( SampleIndex, sensors.GyroZ);
					graphs[3]->addData( SampleIndex, sensors.AccelX);
					graphs[4]->addData( SampleIndex, sensors.AccelY);
					graphs[5]->addData( SampleIndex, sensors.AccelZ);
					graphs[6]->addData( SampleIndex, sensors.MagX);
					graphs[7]->addData( SampleIndex, sensors.MagY);
					graphs[8]->addData( SampleIndex, sensors.MagZ);
					graphs[9]->addData( SampleIndex, sensors.Temp );

					bSensorsChanged = true;
                    break;

                case 3:	// Quaternion
                    {
                    q.setX(      p->GetFloat() );
                    q.setY(      p->GetFloat() );
                    q.setZ(      p->GetFloat() );
                    q.setScalar( p->GetFloat() );

					QMatrix3x3 m;
					m = QuatToMatrix( q );

					double roll = asin(  m(1, 0) ) * (180.0 / PI) * 100.0;
					double pitch = asin( m(1, 2) ) * (180.0 / PI) * 100.0;
					double yaw = -atan2( m(2, 0), m(2, 2) ) * (180.0 / PI) * 100.0;

					graphs[13]->addData( SampleIndex, (float)pitch );
					graphs[14]->addData( SampleIndex, (float)roll );
					graphs[15]->addData( SampleIndex, (float)yaw );

                    bQuatChanged = true;
                    }
                    break;

                case 4:	// Compute values
                    computed.ReadFrom( p );
                    bComputedChanged = true;

					graphs[10]->addData( SampleIndex, (float)computed.Alt );
					graphs[11]->addData( SampleIndex, (float)computed.AltiEst );
					graphs[12]->addData( SampleIndex, (float)computed.AltTemp );
					break;

                case 5:	// Motor values
                    motors.ReadFrom( p );
                    bMotorsChanged = true;
                    break;

                case 6:	// Control quaternion
                    cq.setX      ( p->GetFloat() );
                    cq.setY      ( p->GetFloat() );
                    cq.setZ      ( p->GetFloat() );
                    cq.setScalar ( p->GetFloat() );
                    bTargetQuatChanged = true;

					// this is actually the last packet sent by the quad, so use this to advance the sample index
					SampleIndex++;
					if( SampleIndex > 6000 ) {
						SampleIndex = 0;
					}
					break;


                case 7:	// Debug data
                    debugData.ReadFrom( p );
                    bDebugChanged = true;
                    break;

                case 0x18:	// Settings
                    {
                        PREFS tempPrefs;
                        memset( &tempPrefs, 0, sizeof(tempPrefs) );

                        quint32 toCopy = p->len;
                        if( sizeof(tempPrefs) < toCopy )
                            toCopy = sizeof(tempPrefs);

                        memcpy( &tempPrefs, p->data, toCopy );

                        if( Prefs_CalculateChecksum( tempPrefs ) == tempPrefs.Checksum ) {
                            //PrefsReceived = true;	// Global indicator of valid prefs
                            bPrefsChanged = true;	// local indicator, just to set up the UI
                            prefs = tempPrefs;
                        }
                        else {
                            SendCommand( "QPRF" );	// reqeust them again because the checksum failed
                        }
                    }
                    break;
            }
            delete p;
        }
    } while(p != 0);

    if( bRadioChanged )
    {
		QString volts = QString::number((float)radio.BatteryVolts / 100.f, 'f', 2);

		ui->batteryVal->setValue( radio.BatteryVolts );
		ui->batteryVal->setRightLabel( volts );

		ui->vbVoltage2->setValue( radio.BatteryVolts );
		ui->vbVoltage2->setRightLabel( volts );

		if( RadioMode == 2 )
		{
			ui->radioLeft->setValues( radio.Rudd, radio.Thro );
			ui->radioRight->setValues( radio.Aile, radio.Elev );
			ui->radioLeft_R->setValues( radio.Rudd, radio.Thro );
			ui->radioRight_R->setValues( radio.Aile, radio.Elev );

			ui->channel1Val->setValue( radio.Thro );
			ui->channel1Val->setRightLabel( radio.Thro );
			ui->vbR_Channel1->setValue( radio.Thro );
			ui->vbR_Channel1->setRightLabel( radio.Thro );

			ui->channel3Val->setValue( radio.Elev );
			ui->channel3Val->setRightLabel( radio.Elev );
			ui->vbR_Channel3->setValue( radio.Elev );
			ui->vbR_Channel3->setRightLabel( radio.Elev );
		}
		else
		{
			ui->radioLeft->setValues( radio.Rudd, radio.Elev );
			ui->radioRight->setValues( radio.Aile, radio.Thro  );
			ui->radioLeft_R->setValues( radio.Rudd, radio.Elev );
			ui->radioRight_R->setValues( radio.Aile, radio.Thro );

			ui->channel1Val->setValue( radio.Elev );
			ui->channel1Val->setRightLabel( radio.Elev );
			ui->vbR_Channel1->setValue( radio.Elev );
			ui->vbR_Channel1->setRightLabel( radio.Elev );

			ui->channel3Val->setValue( radio.Thro );
			ui->channel3Val->setRightLabel( radio.Thro );
			ui->vbR_Channel3->setValue( radio.Thro );
			ui->vbR_Channel3->setRightLabel( radio.Thro );
		}

        ui->channel2Val->setValue( radio.Aile );
        ui->channel2Val->setRightLabel( radio.Aile );
        ui->vbR_Channel2->setValue( radio.Aile );
        ui->vbR_Channel2->setRightLabel( radio.Aile );


        ui->channel4Val->setValue( radio.Rudd );
        ui->channel4Val->setRightLabel( radio.Rudd );
        ui->vbR_Channel4->setValue( radio.Rudd );
        ui->vbR_Channel4->setRightLabel( radio.Rudd );

        ui->channel5Val->setValue( radio.Gear );
        ui->channel5Val->setRightLabel( radio.Gear );
        ui->vbR_Channel5->setValue( radio.Gear );
        ui->vbR_Channel5->setRightLabel( radio.Gear );

        ui->channel6Val->setValue( radio.Aux1 );
        ui->channel6Val->setRightLabel( radio.Aux1 );
        ui->vbR_Channel6->setValue( radio.Aux1 );
        ui->vbR_Channel6->setRightLabel( radio.Aux1 );

        ui->channel7Val->setValue( radio.Aux2 );
        ui->channel7Val->setRightLabel( radio.Aux2 );
        ui->vbR_Channel7->setValue( radio.Aux2 );
        ui->vbR_Channel7->setRightLabel( radio.Aux2 );

        ui->channel8Val->setValue( radio.Aux3 );
        ui->channel8Val->setRightLabel( radio.Aux3 );
        ui->vbR_Channel8->setValue( radio.Aux3 );
        ui->vbR_Channel8->setRightLabel( radio.Aux3 );
    }

    if( bMotorsChanged )
    {
        ui->motor_FL_val->setValue( motors.FL );
		ui->motor_FL_val->setRightLabel( motors.FL/8 );

        ui->motor_FR_val->setValue( motors.FR );
		ui->motor_FR_val->setLeftLabel( motors.FR/8 );

        ui->motor_BR_val->setValue( motors.BR );
		ui->motor_BR_val->setLeftLabel( motors.BR/8 );

        ui->motor_BL_val->setValue( motors.BL );
		ui->motor_BL_val->setRightLabel( motors.BL/8 );
    }

    if(bQuatChanged) {
        //if(tcTabs.SelectedTab == tpStatus)
        {
			ui->Orientation_display->setQuat(q);
			ui->Orientation_Accel->setQuat(q);

			QMatrix3x3 m;
			m = QuatToMatrix( q );

			float roll =   asin( m(1,0) ) * (180.0f/PI);
			float pitch =  asin( m(1,2) ) * (180.0f/PI);
			float yaw =  -atan2( m(2,0), m(2,2) ) * (180.0f/PI);

			ui->Horizon_display->setAngles( roll, pitch );
			ui->Heading_display->setHeading(yaw);
        }
	}

	if( bSensorsChanged )
	{
		if( ui->tabWidget->currentWidget() == ui->tpGyroCalib )
		{
			static int calibSampleIndex = 0;

			LFSample sample;
			sample.t = sensors.Temp;
			sample.x = sensors.GyroX;
			sample.y = sensors.GyroY;
			sample.z = sensors.GyroZ;

			ui->gCalibTemp->setValue( sensors.Temp );
			ui->gCalibX->setValue( sensors.GyroX );
			ui->gCalibY->setValue( sensors.GyroY );
			ui->gCalibZ->setValue( sensors.GyroZ );

			calibSampleIndex++;

			bool bDoRedraw = (calibSampleIndex & 3) == 3;
			ui->lfGyroGraph->AddSample(sample , bDoRedraw );

			if( bDoRedraw == false )
			{
				int scaleX = 0, scaleY = 0, scaleZ = 0;

				if(fabs(ui->lfGyroGraph->dSlope.x ) > 0.00001)
					scaleX = (int)round( 1.0 / ui->lfGyroGraph->dSlope.x );
				if( scaleX >= 1024 ) scaleX = 0;

				if(fabs( ui->lfGyroGraph->dSlope.y ) > 0.00001)
					scaleY = (int)round( 1.0 / ui->lfGyroGraph->dSlope.y );
				if( scaleY >= 1024 ) scaleY = 0;

				if(fabs( ui->lfGyroGraph->dSlope.z ) > 0.00001)
					scaleZ = (int)round( 1.0 / ui->lfGyroGraph->dSlope.z );
				if( scaleZ >= 1024 ) scaleZ = 0;

				int offsetX = (int)round( ui->lfGyroGraph->dIntercept.x );
				int offsetY = (int)round( ui->lfGyroGraph->dIntercept.y );
				int offsetZ = (int)round( ui->lfGyroGraph->dIntercept.z );

				QString str;
				str = QString::number( scaleX );
				ui->lblGxScale->setText(str);

				str = QString::number( scaleY );
				ui->lblGyScale->setText(str);

				str = QString::number( scaleZ );
				ui->lblGzScale->setText(str);

				str = QString::number( offsetX );
				ui->lblGxOffset->setText(str);

				str = QString::number( offsetY );
				ui->lblGyOffset->setText(str);

				str = QString::number( offsetZ );
				ui->lblGzOffset->setText(str);
			}
		}
		else if( ui->tabWidget->currentWidget() == ui->tpAccelCalib )
		{
			ui->gAccelXCal->setValue( sensors.AccelX );
			ui->gAccelYCal->setValue( sensors.AccelY );
			ui->gAccelZCal->setValue( sensors.AccelZ );
		}
		else if( ui->tabWidget->currentWidget() == ui->tpSensors )
		{
			sg->replot();
		}
	}

	if(bTargetQuatChanged)
	{
		ui->Orientation_display->setQuat2(cq);
	}

    if( bDebugChanged )
    {
		labelFWVersion->setText( QString( "Firmware Version %1.%2" ).arg(debugData.Version >> 8).arg( debugData.Version & 255, 2, 10, QChar('0')) );

		ui->lblCycles->setText( QString(
			"CPU time (uS): %1 (min), %2 (max), %3 (avg)" ).arg( debugData.MinCycles * 64/80 ).arg( debugData.MaxCycles * 64/80 ).arg( debugData.AvgCycles * 64/80 ) );
    }

    if( bComputedChanged ) {
        ui->Altimeter_display->setAltitude( computed.AltiEst / 1000.0f );

        ui->rollPowerVal->setValue( computed.Roll );
        ui->rollPowerVal->setRightLabel( computed.Roll );

        ui->pitchPowerVal->setValue( computed.Pitch );
        ui->pitchPowerVal->setRightLabel( computed.Pitch );

        ui->yawPowerVal->setValue( computed.Yaw );
        ui->yawPowerVal->setRightLabel( computed.Yaw );
    }

	if( bPrefsChanged ) {
        ConfigureUIFromPreferences();
    }
}

void MainWindow::TestMotor(int index)
{
    if(comm.Connected())
    {
		CancelThrottleCalibration();	// just in case

        index++;
        SendCommand( QString( "M%1t%2").arg( index ).arg( index ) );	// Becomes M1t1, M2t2, etc..
    }
}

void MainWindow::CancelThrottleCalibration(void)
{
	if(ThrottleCalibrationCycle == 0) return;

	quint8 txBuffer[1] = {0};	// Escape from throttle setting
	comm.Send( txBuffer, 1 );
	QString str;
	ui->lblCalibrateDocs->setText(str);
	ui->lblCalibrateDocs->setVisible(false);
	ui->gbControlSetup->setVisible(true);
	ThrottleCalibrationCycle = 0;
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
	(void)index;	// prevent compilation warning from unused variable

	Mode newMode;
	CancelThrottleCalibration();	// just in case

	if(ui->tabWidget->currentWidget() == ui->tpGyroCalib) {
		newMode = GyroCalibration;
	}
	if(ui->tabWidget->currentWidget() == ui->tpAccelCalib) {
		newMode = AccelCalibration;
	}
	else {
		newMode = SensorTest;
	}

	if(newMode == currentMode) {
		return;
	}

	// If we're switching OUT of one of the calibration modes
	// tell the flight controller to revert back to its drift-compensated settings
	if(currentMode == GyroCalibration) {
		SendCommand( "RGyr" );	// revert previous gyro calibration values
	}
	else if(currentMode == AccelCalibration) {
		SendCommand( "RAcl");	// revert previous accel calibration values
	}

	if(newMode == GyroCalibration) {
		SendCommand( "ZrGr" );	// zero gyro calibration settings
	}
	else if(newMode == AccelCalibration) {
		SendCommand( "ZeAc" );	// zero accelerometer calibration settings
	}

	currentMode = newMode;
}


void MainWindow::on_btnSafetyCheck_clicked()
{
	if( ui->btnSafetyCheck->isChecked() == false ) {
		CancelThrottleCalibration();	// If they press it DURING throttle calibration, it's an escape button
	}
}


void MainWindow::on_btnThrottleCalibrate_clicked()
{
QString str;
quint8 txBuffer[1];

	switch(ThrottleCalibrationCycle)
	{
	case 0:
		if( ui->btnSafetyCheck->isChecked() == false )
		{
			str = "You must verify that you have removed your propellers by pressing the button to the right.";
			AbortThrottleCalibrationWithMessage( str , 3 );
			return;
		}
		TestMotor( 6 );
		str = "Throttle calibration has started.  Be sure your flight battery is UNPLUGGED, then press the Throttle Calibration button again.  (Click any other button to cancel)";
		ui->lblCalibrateDocs->setVisible(true);
		ui->lblCalibrateDocs->setText(str);
		ThrottleCalibrationCycle = 1;

		// TODO - Should probably disable all other buttons, and make an abort button visible
		break;

	case 1:
		if( radio.BatteryVolts > 0 )
		{
			str = "You must disconnect your flight battery to calibrate your ESC throttle range.  Failure to do so is a serious safety hazard.";
			AbortThrottleCalibrationWithMessage( str , 5 );
			return;
		}

		txBuffer[0] = (quint8)0xFF;
		comm.Send( txBuffer, 1 );
		str = "Plug in your flight battery and wait for the ESCs to stop beeping (about 5 seconds), then press the Throttle Calibration button again";
		ui->lblCalibrateDocs->setText(str);
		ThrottleCalibrationCycle = 2;
		break;

	case 2:
		txBuffer[0] = 0;		// Finish
		comm.Send( txBuffer, 1 );
		str = "Once the ESCs stop beeping (about 5 seconds), calibration is complete and your may remove the flight battery";
		ui->lblCalibrateDocs->setText(str);

		qApp->processEvents();	// force the label we just updated to repaint

		QThread::sleep( 5 );
		str = "";
		ui->lblCalibrateDocs->setText(str);
		ThrottleCalibrationCycle = 0;

		// TODO: Re-enable all other buttons, hide the abort button
		break;
	}
}

void MainWindow::AbortThrottleCalibrationWithMessage( QString & msg , int delay )
{
quint8 txBuffer[1];

	txBuffer[0] = (quint8)0x0;
	comm.Send( txBuffer, 1 );

	QString backup = ui->lblCalibrateDocs->styleSheet();
	ui->lblCalibrateDocs->setVisible(true);
	ui->lblCalibrateDocs->setText(msg);
	ui->lblCalibrateDocs->setStyleSheet("QLabel { background-color : orange; color : black; }");

	qApp->processEvents();	// force the label we just updated to repaint

	QThread::sleep( delay );
	ui->lblCalibrateDocs->setStyleSheet( backup );
	ui->lblCalibrateDocs->setText("");
	CancelThrottleCalibration();
}


void MainWindow::AttemptSetValue( QSlider * slider , int value )
{
	if( value < slider->minimum() || value > slider->maximum() ) return;
	slider->setValue( value );
}

void MainWindow::AttemptSetValue( QSpinBox * spin , int value )
{
	if( value < spin->minimum() || value > spin->maximum() ) return;
	spin->setValue( value );
}

void MainWindow::AttemptSetValue( QDoubleSpinBox * spin, double value )
{
	if( value < spin->minimum() || value > spin->maximum() ) return;
	spin->setValue( value );
}

void MainWindow::AttemptSetValue( QScrollBar* scroll, int value )
{
	if( value < scroll->minimum() || value > scroll->maximum() ) return;
	scroll->setValue( value );
}


void MainWindow::ConfigureUIFromPreferences(void)
{
	InternalChange = true;

	// Radio Setup
	//----------------------------------------------------------------------------
	ui->cbReceiverType->setCurrentIndex( prefs.ReceiverType );

	ui->revR_Channel1->setChecked( prefs.ThroScale < 0 );
	ui->revR_Channel2->setChecked( prefs.AileScale < 0 );
	ui->revR_Channel3->setChecked( prefs.ElevScale < 0 );
	ui->revR_Channel4->setChecked( prefs.RuddScale < 0 );
	ui->revR_Channel5->setChecked( prefs.GearScale < 0 );
	ui->revR_Channel7->setChecked( prefs.Aux1Scale < 0 );
	ui->revR_Channel6->setChecked( prefs.Aux2Scale < 0 );
	ui->revR_Channel8->setChecked( prefs.Aux3Scale < 0 );

	if( RadioMode == 2 ) {
		ui->cbR_Channel1->setCurrentIndex( prefs.ThroChannel );
		ui->cbR_Channel3->setCurrentIndex( prefs.ElevChannel );
	}
	else {
		ui->cbR_Channel1->setCurrentIndex( prefs.ElevChannel );
		ui->cbR_Channel3->setCurrentIndex( prefs.ThroChannel );
	}
	ui->cbR_Channel2->setCurrentIndex( prefs.AileChannel );
	ui->cbR_Channel4->setCurrentIndex( prefs.RuddChannel );
	ui->cbR_Channel5->setCurrentIndex( prefs.GearChannel );
	ui->cbR_Channel6->setCurrentIndex( prefs.Aux1Channel );
	ui->cbR_Channel7->setCurrentIndex( prefs.Aux2Channel );
	ui->cbR_Channel8->setCurrentIndex( prefs.Aux3Channel );


	float Source = prefs.AutoLevelRollPitch;
	Source = (Source * 2.0f) / (PI / 180.0f) * 1024.0f;
	AttemptSetValue( ui->hsAutoRollPitchSpeed, (int)(Source + 0.5f) );

	Source = prefs.AutoLevelYawRate;
	Source = (Source * 2.0f) / (PI / 180.0f) * 1024.0f * 250.0f;
	AttemptSetValue( ui->hsAutoYawSpeed, (int)(Source + 0.5f) / 10 );

	Source = prefs.ManualRollPitchRate;
	Source = (Source * 2.0f) / (PI / 180.0f) * 1024.0f * 250.0f;
	AttemptSetValue( ui->hsManualRollPitchSpeed, (int)(Source * 20.0f + 0.5f) );

	Source = prefs.ManualYawRate;
	Source = (Source * 2.0f) / (PI / 180.0f) * 1024.0f * 250.0f;
	AttemptSetValue( ui->hsManualYawSpeed, (int)(Source * 20.0f + 0.5f) );


	ui->hsAccelCorrectionFilter->setValue( prefs.AccelCorrectionFilter );
	ui->hsThrustCorrection->setValue( prefs.ThrustCorrectionScale );


	// Flight Control Setup
	//----------------------------------------------------------------------------

	ui->cbPitchRollLocked->setChecked( prefs.PitchRollLocked != 0 );
	AttemptSetValue( ui->hsPitchGain, prefs.PitchGain + 1 );
	AttemptSetValue( ui->hsRollGain, prefs.RollGain + 1 );
	AttemptSetValue( ui->hsYawGain, prefs.YawGain + 1 );
	AttemptSetValue( ui->hsAscentGain, prefs.AscentGain + 1 );
	AttemptSetValue( ui->hsAltiGain, prefs.AltiGain + 1 );
	//cbEnableAdvanced.Checked = (prefs.UseAdvancedPID != 0);



	// System Setup
	//----------------------------------------------------------------------------

	ui->cbUseBatteryMonitor->setChecked(prefs.UseBattMon == 1);

	AttemptSetValue( ui->sbLowThrottle, prefs.MinThrottle / 8 );
	AttemptSetValue( ui->sbArmedLowThrottle, prefs.MinThrottleArmed / 8 );
	AttemptSetValue( ui->sbHighThrottle, prefs.MaxThrottle / 8 );
	AttemptSetValue( ui->sbTestThrottle, prefs.ThrottleTest / 8 );
	ui->btnDisableMotors->setChecked(prefs.DisableMotors == 1);


	AttemptSetValue( ui->sbLowVoltageAlarmThreshold, (double)prefs.LowVoltageAlarmThreshold / 100.0 );
	AttemptSetValue( ui->sbVoltageOffset, (double)prefs.VoltageOffset / 100.0 );

	ui->cbLowVoltageAlarm->setChecked(prefs.LowVoltageAlarm != 0);

	switch( prefs.ArmDelay )
	{
		default:
		case 250: ui->cbArmingDelay->setCurrentIndex(0); break;	// 1 sec
		case 125: ui->cbArmingDelay->setCurrentIndex(1); break;	// 1/2 sec
		case 62:  ui->cbArmingDelay->setCurrentIndex(2); break;	// 1/4 sec
		case 0:   ui->cbArmingDelay->setCurrentIndex(3); break;	// none
	}

	switch(prefs.DisarmDelay)
	{
		default:
		case 250: ui->cbDisarmDelay->setCurrentIndex(0); break;	// 1 sec
		case 125: ui->cbDisarmDelay->setCurrentIndex(1); break;	// 1/2 sec
		case 62:  ui->cbDisarmDelay->setCurrentIndex(2); break;	// 1/4 sec
		case 0:   ui->cbDisarmDelay->setCurrentIndex(3); break;	// none
	}



	// Gyro Calibration
	//----------------------------------------------------------------------------


	// Accel Calibration
	//----------------------------------------------------------------------------

	if( prefs.PitchCorrectSin < -PI * 0.5 || prefs.PitchCorrectSin > PI * 0.5 ||
		prefs.PitchCorrectCos < -PI * 0.5 || prefs.PitchCorrectCos > PI * 0.5 )
	{
		prefs.PitchCorrectSin = 0.0f;
		prefs.PitchCorrectCos = 1.0f;
	}

	if( prefs.RollCorrectSin < -PI * 0.5 || prefs.RollCorrectSin > PI * 0.5 ||
		prefs.RollCorrectCos < -PI * 0.5 || prefs.RollCorrectCos > PI * 0.5)
	{
		prefs.RollCorrectSin = 0.0f;
		prefs.RollCorrectCos = 1.0f;
	}


	double RollAngle = asin( prefs.RollCorrectSin );
	double PitchAngle = asin( prefs.PitchCorrectSin );

	QString str = QString( "%1, %2, %3" ).arg( prefs.AccelOffsetX ).arg( prefs.AccelOffsetY ).arg( prefs.AccelOffsetZ );
	ui->lblAccelCalFinal->setText( str );

	AttemptSetValue( ui->udRollCorrection,  RollAngle * 180.0 / PI );
	AttemptSetValue( ui->udPitchCorrection, PitchAngle * 180.0 / PI );


	InternalChange = false;
}


// -----------------------------------------------
// System Test code
// -----------------------------------------------

void MainWindow::on_btnMotorTest_FL_pressed() {
    TestMotor(0);
}

void MainWindow::on_btnMotorTest_FR_pressed() {
    TestMotor(1);
}

void MainWindow::on_btnMotorTest_BR_pressed() {
    TestMotor(2);
}

void MainWindow::on_btnMotorTest_BL_pressed() {
    TestMotor(3);
}

void MainWindow::on_btnBeeper_pressed() {
    TestMotor(4);
}

void MainWindow::on_btnLED_pressed() {
    TestMotor(5);
}

void MainWindow::UpdateElev8Preferences(void)
{
	// Send prefs
	prefs.Checksum = Prefs_CalculateChecksum( prefs );
	SendCommand( "UPrf" );	// Update preferences

	QThread::msleep( 10 );	// Sleep a moment to let the Elev8-FC get ready

	quint8 * prefBytes = (quint8 *)&prefs;

	// Have to slow this down a little during transmission because the buffer
	// on the other end is small to save ram, and we don't have flow control
	for(int i = 0; i < (int)sizeof(prefs); i+=4)
	{
		comm.Send( prefBytes + i, 4 );	// send 4 bytes at a time (assumes the struct is a multiple of 4 bytes)
		QThread::msleep( 5 );			// sleep for a moment to let the Prop commit them
	}

	// Query prefs (forces to be applied to UI)
	SendCommand( "QPRF" );
}


// -----------------------------------------------
// Radio Setup code
// -----------------------------------------------

void MainWindow::SetReverseChannel( int channel , bool bReverse )
{
	if( channel < 1 || channel > 8 ) return;

	short scale = abs(prefs.ChannelScale( channel-1 ));
	if( bReverse ) scale *= -1;

	// Don't re-write the prefs if we set it to the same value
	if( prefs.ChannelScale( channel-1) == scale ) return;

	prefs.ChannelScale( channel-1 ) = scale;
	UpdateElev8Preferences();
}


void MainWindow::on_revR_Channel1_clicked(bool checked) {
	SetReverseChannel( 1 , checked );
}

void MainWindow::on_revR_Channel2_clicked(bool checked) {
	SetReverseChannel( 2 , checked );
}

void MainWindow::on_revR_Channel3_clicked(bool checked) {
	SetReverseChannel( 3 , checked );
}

void MainWindow::on_revR_Channel4_clicked(bool checked) {
	SetReverseChannel( 4 , checked );
}

void MainWindow::on_revR_Channel5_clicked(bool checked) {
	SetReverseChannel( 5 , checked );
}

void MainWindow::on_revR_Channel6_clicked(bool checked) {
	SetReverseChannel( 6 , checked );
}

void MainWindow::on_revR_Channel7_clicked(bool checked) {
	SetReverseChannel( 7 , checked );
}

void MainWindow::on_revR_Channel8_clicked(bool checked) {
	SetReverseChannel( 8 , checked );
}


void MainWindow::on_btnUploadRadioChanges_clicked()
{
	int Value = ui->hsAutoRollPitchSpeed->value();
	float Rate = ((float)Value / 1024.0f) * (PI / 180.0f) * 0.5f;
	prefs.AutoLevelRollPitch = Rate;

	Value = ui->hsAutoYawSpeed->value() * 10;
	Rate = (((float)Value / 250.0f) / 1024.0f) * (PI / 180.0f) * 0.5f;
	prefs.AutoLevelYawRate = Rate;

	Value = ui->hsManualRollPitchSpeed->value() * 10;
	Rate = (((float)Value / 250.0f) / 1024.0f) * (PI / 180.0f) * 0.5f;
	prefs.ManualRollPitchRate = Rate;

	Value = ui->hsManualYawSpeed->value() * 10;
	Rate = (((float)Value / 250.0f) / 1024.0f) * (PI / 180.0f) * 0.5f;
	prefs.ManualYawRate = Rate;

	prefs.AccelCorrectionFilter = (short)ui->hsAccelCorrectionFilter->value();
	prefs.ThrustCorrectionScale = (short)ui->hsThrustCorrection->value();

	UpdateElev8Preferences();
}

void MainWindow::on_btnReceiverReset_clicked()
{
	for(int i = 0; i < 8; i++)
	{
		prefs.ChannelIndex(i)  = i;
		prefs.ChannelScale(i)  = 1024;
		prefs.ChannelCenter(i) = 0;
	}
	UpdateElev8Preferences();
}

void MainWindow::on_btnReceiverCalibrate_clicked()
{
	CalibrateControlsStep = 1;
	CalibrateTimer = 500;
}


void MainWindow::CheckCalibrateControls(void)
{
	switch(CalibrateControlsStep)
	{
		case 0:	return;	// Inactive

		case 1:
			for(int i = 0; i < 8; i++) {
				channelData[i] = ChannelData();	// reset the channel data struct
			}
			SendCommand("Rrad");
			CalibrateControlsStep++;
			return;

		case 2:
		{
			ui->gbControlSetup->setVisible(false);
			ui->lblRadioCalibrateDocs->setVisible(true);

			QString str = QString( \
                    "Move both STICKS all the way to the RIGHT and UP, and all\n" \
					"SWITCHES/KNOBS to their MAXIMUM value position. (1 or 2/clockwise)\n" \
					"(Controls above may respond incorrectly) - %1").arg( CalibrateTimer );

			ui->lblRadioCalibrateDocs->setText( str );

			//if(CalibrateControlsStep == 200) {
			//	SystemSounds.Exclamation.Play();
			//}

			if(CalibrateTimer < 10)
			{
				for(int i = 0; i < 8; i++)
				{
					if( abs(channelData[i].max) < abs( radio[i] ) ) {
						channelData[i].max = radio[i];
					}
				}
			}

			if( --CalibrateTimer == 0 )
			{
				CalibrateControlsStep++;
				CalibrateTimer = 500;
			}
			break;
		}

		case 3:
		{
			QString str = QString( \
                    "Move both STICKS all the way to the LEFT and DOWN, and all\n" \
				    "SWITCHES/KNOBS to their MINIMUM value position (0/counter-clockwise) - %1").arg( CalibrateTimer );

			ui->lblRadioCalibrateDocs->setText( str );

			//if(CalibrateControlsStep == 200) {
			//	SystemSounds.Exclamation.Play();
			//}

			if(CalibrateTimer < 10)
			{
				for(int i = 0; i < 8; i++)
				{
					if( abs(channelData[i].min) < abs( radio[i] ) ) {
						channelData[i].min = radio[i];
					}
				}
			}

			if( --CalibrateTimer == 0 )
			{
				CalibrateControlsStep++;
				CalibrateTimer = 500;
			}
			break;
		}

		case 4:
		{
			CalibrateControlsStep = 0;
			QString str;
			ui->lblRadioCalibrateDocs->setText(str);
			ui->lblRadioCalibrateDocs->setVisible(false);
			ui->gbControlSetup->setVisible(true);

			// figure out reverses
			for(int i = 0; i < 8; i++)
			{
				if(channelData[i].min > channelData[i].max) {
					channelData[i].reverse = true;	// Needs to be reversed with respect to what's already stored
				}

				int range = abs( channelData[i].min - channelData[i].max );
				channelData[i].scale = (int)((1.0f / ((float)range / 2048.0f)) * 1024.0f);
				channelData[i].center = (channelData[i].min + channelData[i].max) / 2;
				if(channelData[i].reverse) channelData[i].scale *= -1;
			}

			// apply to prefs
			prefs.ThroScale = (short)channelData[0].scale;
			prefs.AileScale = (short)channelData[1].scale;
			prefs.ElevScale = (short)channelData[2].scale;
			prefs.RuddScale = (short)channelData[3].scale;
			prefs.GearScale = (short)channelData[4].scale;
			prefs.Aux1Scale = (short)channelData[5].scale;
			prefs.Aux2Scale = (short)channelData[6].scale;
			prefs.Aux3Scale = (short)channelData[7].scale;


			prefs.ThroCenter = (short)channelData[0].center;
			prefs.AileCenter = (short)channelData[1].center;
			prefs.ElevCenter = (short)channelData[2].center;
			prefs.RuddCenter = (short)channelData[3].center;
			prefs.GearCenter = (short)channelData[4].center;
			prefs.Aux1Center = (short)channelData[5].center;
			prefs.Aux2Center = (short)channelData[6].center;
			prefs.Aux3Center = (short)channelData[7].center;

			UpdateElev8Preferences();
			break;
		}
	}
}

void MainWindow::SetChannelMapping( int DestChannel, int SourceChannel )
{
	if( DestChannel < 0 || DestChannel > 7 || SourceChannel < 0 || SourceChannel > 7 ) return;

	prefs.ChannelIndex(DestChannel) = SourceChannel;
	UpdateElev8Preferences();
}

void MainWindow::on_cbR_Channel1_currentIndexChanged(int index) {
	if(!InternalChange) {
		if( RadioMode == 2 ) {
			SetChannelMapping(0, index);
		}
		else {
			SetChannelMapping(2, index);
		}
	}
}

void MainWindow::on_cbR_Channel2_currentIndexChanged(int index) {
	if(!InternalChange) SetChannelMapping(1, index);
}

void MainWindow::on_cbR_Channel3_currentIndexChanged(int index) {
	if(!InternalChange) {
		if( RadioMode == 2 ) {
			SetChannelMapping(2, index);
		}
		else {
			SetChannelMapping(0, index);
		}
	}
}

void MainWindow::on_cbR_Channel4_currentIndexChanged(int index) {
	if(!InternalChange) SetChannelMapping(3, index);
}

void MainWindow::on_cbR_Channel5_currentIndexChanged(int index) {
	if(!InternalChange) SetChannelMapping(4, index);
}

void MainWindow::on_cbR_Channel6_currentIndexChanged(int index) {
	if(!InternalChange) SetChannelMapping(5, index);
}

void MainWindow::on_cbR_Channel7_currentIndexChanged(int index) {
	if(!InternalChange) SetChannelMapping(6, index);
}

void MainWindow::on_cbR_Channel8_currentIndexChanged(int index) {
	if(!InternalChange) SetChannelMapping(7, index);
}



void MainWindow::on_hsAutoRollPitchSpeed_valueChanged(int value)
{
	QString str = QString("%1 deg").arg( value );
	ui->lblAutoRollPitchSpeed->setText( str );
}

void MainWindow::on_hsAutoYawSpeed_valueChanged(int value)
{
	QString str = QString("%1 deg/s").arg( value*10 );
	ui->lblAutoYawSpeed->setText( str );
}

void MainWindow::on_hsManualRollPitchSpeed_valueChanged(int value)
{
	QString str = QString("%1 deg/s").arg( value*10 );
	ui->lblManualRollPitchSpeed->setText( str );
}

void MainWindow::on_hsManualYawSpeed_valueChanged(int value)
{
	QString str = QString("%1 deg/s").arg( value*10 );
	ui->lblManualYawSpeed->setText( str );
}

void MainWindow::on_hsAccelCorrectionFilter_valueChanged(int value)
{
	QString str = QString::number((float) value / 256.f, 'f', 3);
	ui->lblAccelCorrectionFilter->setText( str );
}

void MainWindow::on_hsThrustCorrection_valueChanged(int value)
{
	QString str = QString::number((float) value / 256.f, 'f', 3);
	ui->lblThrustCorrection->setText( str );
}


// -----------------------------------------------
// Flight control setup code
// -----------------------------------------------

void MainWindow::on_hsPitchGain_valueChanged(int value)
{
	if(ui->cbPitchRollLocked->isChecked())
	{
		if(ui->hsRollGain->value() != value) {
			ui->hsRollGain->setValue( value );
		}
	}
	QString str = QString::number( value );
	ui->lblPitchGain->setText( str );
}

void MainWindow::on_hsRollGain_valueChanged(int value)
{
	if(ui->cbPitchRollLocked->isChecked())
	{
		if(ui->hsPitchGain->value() != value) {
			ui->hsPitchGain->setValue( value );
		}
	}
	QString str = QString::number( value );
	ui->lblRollGain->setText( str );
}

void MainWindow::on_cbPitchRollLocked_stateChanged(int arg1)
{
	(void)arg1;
	if(ui->cbPitchRollLocked->isChecked())
	{
		if( ui->hsRollGain->value() != ui->hsPitchGain->value() ) {
			ui->hsRollGain->setValue( ui->hsPitchGain->value() );
		}
	}
}

void MainWindow::on_hsYawGain_valueChanged(int value)
{
	QString str = QString::number( value );
	ui->lblYawGain->setText( str );
}


void MainWindow::on_hsAscentGain_valueChanged(int value)
{
	QString str = QString::number( value );
	ui->lblAscentGain->setText( str );
}

void MainWindow::on_hsAltiGain_valueChanged(int value)
{
	QString str = QString::number( value );
	ui->lblAltiGain->setText( str );
}

void MainWindow::on_btnUploadFlightChanges_clicked()
{
	prefs.PitchRollLocked = ui->cbPitchRollLocked->isChecked() ? (quint8)1 : (quint8)0;
	prefs.UseAdvancedPID = 0; // cbEnableAdvanced->isChecked() ? (quint8)1 : (quint8)0;

	prefs.PitchGain =  (quint8)(ui->hsPitchGain->value() - 1);
	prefs.RollGain =   (quint8)(ui->hsRollGain->value() - 1);
	prefs.YawGain =    (quint8)(ui->hsYawGain->value() - 1);
	prefs.AscentGain = (quint8)(ui->hsAscentGain->value() - 1);
	prefs.AltiGain =   (quint8)(ui->hsAltiGain->value() - 1);

	// Apply the prefs to the elev-8
	UpdateElev8Preferences();
}


// -----------------------------------------------
// System setup code
// -----------------------------------------------

void MainWindow::on_btnUploadSystemSetup_clicked()
{
	static qint16 DelayTable[] = {250, 125, 62, 0 };

	prefs.MinThrottle = (qint16)(ui->sbLowThrottle->value() * 8);
	prefs.MinThrottleArmed = (qint16)(ui->sbArmedLowThrottle->value() * 8);
	prefs.ThrottleTest = (qint16)(ui->sbTestThrottle->value() * 8);
	prefs.MaxThrottle = (qint16)(ui->sbHighThrottle->value() * 8);

	prefs.UseBattMon = (quint8)(ui->cbUseBatteryMonitor->isChecked() ? 1 : 0);
	prefs.LowVoltageAlarmThreshold = (qint16)(ui->sbLowVoltageAlarmThreshold->value() * 100);
	prefs.VoltageOffset = (qint16)(ui->sbVoltageOffset->value() * 100);
	prefs.LowVoltageAlarm = (quint8)(ui->cbLowVoltageAlarm->isChecked() ? 1 : 0);

	prefs.ArmDelay = DelayTable[ui->cbArmingDelay->currentIndex()];
	prefs.DisarmDelay = DelayTable[ui->cbDisarmDelay->currentIndex()];

	prefs.DisableMotors = (quint8)(ui->btnDisableMotors->isChecked() ? 1 : 0);

	UpdateElev8Preferences();
}



// -----------------------------------------------
// Gyro Calibrate code
// -----------------------------------------------

void MainWindow::on_btnRestartCalibration_clicked()
{
	ui->lfGyroGraph->Reset();
}

void MainWindow::on_btnUploadGyroCalibration_clicked()
{
	prefs.DriftScaleX =  (int)round( 1.0 / ui->lfGyroGraph->dSlope.x );
	prefs.DriftScaleY =  (int)round( 1.0 / ui->lfGyroGraph->dSlope.y );
	prefs.DriftScaleZ =  (int)round( 1.0 / ui->lfGyroGraph->dSlope.z );
	prefs.DriftOffsetX = (int)round( ui->lfGyroGraph->dIntercept.x );
	prefs.DriftOffsetY = (int)round( ui->lfGyroGraph->dIntercept.y );
	prefs.DriftOffsetZ = (int)round( ui->lfGyroGraph->dIntercept.z );

	UpdateElev8Preferences();
}


// -----------------------------------------------
// Accel Calibrate code
// -----------------------------------------------

void MainWindow::GetAccelAvgSasmple( int i )
{
	QLabel * labels[] = { ui->lblAccelCal1, ui->lblAccelCal2, ui->lblAccelCal3, ui->lblAccelCal4 };

	accXCal[i] = ui->gAccelXCal->getMovingAverage();
	accYCal[i] = ui->gAccelYCal->getMovingAverage();
	accZCal[i] = ui->gAccelZCal->getMovingAverage();

	QString str = QString("%1, %2, %3")
        .arg(QString::number(accXCal[i], 'f', 1))
        .arg(QString::number(accYCal[i], 'f', 1))
        .arg(QString::number(accZCal[i], 'f', 1));
       
	labels[i]->setText( str );
}

void MainWindow::on_btnAccelCal1_clicked() {
	GetAccelAvgSasmple(0);
}

void MainWindow::on_btnAccelCal2_clicked() {
	GetAccelAvgSasmple(1);
}

void MainWindow::on_btnAccelCal3_clicked() {
	GetAccelAvgSasmple(2);
}

void MainWindow::on_btnAccelCal4_clicked() {
	GetAccelAvgSasmple(3);
}

void MainWindow::on_btnAccelCalUpload_clicked()
{
	float fx = 0.0f, fy = 0.0f, fz = 0.0f;
	for(int i = 0; i < 4; i++)
	{
		fx += accXCal[i] * 0.25f;
		fy += accYCal[i] * 0.25f;
		fz += accZCal[i] * 0.25f;
	}

	int ax = (int)roundf( fx );
	int ay = (int)roundf( fy );
	int az = (int)roundf( fz );

	QString str = QString("%1, %2, %3").arg(ax).arg(ay).arg(az);
	ui->lblAccelCalFinal->setText( str );

	az -= 4096;	// OneG

	prefs.AccelOffsetX = ax;
	prefs.AccelOffsetY = ay;
	prefs.AccelOffsetZ = az;

	UpdateElev8Preferences();
}

void MainWindow::on_btnUploadAngleCorrection_clicked()
{
	// Upload calibration data
	double rollOffset = (float)((double)ui->udRollCorrection->value() * PI / 180.0);
	double pitchOffset = (float)((double)ui->udPitchCorrection->value() * PI / 180.0);

	prefs.RollCorrectSin =  sinf( rollOffset );
	prefs.RollCorrectCos =  cosf( rollOffset );
	prefs.PitchCorrectSin = sinf( pitchOffset );
	prefs.PitchCorrectCos = cosf( pitchOffset );

	UpdateElev8Preferences();
}

void MainWindow::on_actionAbout_triggered()
{
	QDialog * dlg = new AboutBox(this);
	dlg->show();
}

void MainWindow::on_cbGyroX_clicked(bool checked)		{graphs[ 0]->setVisible( checked ); }
void MainWindow::on_cbGyroY_clicked(bool checked)		{graphs[ 1]->setVisible( checked ); }
void MainWindow::on_cbGyroZ_clicked(bool checked)		{graphs[ 2]->setVisible( checked ); }
void MainWindow::on_cbAccelX_clicked(bool checked)		{graphs[ 3]->setVisible( checked ); }
void MainWindow::on_cbAccelY_clicked(bool checked)		{graphs[ 4]->setVisible( checked ); }
void MainWindow::on_cbAccelZ_clicked(bool checked)		{graphs[ 5]->setVisible( checked ); }
void MainWindow::on_cbMagX_clicked(bool checked)		{graphs[ 6]->setVisible( checked ); }
void MainWindow::on_cbMagY_clicked(bool checked)		{graphs[ 7]->setVisible( checked ); }
void MainWindow::on_cbMagZ_clicked(bool checked)		{graphs[ 8]->setVisible( checked ); }
void MainWindow::on_cbGyroTemp_clicked(bool checked)	{graphs[ 9]->setVisible( checked ); }
void MainWindow::on_cbAlti_clicked(bool checked)		{graphs[10]->setVisible( checked ); }
void MainWindow::on_cbAltiEst_clicked(bool checked)		{graphs[11]->setVisible( checked ); }
void MainWindow::on_cbLaserHeight_clicked(bool checked) {graphs[12]->setVisible( checked ); }
void MainWindow::on_cbPitch_clicked(bool checked)		{graphs[13]->setVisible( checked ); }
void MainWindow::on_cbRoll_clicked(bool checked)		{graphs[14]->setVisible( checked ); }
void MainWindow::on_cbYaw_clicked(bool checked)			{graphs[15]->setVisible( checked ); }
void MainWindow::on_cbVoltage_clicked(bool checked)		{graphs[16]->setVisible( checked ); }


void MainWindow::on_actionExport_Settings_to_File_triggered()
{
	// Get the filename from the user
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Settings File"), QDir::currentPath(), tr("Elev8 Settings Files (*.elev8set *.xml)"));
	if (fileName.isEmpty())
		return;

	QFile file(fileName);
	if(!file.open(QFile::WriteOnly | QFile::Text)) {
		return;
	}

	// Serialize the settings data into an XML file
	WriteSettings( &file );

	file.close();
}


void MainWindow::on_actionImport_Settings_from_File_triggered()
{
	// Get the filename from the user
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open Settings File"), QDir::currentPath(), tr("Elev8 Settings Files (*.elev8set *.xml)"));
	if (fileName.isEmpty())
		return;

	QFile file(fileName);
	if(!file.open(QFile::ReadOnly | QFile::Text)) {
		return;
	}

	// Deserialize the settings data from the XML file
	ReadSettings( &file );

	file.close();
}


// With QString::number handling overloads for most types, this function should work for just about anything
template<typename TYPE> static void WritePref( QXmlStreamWriter & writer , const char * pPrefName , TYPE Value )
{
	writer.writeStartElement( pPrefName );
	writer.writeAttribute( "Value" , QString::number(Value) );
	writer.writeEndElement();
}


void MainWindow::WriteSettings( QIODevice *file )
{
	QXmlStreamWriter writer( file );
	writer.setAutoFormatting(true);
	writer.writeStartDocument();

	writer.writeStartElement("Elev8-Prefs");

	QString verString = QString( "%1.%2" ).arg(debugData.Version >> 8).arg( debugData.Version & 255, 2, 10, QChar('0'));
	writer.writeAttribute("Version", verString );

	WritePref( writer, "DriftScaleX", prefs.DriftScaleX );
	WritePref( writer, "DriftScaleY", prefs.DriftScaleY );
	WritePref( writer, "DriftScaleZ", prefs.DriftScaleZ );

	WritePref( writer, "DriftOffsetX", prefs.DriftOffsetX );
	WritePref( writer, "DriftOffsetY", prefs.DriftOffsetY );
	WritePref( writer, "DriftOffsetZ", prefs.DriftOffsetZ );

	WritePref( writer, "AccelOffsetX", prefs.AccelOffsetX );
	WritePref( writer, "AccelOffsetY", prefs.AccelOffsetY );
	WritePref( writer, "AccelOffsetZ", prefs.AccelOffsetZ );

	WritePref( writer, "MagOfsX", prefs.MagOfsX );
	WritePref( writer, "MagOfsY", prefs.MagOfsY );
	WritePref( writer, "MagOfsZ", prefs.MagOfsZ );

	WritePref( writer, "MagScaleX", prefs.MagScaleX );
	WritePref( writer, "MagScaleY", prefs.MagScaleY );
	WritePref( writer, "MagScaleZ", prefs.MagScaleZ );

	WritePref( writer, "RollCorrectSin", prefs.RollCorrectSin );
	WritePref( writer, "RollCorrectCos", prefs.RollCorrectCos );

	WritePref( writer, "PitchCorrectSin", prefs.PitchCorrectSin );
	WritePref( writer, "PitchCorrectCos", prefs.PitchCorrectCos );


	float RateScale = 2.0f / (PI/180.0f) * 1024.0;
	WritePref( writer, "AutoLevelRollPitch", prefs.AutoLevelRollPitch * RateScale );
	WritePref( writer, "AutoLevelYawRate", prefs.AutoLevelYawRate * RateScale * 250.0f );
	WritePref( writer, "ManualRollPitchRate", prefs.ManualRollPitchRate * RateScale * 250.0f );
	WritePref( writer, "ManualYawRate", prefs.ManualYawRate * RateScale * 250.0f );

	WritePref( writer, "PitchGain", prefs.PitchGain );
	WritePref( writer, "RollGain", prefs.RollGain );
	WritePref( writer, "YawGain", prefs.YawGain );
	WritePref( writer, "AscentGain", prefs.AscentGain );

	WritePref( writer, "AltiGain", prefs.AltiGain );
	WritePref( writer, "PitchRollLocked", prefs.PitchRollLocked );
	WritePref( writer, "UseAdvancedPID", prefs.UseAdvancedPID );
	WritePref( writer, "ReceiverType", prefs.ReceiverType );

	WritePref( writer, "UseBattMon", prefs.UseBattMon );
	WritePref( writer, "DisableMotors", prefs.DisableMotors );
	WritePref( writer, "LowVoltageAlarm", prefs.LowVoltageAlarm );
	WritePref( writer, "LowVoltageAscentLimit", prefs.LowVoltageAscentLimit );

	WritePref( writer, "ThrottleTest", prefs.ThrottleTest );
	WritePref( writer, "MinThrottle", prefs.MinThrottle );
	WritePref( writer, "MaxThrottle", prefs.MaxThrottle );
	WritePref( writer, "CenterThrottle", prefs.CenterThrottle );
	WritePref( writer, "MinThrottleArmed", prefs.MinThrottleArmed );
	WritePref( writer, "ArmDelay", prefs.ArmDelay );
	WritePref( writer, "DisarmDelay", prefs.DisarmDelay );

	WritePref( writer, "ThrustCorrectionScale", prefs.ThrustCorrectionScale );
	WritePref( writer, "AccelCorrectionFilter", prefs.AccelCorrectionFilter );

	WritePref( writer, "VoltageOffset", prefs.VoltageOffset );
	WritePref( writer, "LowVoltageAlarmThreshold", prefs.LowVoltageAlarmThreshold );

	WritePref( writer, "ThroChannel", prefs.ThroChannel );
	WritePref( writer, "AileChannel", prefs.AileChannel );
	WritePref( writer, "ElevChannel", prefs.ElevChannel );
	WritePref( writer, "RuddChannel", prefs.RuddChannel );
	WritePref( writer, "GearChannel", prefs.GearChannel );
	WritePref( writer, "Aux1Channel", prefs.Aux1Channel );
	WritePref( writer, "Aux2Channel", prefs.Aux2Channel );
	WritePref( writer, "Aux3Channel", prefs.Aux3Channel );

	WritePref( writer, "ThroScale", prefs.ThroScale );
	WritePref( writer, "ThroCenter", prefs.ThroCenter );

	WritePref( writer, "AileScale", prefs.AileScale );
	WritePref( writer, "AileCenter", prefs.AileCenter );

	WritePref( writer, "ElevScale", prefs.ElevScale );
	WritePref( writer, "ElevCenter", prefs.ElevCenter );

	WritePref( writer, "RuddScale", prefs.RuddScale );
	WritePref( writer, "RuddCenter", prefs.RuddCenter );

	WritePref( writer, "GearScale", prefs.GearScale );
	WritePref( writer, "GearCenter", prefs.GearCenter );

	WritePref( writer, "Aux1Scale", prefs.Aux1Scale );
	WritePref( writer, "Aux1Center", prefs.Aux1Center );

	WritePref( writer, "Aux2Scale", prefs.Aux2Scale );
	WritePref( writer, "Aux2Center", prefs.Aux2Center );

	WritePref( writer, "Aux3Scale", prefs.Aux3Scale );
	WritePref( writer, "Aux3Center", prefs.Aux3Center );

	writer.writeEndElement();	// prefs block
	writer.writeEndDocument();
}

void MainWindow::ReadSettings( QIODevice *file )
{
	QXmlStreamReader reader;

	// Read the settings from the XML file
	reader.setDevice( file );
	reader.readNext();

	while(!reader.atEnd())
	{
		if(reader.isStartElement())
		{
			if(reader.name() == "Elev8-Prefs")
			{
				if( reader.attributes().length() > 0 )
				{
					QXmlStreamAttribute attr = reader.attributes()[0];
					if( attr.name() == "Version" )
					{
						QString val = attr.value().toString();
						ReadSettingsContents( reader );
					}
				}
			}
			else {
				// This is not an elev8 settings file
				// TODO: Display an error
				return;
			}
			reader.readNext();
		}
		else if( reader.isEndElement() ) {
			reader.readNext();
		}
		else
			reader.readNext();
	}

	// Finally, set the UI from the prefs
	ConfigureUIFromPreferences();
}


static bool ReadInt( QXmlStreamReader & reader , int & val )
{
	bool ok = false;
	QXmlStreamAttribute attr = reader.attributes()[0];
	if( attr.name() == "Value" ) {
		int temp = attr.value().toInt( &ok );
		if( ok ) val = temp;
	}
	return ok;
}

static bool ReadInt( QXmlStreamReader & reader , short & val )
{
	bool ok = false;
	QXmlStreamAttribute attr = reader.attributes()[0];
	if( attr.name() == "Value" ) {
		int temp = attr.value().toInt( &ok );
		if( ok ) val = (short)temp;
	}
	return ok;
}

static bool ReadInt( QXmlStreamReader & reader , char & val )
{
	bool ok = false;
	QXmlStreamAttribute attr = reader.attributes()[0];
	if( attr.name() == "Value" ) {
		int temp = attr.value().toInt( &ok );
		if( ok ) val = (char)temp;
	}
	return ok;
}


static bool ReadFloat( QXmlStreamReader & reader , float & val , float scale = 1.0f )
{
	bool ok = false;
	QXmlStreamAttribute attr = reader.attributes()[0];
	if( attr.name() == "Value" ) {
		float temp = attr.value().toFloat( &ok );
		if( ok ) val = temp / scale;
	}
	return ok;
}


void MainWindow::ReadSettingsContents( QXmlStreamReader & reader )
{
	reader.readNext();

	const float RateScale = 1.0 / (2.0f / (PI/180.0f) * 1024.0);

	while( !(reader.isEndElement() && reader.name() == "Elev8-Prefs") )
	{
		if( reader.isStartElement() )
		{
			if(      reader.name() == "DriftScaleX")			ReadInt(reader, prefs.DriftScaleX);
			else if( reader.name() == "DriftScaleY" )			ReadInt(reader, prefs.DriftScaleY);
			else if( reader.name() == "DriftScaleZ")			ReadInt(reader, prefs.DriftScaleZ);
			else if( reader.name() == "DriftOffsetX")			ReadInt(reader, prefs.DriftOffsetX);
			else if( reader.name() == "DriftOffsetY")			ReadInt(reader, prefs.DriftOffsetY);
			else if( reader.name() == "DriftOffsetZ")			ReadInt(reader, prefs.DriftOffsetZ);

			else if( reader.name() == "AccelOffsetX")			ReadInt(reader, prefs.AccelOffsetX);
			else if( reader.name() == "AccelOffsetY")			ReadInt(reader, prefs.AccelOffsetY);
			else if( reader.name() == "AccelOffsetZ")			ReadInt(reader, prefs.AccelOffsetZ);

			else if( reader.name() == "MagOfsX")				ReadInt(reader, prefs.MagOfsX);
			else if( reader.name() == "MagOfsY")				ReadInt(reader, prefs.MagOfsY);
			else if( reader.name() == "MagOfsZ")				ReadInt(reader, prefs.MagOfsZ);
			else if( reader.name() == "MagScaleX")				ReadInt(reader, prefs.MagScaleX);
			else if( reader.name() == "MagScaleY")				ReadInt(reader, prefs.MagScaleY);
			else if( reader.name() == "MagScaleZ")				ReadInt(reader, prefs.MagScaleZ);

			else if( reader.name() == "RollCorrectSin")			ReadFloat(reader, prefs.RollCorrectSin);
			else if( reader.name() == "RollCorrectCos")			ReadFloat(reader, prefs.RollCorrectCos);
			else if( reader.name() == "PitchCorrectSin")		ReadFloat(reader, prefs.PitchCorrectSin);
			else if( reader.name() == "PitchCorrectCos")		ReadFloat(reader, prefs.PitchCorrectCos);

			else if( reader.name() == "AutoLevelRollPitch")		ReadFloat(reader, prefs.AutoLevelRollPitch, RateScale );
			else if( reader.name() == "AutoLevelYawRate")		ReadFloat(reader, prefs.AutoLevelYawRate, RateScale / 250.0f );
			else if( reader.name() == "ManualRollPitchRate")	ReadFloat(reader, prefs.ManualRollPitchRate, RateScale / 250.0f );
			else if( reader.name() == "ManualYawRate")			ReadFloat(reader, prefs.ManualYawRate, RateScale / 250.0f );

			else if( reader.name() == "PitchGain")				ReadInt(reader, prefs.PitchGain);
			else if( reader.name() == "RollGain")				ReadInt(reader, prefs.RollGain);
			else if( reader.name() == "YawGain")				ReadInt(reader, prefs.YawGain);
			else if( reader.name() == "AscentGain")				ReadInt(reader, prefs.AscentGain);
			else if( reader.name() == "AltiGain")				ReadInt(reader, prefs.AltiGain);
			else if( reader.name() == "PitchRollLocked")		ReadInt(reader, prefs.PitchRollLocked);
			else if( reader.name() == "UseAdvancedPID")			ReadInt(reader, prefs.UseAdvancedPID);

			else if( reader.name() == "ReceiverType")			ReadInt(reader, prefs.ReceiverType);
			else if( reader.name() == "UseBattMon")				ReadInt(reader, prefs.UseBattMon);
			else if( reader.name() == "DisableMotors")			ReadInt(reader, prefs.DisableMotors);
			else if( reader.name() == "LowVoltageAlarm")		ReadInt(reader, prefs.LowVoltageAlarm);
			else if( reader.name() == "LowVoltageAscentLimit")	ReadInt(reader, prefs.LowVoltageAscentLimit);

			else if( reader.name() == "ThrottleTest")			ReadInt(reader, prefs.ThrottleTest);
			else if( reader.name() == "MinThrottle")			ReadInt(reader, prefs.MinThrottle);
			else if( reader.name() == "MaxThrottle")			ReadInt(reader, prefs.MaxThrottle);
			else if( reader.name() == "CenterThrottle")			ReadInt(reader, prefs.CenterThrottle);
			else if( reader.name() == "MinThrottleArmed")		ReadInt(reader, prefs.MinThrottleArmed);
			else if( reader.name() == "ArmDelay")				ReadInt(reader, prefs.ArmDelay);
			else if( reader.name() == "DisarmDelay")			ReadInt(reader, prefs.DisarmDelay);
			else if( reader.name() == "ThrustCorrectionScale")	ReadInt(reader, prefs.ThrustCorrectionScale);
			else if( reader.name() == "AccelCorrectionFilter")	ReadInt(reader, prefs.AccelCorrectionFilter);
			else if( reader.name() == "VoltageOffset")			ReadInt(reader, prefs.VoltageOffset);
			else if( reader.name() == "LowVoltageAlarmThreshold")	ReadInt(reader, prefs.LowVoltageAlarmThreshold);

			else if( reader.name() == "ThroChannel")			ReadInt(reader, prefs.ThroChannel);
			else if( reader.name() == "AileChannel")			ReadInt(reader, prefs.AileChannel);
			else if( reader.name() == "ElevChannel")			ReadInt(reader, prefs.ElevChannel);
			else if( reader.name() == "RuddChannel")			ReadInt(reader, prefs.RuddChannel);
			else if( reader.name() == "GearChannel")			ReadInt(reader, prefs.GearChannel);
			else if( reader.name() == "Aux1Channel")			ReadInt(reader, prefs.Aux1Channel);
			else if( reader.name() == "Aux2Channel")			ReadInt(reader, prefs.Aux2Channel);
			else if( reader.name() == "Aux3Channel")			ReadInt(reader, prefs.Aux3Channel);

			else if( reader.name() == "ThroScale")				ReadInt(reader, prefs.ThroScale);
			else if( reader.name() == "AileScale")				ReadInt(reader, prefs.AileScale);
			else if( reader.name() == "ElevScale")				ReadInt(reader, prefs.ElevScale);
			else if( reader.name() == "RuddScale") 				ReadInt(reader, prefs.RuddScale);
			else if( reader.name() == "GearScale") 				ReadInt(reader, prefs.GearScale);
			else if( reader.name() == "Aux1Scale") 				ReadInt(reader, prefs.Aux1Scale);
			else if( reader.name() == "Aux2Scale") 				ReadInt(reader, prefs.Aux2Scale);
			else if( reader.name() == "Aux3Scale") 				ReadInt(reader, prefs.Aux3Scale);

			else if( reader.name() == "ThroCenter")				ReadInt(reader, prefs.ThroCenter);
			else if( reader.name() == "AileCenter")				ReadInt(reader, prefs.AileCenter);
			else if( reader.name() == "ElevCenter")				ReadInt(reader, prefs.ElevCenter);
			else if( reader.name() == "RuddCenter")				ReadInt(reader, prefs.RuddCenter);
			else if( reader.name() == "GearCenter")				ReadInt(reader, prefs.GearCenter);
			else if( reader.name() == "Aux1Center")				ReadInt(reader, prefs.Aux1Center);
			else if( reader.name() == "Aux2Center")				ReadInt(reader, prefs.Aux2Center);
			else if( reader.name() == "Aux3Center")				ReadInt(reader, prefs.Aux3Center);
		}

		reader.readNext();
	}
}
