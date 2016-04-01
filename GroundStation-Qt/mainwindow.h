#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QLabel>
#include <QQuaternion>
#include <QVector>

#include "connection.h"
#include "elev8data.h"
#include "prefs.h"
#include "qcustomplot.h"

class QComboBox;
class QSlider;
class QSpinBox;
class QDoubleSpinBox;
class QScrollBar;


namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

	void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

	void UpdateStatus(void);
	void SendCommand(const char *command);
	void SendCommand(QString command);
	void ProcessPackets(void);

	void ConfigureUIFromPreferences(void);
	void UpdateElev8Preferences(void);


private slots:
	void on_btnBeeper_pressed();
	void on_btnLED_pressed();
	void on_btnMotorTest_FL_pressed();
	void on_btnMotorTest_FR_pressed();
	void on_btnMotorTest_BR_pressed();
	void on_btnMotorTest_BL_pressed();

	void on_btnThrottleCalibrate_clicked();
	void on_btnSafetyCheck_clicked();

	void on_connectionMade();

	void on_revR_Channel1_clicked(bool checked);
	void on_revR_Channel2_clicked(bool checked);
	void on_revR_Channel3_clicked(bool checked);
	void on_revR_Channel4_clicked(bool checked);
	void on_revR_Channel5_clicked(bool checked);
	void on_revR_Channel6_clicked(bool checked);
	void on_revR_Channel7_clicked(bool checked);
	void on_revR_Channel8_clicked(bool checked);

	void on_btnUploadRadioChanges_clicked();

	void on_btnReceiverReset_clicked();
	void on_btnReceiverCalibrate_clicked();

	void on_hsAutoRollPitchSpeed_valueChanged(int value);
	void on_hsAutoYawSpeed_valueChanged(int value);
	void on_hsManualRollPitchSpeed_valueChanged(int value);
	void on_hsManualYawSpeed_valueChanged(int value);

	void on_cbR_Channel1_currentIndexChanged(int index);
	void on_cbR_Channel2_currentIndexChanged(int index);
	void on_cbR_Channel3_currentIndexChanged(int index);
	void on_cbR_Channel4_currentIndexChanged(int index);
	void on_cbR_Channel5_currentIndexChanged(int index);
	void on_cbR_Channel6_currentIndexChanged(int index);
	void on_cbR_Channel7_currentIndexChanged(int index);
	void on_cbR_Channel8_currentIndexChanged(int index);

	void on_hsAccelCorrectionFilter_valueChanged(int value);
	void on_hsThrustCorrection_valueChanged(int value);
	void on_btnUploadSystemSetup_clicked();

	void on_actionReset_Flight_Controller_triggered();
	void on_actionRadio_Mode_1_triggered();
	void on_actionRadio_Mode_2_triggered();
	void on_actionRestore_Factory_Defaults_triggered();

	void on_btnRestartCalibration_clicked();
	void on_btnUploadGyroCalibration_clicked();

	void on_btnAccelCal1_clicked();
	void on_btnAccelCal2_clicked();
	void on_btnAccelCal3_clicked();
	void on_btnAccelCal4_clicked();
	void on_btnAccelCalUpload_clicked();

	void on_tabWidget_currentChanged(int index);

	void on_hsPitchGain_valueChanged(int value);
	void on_hsRollGain_valueChanged(int value);
	void on_cbPitchRollLocked_stateChanged(int arg1);
	void on_hsYawGain_valueChanged(int value);
	void on_hsAscentGain_valueChanged(int value);
	void on_hsAltiGain_valueChanged(int value);
	void on_btnUploadFlightChanges_clicked();

	void on_btnUploadAngleCorrection_clicked();

	void on_actionAbout_triggered();

	void on_cbGyroX_clicked();
	void on_cbGyroY_clicked();
	void on_cbGyroZ_clicked();
	void on_cbAccelX_clicked();
	void on_cbAccelY_clicked();
	void on_cbAccelZ_clicked();
	void on_cbMagX_clicked();
	void on_cbMagY_clicked();
	void on_cbMagZ_clicked();
	void on_cbGyroTemp_clicked();
	void on_cbAlti_clicked();
	void on_cbAltiEst_clicked();
	void on_cbLaserHeight_clicked();

	void on_actionExport_Settings_to_File_triggered();
	void on_actionImport_Settings_from_File_triggered();

private:
	void FillChannelComboBox( QComboBox *cb , int defaultIndex );
	void SetChannelMapping( int DestChannel, int SourceChannel );

	void SetRadioMode( int mode );

	void AttemptSetValue( QSlider * slider , int value );
	void AttemptSetValue( QSpinBox * slider , int value );
	void AttemptSetValue( QDoubleSpinBox * slider , double value );
	void AttemptSetValue( QScrollBar * slider , int value );
	void SetReverseChannel(int channel, bool bReverse);

	void TestMotor(int);
	void CancelThrottleCalibration(void);
	void AbortThrottleCalibrationWithMessage( QString & msg , int delay );
	void CheckCalibrateControls(void);

	void loadSettings(void);
	void saveSettings(void);

	void WriteSettings( QIODevice *file );
	void ReadSettings( QIODevice *file );
	void ReadSettingsContents( QXmlStreamReader & reader );

	void GetAccelAvgSasmple( int i );

	QString m_sSettingsFile;


	enum Mode {
		None,
		RadioTest,
		SensorTest,
		MotorTest,
		GyroCalibration,
		AccelCalibration,
	};

	Mode currentMode;

	Ui::MainWindow *ui;
	QLabel * labelStatus;
	QLabel * labelGSVersion;
	QLabel * labelFWVersion;

	int Heartbeat;
	int RadioMode;	// Mode == 1 or 2

	Connection comm;
	CommStatus stat;

	RadioData radio;
	SensorData sensors;
	QQuaternion q;
	QQuaternion cq;
	MotorData motors;
	ComputedData computed;
	DebugValues debugData;

	float accXCal[4];
	float accYCal[4];
	float accZCal[4];

	bool InternalChange;

	int ThrottleCalibrationCycle;

	int CalibrateControlsStep;
	int CalibrateTimer;

	ChannelData channelData[8];

	int SampleIndex;
	QCPGraph * graphs[16];
	QCustomPlot * sg;

	PREFS prefs;
};

#endif // MAINWINDOW_H
