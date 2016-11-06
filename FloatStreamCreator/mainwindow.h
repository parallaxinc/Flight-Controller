#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QQuaternion>
#include <QVector>
#include <QVector3D>

#include "connection.h"
#include "elev8data.h"



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


private slots:
	void on_btnCompile_clicked();

	void on_elevationRot_valueChanged(int value);

	void on_headingRot_valueChanged(int value);

	void on_pbMagCalibrate_clicked();

private:

	QLabel * labelStatus;
	QLabel * labelGSVersion;
	QLabel * labelFWVersion;

	int Heartbeat;
	bool showHexMode;

	Connection comm;
	CommStatus stat;

	RadioPacked packedRadio;
	RadioData radio;
	SensorData sensors;
	QQuaternion q;		// External
	QQuaternion cq;		// Computed
	MotorData motors;
	ComputedData computed;
	DebugValues debugData;

	float accXCal[4];
	float accYCal[4];
	float accZCal[4];

	bool InternalChange;

	int pointsUsed, pointIndex;
	QVector3D pts[4096];

	void UpdateMagSphere(void);

	Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
