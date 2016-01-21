#ifndef ORIENTATION_WIDGET_H
#define ORIENTATION_WIDGET_H

#include <QWidget>
#include <QPointF>
#include <QVector>
#include <QVector3D>
#include <QMatrix4x4>
#include <QQuaternion>


class Orientation_Widget : public QWidget
{
	Q_OBJECT

public:
	Orientation_Widget(QWidget *parent = 0);
	~Orientation_Widget();

	void Init(void);
	void setQuat( QQuaternion q );
	void setQuat2( QQuaternion q );

protected:
	void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
	void DrawShape( QPainter &painter, QMatrix4x4 &m, QColor col, QVector<QVector3D> &points, QVector<int> &lines );

private:
	QVector<QVector3D>	pt;		// Transformed points
	QVector<QPointF>	Quad2d;	// 2d source points for quad shape
	QVector<QVector3D>	QuadPt;	// 3D points (top, bottom, connectors, line for heading)

	QVector<int>		QuadLine;

	float CenterX, CenterY;

	static const float ViewDist = 9.0f;
	static const float ViewScale = 1200.0f;

	float DrawScale;

	QQuaternion quat, quat2;
	QMatrix4x4 mat, mat2;

	bool bQuat2Valid;

};

#endif
