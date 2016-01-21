#include "orientation_widget.h"
#include <QPainter>
#include <QBitmap>
#include <math.h>

Orientation_Widget::Orientation_Widget(QWidget *parent) : QWidget(parent)
{
	Init();
}

Orientation_Widget::~Orientation_Widget()
{
}

void Orientation_Widget::Init(void)
{
	Quad2d.clear();
	Quad2d.append( QPointF( 0.9192f,  0.8485f) );
	Quad2d.append( QPointF( 0.3000f,  0.2293f) );
	Quad2d.append( QPointF( 0.3000f, -0.2293f) );
	Quad2d.append( QPointF( 0.9192f, -0.8485f) );
	Quad2d.append( QPointF( 0.8485f, -0.9192f) );
	Quad2d.append( QPointF( 0.2293f, -0.3000f) );
	Quad2d.append( QPointF(-0.2293f, -0.3000f) );
	Quad2d.append( QPointF(-0.8485f, -0.9192f) );
	Quad2d.append( QPointF(-0.9192f, -0.8485f) );
	Quad2d.append( QPointF(-0.3000f, -0.2293f) );
	Quad2d.append( QPointF(-0.3000f,  0.2293f) );
	Quad2d.append( QPointF(-0.9192f,  0.8485f) );
	Quad2d.append( QPointF(-0.8485f,  0.9192f) );
	Quad2d.append( QPointF(-0.2293f,  0.3000f) );
	Quad2d.append( QPointF( 0.2293f,  0.3000f) );
	Quad2d.append( QPointF( 0.8485f,  0.9192f) );


	CenterX = CenterY = 0.0f;

	const int sourceLine[] = {		0,1, 1,2, 2,3, 3,4,
									4,5, 5,6, 6,7, 7,8,			// Top shell
									8,9, 9,10, 10,11, 11,12,
									12,13, 13,14, 14,15, 15,0,

									16,17, 17,18, 18,19, 19,20,
									20,21, 21,22, 22,23, 23,24,	// Bottom shell
									24,25, 25,26, 26,27, 27,28,
									28,29, 29,30, 30,31, 31,16,

									0,16, 1,17, 2,18, 3,19,
									4,20, 5,21, 6,22, 7,23,		// Connections between top & bottom
									8,24, 9,25, 10,26, 11,27,
									12,28, 13,29, 14,30, 15,31,

									2,5, 6,9, 10,13, 14,1,		// Connections on arms, top & bottom
									18,21, 22,25, 26,29, 30,17,

									32,33
							};

	QuadLine.clear();
	for( unsigned int i=0; i<sizeof(sourceLine)/sizeof(int); i++ ) {
		QuadLine.append( sourceLine[i] );
	}

	float cubeHeight = 0.8;
	float cubeWidth =  1.2;
	float cubeDepth =  1.2;

	QuadPt.resize(34);
	pt.resize(34);

	for(int i = 0; i < 16; i++) {
		QuadPt[i] =    QVector3D( Quad2d[i].x() * cubeWidth, -cubeHeight * 0.1f, Quad2d[i].y() * cubeDepth );
		QuadPt[i+16] = QVector3D( Quad2d[i].x() * cubeWidth, +cubeHeight * 0.1f, Quad2d[i].y() * cubeDepth );
	}

	QuadPt[32] = QVector3D( 0.0f, 0.0f, 0.3f * cubeDepth );
	QuadPt[33] = QVector3D( 0.0f, 0.0f, 0.5f * cubeDepth );

	bQuat2Valid = false;
}

void Orientation_Widget::setQuat( QQuaternion q )
{
	quat = q;
	QMatrix3x3 m = quat.toRotationMatrix();
	mat = QMatrix4x4(m);

	update();
}

void Orientation_Widget::setQuat2( QQuaternion q )
{
	quat2 = q;
	QMatrix3x3 m = quat2.toRotationMatrix();
	mat2 = QMatrix4x4(m);
	bQuat2Valid = true;

	update();
}

void Orientation_Widget::paintEvent(QPaintEvent * event)
{
	(void)event;

	QPainter painter(this);

	// Draw the border and fill the background
	painter.setRenderHint(QPainter::Antialiasing, false);

	painter.setBrush( painter.background() );
	painter.setPen( Qt::NoPen );

	painter.drawRect( 0, 0, width(), height() );

	painter.setBrush( Qt::NoBrush );
	painter.setPen( QPen( QColor::fromRgb(0,0,0)) );

	painter.setRenderHint(QPainter::Antialiasing, true);

	CenterX = (float)(width() / 2);
	CenterY = (float)(height() / 2);
	DrawScale = ViewScale * ((float)width()/ 560.0f);

	if(bQuat2Valid) {
		DrawShape( painter, mat2, QColor::fromRgb(192,192,192), QuadPt, QuadLine );
	}
	DrawShape( painter, mat, QColor::fromRgb(0,0,0), QuadPt, QuadLine );
}

void Orientation_Widget::DrawShape( QPainter &painter, QMatrix4x4 &m, QColor col, QVector<QVector3D> &points, QVector<int> &lines )
{
	for(int i = 0; i < points.length(); i++) {
		pt[i] = m * points[i];
	}

	for(int i = 0; i < points.length(); i++)
	{
		pt[i].setZ( pt[i].z() + ViewDist );

		if(pt[i].z() == 0.0f) {
			pt[i].setZ( 0.0001f );
		}

		pt[i].setX( pt[i].x() / pt[i].z() );
		pt[i].setY( pt[i].y() / pt[i].z() );
	}

	QPen penCol(col);
	painter.setPen( penCol );

	QPointF cb[2];

	for(int i = 0; i < lines.length(); i += 2)
	{
		cb[0].setX( pt[lines[i]].x() );
		cb[0].setY( pt[lines[i]].y() );
		cb[1].setX( pt[lines[i + 1]].x() );
		cb[1].setY( pt[lines[i + 1]].y() );

		cb[0].setX( cb[0].x() *  DrawScale + CenterX );
		cb[0].setY( cb[0].y() * -DrawScale + CenterY );
		cb[1].setX( cb[1].x() *  DrawScale + CenterX );
		cb[1].setY( cb[1].y() * -DrawScale + CenterY );

		painter.drawLine( cb[0], cb[1] );
	}
}
