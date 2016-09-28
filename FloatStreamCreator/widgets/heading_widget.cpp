#include "heading_widget.h"
#include <QPainter>
#include <QBitmap>
#include <math.h>

//! [0]
Heading_Widget::Heading_Widget(QWidget * parent) : QWidget(parent)
{
    backImage.load(":/images/heading_background.png");
    headingRing.load(":/images/heading_wheel.png");
    airplaneImage.load(":/images/heading_aircraft.png");

    QBitmap mask;

    mask = backImage.createMaskFromColor( QColor(255, 255, 0) );
    backImage.setMask( mask );

    mask = headingRing.createMaskFromColor( QColor(255, 255, 0) );
    headingRing.setMask( mask );

    mask = airplaneImage.createMaskFromColor( QColor(255, 255, 0) );
    airplaneImage.setMask( mask );

    headingAngle = 0;
}
//! [0]


//! [1]
QSize Heading_Widget::minimumSizeHint() const
{
    return QSize(30, 30);
}
//! [1]


//! [2]
QSize Heading_Widget::sizeHint() const
{
    return QSize(70, 70);
}
//! [2]

void Heading_Widget::setHeading(float f)
{
    f = roundf(f);
    if( headingAngle != f ) {
        headingAngle = f;
        update();
    }
}


void Heading_Widget::paintEvent(QPaintEvent * event)
{
    (void)event;

    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

	int controlWidth = this->width();
	int controlHeight = this->height();
	int CenterX = controlWidth / 2;
	int CenterY = controlHeight / 2;

	int drawSize = controlWidth < controlHeight ? controlWidth : controlHeight;
	int xOffset = CenterX - (drawSize/2);
	int yOffset = CenterY - (drawSize/2);

	float xScale = (float)drawSize / (float)backImage.width();
	float yScale = (float)drawSize / (float)backImage.height();

	painter.translate(xOffset, yOffset);
	painter.scale(xScale, yScale);
    painter.drawPixmap(0, 0, backImage);

    QTransform trans;
	trans.translate( CenterX, CenterY );
	trans.scale(xScale, yScale);
    trans.rotate( -headingAngle );
    trans.translate( -137,  -137 );

    painter.setTransform(trans);
    painter.drawPixmap(0, 0, headingRing);

    painter.resetTransform();
	painter.translate(xOffset, yOffset);
	painter.scale(xScale, yScale);
    painter.drawPixmap(70, 40, airplaneImage);
}
