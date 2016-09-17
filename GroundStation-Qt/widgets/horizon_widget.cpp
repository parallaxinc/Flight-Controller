#include "horizon_widget.h"
#include <QPainter>
#include <QBitmap>
#include <math.h>

//! [0]
Horizon_Widget::Horizon_Widget(QWidget * parent) : QWidget(parent)
{
    horizonImage.load(":/images/horizon_groundsky.png");
    foreImage.load(":/images/horizon_background.png");

    foreMask = foreImage.createMaskFromColor( QColor(255, 255, 0) );
    foreImage.setMask( foreMask );

    rollAngle = pitchAngle = 0;
}
//! [0]


//! [1]
QSize Horizon_Widget::minimumSizeHint() const
{
    return QSize(30, 30);
}
//! [1]


//! [2]
QSize Horizon_Widget::sizeHint() const
{
    return QSize(70, 70);
}
//! [2]

void Horizon_Widget::setAngles(float roll, float pitch)
{
    roll = roundf(roll);
    pitch = roundf(pitch);
    if( rollAngle != roll || pitchAngle != pitch ) {
        rollAngle = roll;
        pitchAngle = pitch;
        update();
    }
}


void Horizon_Widget::paintEvent(QPaintEvent * event)
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

	float xScale = (float)drawSize / (float)foreImage.width();
	float yScale = (float)drawSize / (float)foreImage.height();

	painter.setClipRect( CenterX - drawSize/2 + 10, CenterY - drawSize/2 + 10, drawSize - 20, drawSize - 20 );
	painter.setClipping(true);

    QTransform trans;
	trans.translate( xOffset + 150*xScale, yOffset + 150*yScale );
	trans.scale(xScale, yScale);
    trans.rotate( rollAngle );
	trans.translate( -125,  -360 + (pitchAngle*4) );

    painter.setTransform(trans);
    painter.drawPixmap(0, 0, horizonImage);

    painter.setClipping(false);
    painter.resetTransform();
	painter.translate( xOffset, yOffset );
    painter.scale(xScale, yScale);
    painter.drawPixmap(0, 0, foreImage);
}
