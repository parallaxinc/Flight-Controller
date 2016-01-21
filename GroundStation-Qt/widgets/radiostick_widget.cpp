#include "radiostick_widget.h"
#include <QPainter>
#include <QBitmap>


const int StickDiam = 30;


//! [0]
RadioStick_Widget::RadioStick_Widget(QWidget * parent) : QWidget(parent)
{
	backImage.load(":/images/radiostick_background.png");

	QBitmap mask = backImage.createMaskFromColor( QColor(255, 255, 0) );
	backImage.setMask( mask );

	stickPen.setWidth( StickDiam * 2 / 3 );
	stickPen.setCapStyle(Qt::RoundCap);
	stickPen.setColor(QColor::fromRgb(128, 128, 128));

	tipBrush = QBrush(QColor::fromRgb(160,160,160));

	xValue = yValue = 0;
}
//! [0]


//! [1]
QSize RadioStick_Widget::minimumSizeHint() const
{
    return QSize(30, 30);
}
//! [1]


//! [2]
QSize RadioStick_Widget::sizeHint() const
{
    return QSize(70, 70);
}
//! [2]

void RadioStick_Widget::setValues(float x, float y)
{
    if( xValue != x || yValue != y ) {
        xValue = x;
        yValue = y;
        update();
    }
}

void RadioStick_Widget::paintEvent(QPaintEvent * event)
{
    (void)event;

    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    float xScale = (float)this->width() / (float)backImage.width();
    float yScale = (float)this->height() / (float)backImage.height();

    painter.scale(xScale, yScale);
    painter.drawPixmap(0, 0, backImage);

    // display radio stick

    float centerX = (float)backImage.width() * 0.5f;
    float centerY = (float)backImage.height() * 0.5f;

    float radius = qMin( centerX, centerY ) * 0.6f;
    float range = 1024.0f;


    float xtemp, ytemp;
    if(xValue <= -range) {
        xtemp = -range;
    }
    else if(xValue >= range) {
        xtemp = range;
    }
    else {
        xtemp = xValue;
    }

    if(yValue <= -range) {
        ytemp = -range;
    }
    else if(yValue >= range) {
        ytemp = range;
    }
    else {
        ytemp = yValue;
    }

    xtemp = (xtemp / range) * radius;
    ytemp = (ytemp / range) * radius;

    float px = centerX + xtemp;
    float py = centerY - ytemp;		// Y is inverted when drawing

    painter.setRenderHint(QPainter::Antialiasing, true);
	painter.setPen( stickPen );

    painter.drawLine( centerX, centerY, px, py );

    int diam = StickDiam;
	painter.setBrush( tipBrush );
    painter.setPen( QPen(QColor::fromRgb(0,0,0)) );
    painter.drawEllipse( px - diam / 2, py - diam / 2, diam, diam );
    //pe.Graphics.DrawEllipse( Pens.Black, px - diam / 2, py - diam / 2, diam, diam );
}
