#include "angle_widget.h"
#include <QPainter>
#include <QBitmap>


//! [0]
Angle_Widget::Angle_Widget(QWidget * parent) : QWidget(parent)
{
	backImage.load(":/images/heading_background.png");
    needleImage.load(":/images/altimeter_longneedle.png");

    QBitmap mask;

    mask = backImage.createMaskFromColor( QColor(255, 255, 0) );
    backImage.setMask( mask );

    mask = needleImage.createMaskFromColor( QColor(255, 255, 0) );
    needleImage.setMask( mask );

    rotAngle = 0;
}
//! [0]


//! [1]
QSize Angle_Widget::minimumSizeHint() const
{
    return QSize(30, 30);
}
//! [1]


//! [2]
QSize Angle_Widget::sizeHint() const
{
    return QSize(70, 70);
}
//! [2]

void Angle_Widget::setAngle(float f)
{
    rotAngle = f;
    update();
}


void Angle_Widget::paintEvent(QPaintEvent * event)
{
    (void)event;

    QPainter painter(this);
    //painter.setRenderHint(QPainter::Antialiasing, true);
    //painter.setRenderHint(QPainter::HighQualityAntialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    float xScale = (float)this->width() / (float)backImage.width();
    float yScale = (float)this->height() / (float)backImage.height();

    painter.scale(xScale, yScale);
    painter.drawPixmap(0, 0, backImage);
    painter.scale(1,1);

    QTransform trans;
    trans.translate( width()/2, height()/2);
    trans.scale(xScale,yScale);
    trans.rotate( rotAngle );
    trans.translate( -14,  -111 );

    painter.setTransform(trans);
    painter.drawPixmap(0, 0, needleImage);
}
