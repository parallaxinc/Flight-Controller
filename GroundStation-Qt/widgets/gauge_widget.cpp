#include "gauge_widget.h"
#include <QPainter>
#include <QBitmap>
#include <math.h>


//! [0]
Gauge_Widget::Gauge_Widget(QWidget * parent) : QWidget(parent)
{
	range = 100.0f;
	value = 0.0f;
	gaugeCircle = 0.80f;	// 80% of a circle is full-range for a standard gauge, set to 1.0 to use the entire angular range

	displayOffset = 0.0f;
	displayScale = 1.0f;

	avg.Init(128);
}
//! [0]


//! [1]
QSize Gauge_Widget::minimumSizeHint() const
{
    return QSize(30, 30);
}
//! [1]


//! [2]
QSize Gauge_Widget::sizeHint() const
{
    return QSize(70, 70);
}
//! [2]

void Gauge_Widget::paintEvent(QPaintEvent * event)
{
    (void)event;

	// Compute the angle for the gauge based on the value and current range
	float temp;
	if(value <= -range) {
		temp = -range;
	}
	else if(value >= range) {
		temp = range;
	}
	else {
		temp = value;
	}

	float angle = (temp / range) * 3.141592654f * gaugeCircle;

	float centerX = (float)width() * 0.5f;
	float centerY = (float)height() * 0.5f;
	float radius = qMin( centerX, centerY );

	int outerDiam = qMin( width(), height() ) - 2;
	int offsX = 1 + (width() - outerDiam) / 2;
	int offsY = 1 + (height() - outerDiam) / 2;

	QPainter p(this);

	p.setRenderHint(QPainter::Antialiasing, false);

	QColor foreCol = p.background().color();
	foreCol = foreCol.darker(106);

	p.setBrush( p.background() );
	p.setPen( Qt::NoPen );
	p.drawRect( 0, 0, width(), height() );

	p.setBrush( QBrush(foreCol) );
	p.setPen( QPen(QColor::fromRgb(255,255,255)) );
	p.drawEllipse( offsX, offsY, outerDiam, outerDiam );

	p.setRenderHint(QPainter::Antialiasing, true);



	// Paint the gauge
	float endX = centerX + sinf( angle ) * radius;
	float endY = centerY - cosf( angle ) * radius;

	//g.FillEllipse( SystemBrushes.ControlLight, this.ClientRectangle );
	//g.DrawEllipse( Pens.White, this.ClientRectangle );

	p.setPen( QPen(QColor::fromRgb(0,0,0)) );
	QPointF p1(centerX, centerY);
	QPointF p2(endX, endY);
	p.drawLine( p1, p2 );


	temp = value * displayScale + displayOffset;
	QString s;
	if( displayScale != 1.f ) {
		s = QString::number(temp, 'f', 1) + displayPostfix;
	}
	else {
		s = QString::number(temp, 'f', 1) + displayPostfix;
	}

	p.setFont( font() );
	p.drawText( 10, 1, width()-20, height()-8, Qt::AlignLeft | Qt::AlignBottom, s );

	s = QString::number(avg.Value(), 'f', 1);
	p.drawText( 10, 1, width()-20, height()-8, Qt::AlignRight | Qt::AlignBottom, s );
}
