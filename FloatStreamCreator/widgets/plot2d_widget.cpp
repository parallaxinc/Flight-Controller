#include "plot2d_widget.h"
#include <QPainter>
#include <QBitmap>


//! [0]
Plot2d_Widget::Plot2d_Widget(QWidget * parent) : QWidget(parent)
{
	xmin = -1000.0f;
	xmax =  1000.0f;
	ymin = -1000.0f;
	ymax =  1000.0f;

	samplesUsed = sampleIndex = 0;

	sPen = QPen(QColor::fromRgb(64, 64, 64, 192));
	sPen.setWidthF(2.0f);

	cPen = QPen(QColor::fromRgb(0, 0, 0, 255));
}
//! [0]


void Plot2d_Widget::Reset(void)
{
	samplesUsed = 0;
	sampleIndex = 0;
}

/*static int clamp(int v, int mn, int mx)
{
    if( v < mn ) return mn;
    if( v > mx ) return mx;
    return v;
}*/

void Plot2d_Widget::AddSample( QPointF &newSample , bool bRedraw )
{
	samples[ sampleIndex ] = newSample;
	sampleIndex = (sampleIndex+1) & 4095;
	if( samplesUsed < 4096 ) samplesUsed++;

	if(bRedraw) {
		update();
	}
}


void Plot2d_Widget::paintEvent(QPaintEvent * event)
{
    (void)event;

	QPainter p(this);

	float xs = (float)width() / (xmax - xmin);
	float ys = (float)height() / (ymax - ymin);

    // Draw the border and fill the background
	p.setRenderHint(QPainter::Antialiasing, false);

	p.setBrush( p.background() );
	p.setPen( Qt::NoPen );
	p.drawRect( 0, 0, width(), height() );
	p.setBrush(Qt::NoBrush);

	if(samplesUsed == 0) return;

	p.setRenderHint(QPainter::Antialiasing, true);


	float x1, y1, x2, y2;
	x1 = x2 = samples[0].x();
	y1 = y2 = samples[0].y();

	p.setPen( sPen );
	for( int i = 0; i < samplesUsed; i++ )
	{
		float x = (samples[i].x() - xmin) * xs;
		float y = (samples[i].y() - ymin) * ys;

		if( x < x1 ) x1 = x;
		if( x > x2 ) x2 = x;
		if( y < y1 ) y1 = y;
		if( y > y2 ) y2 = y;

		p.drawPoint(x, y);
	}

	float xc = (x1+x2)/2;
	float yc = (y1+y2)/2;

	QString s = QString::asprintf( "%0.1f, %0.1f", xc, yc );
	p.setRenderHint(QPainter::Antialiasing, false);
	p.setPen( sPen );
	p.setFont( font() );
	p.drawText( 10, 1, width()-10, height()-8, Qt::AlignLeft | Qt::AlignTop, s );
}
