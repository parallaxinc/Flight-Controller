#include "linefit_widget.h"
#include <QPainter>
#include <QBitmap>


//! [0]
LineFit_Widget::LineFit_Widget(QWidget * parent) : QWidget(parent)
{
	xmin = -200.0f;
	xmax =  500.0f;
	ymin = -1000.0f;
	ymax =  2000.0f;

	samplesUsed = sampleIndex = 0;
	dSlope.x = dSlope.y = dSlope.z = dSlope.t = 1.0;

	rPen = QPen(QColor::fromRgb(255, 0, 0, 128));
	gPen = QPen(QColor::fromRgb(0, 255, 0, 128));
	bPen = QPen(QColor::fromRgb(0, 0, 224, 128));

	lrPen = QPen(QColor::fromRgb(255, 128, 128));
	lgPen = QPen(QColor::fromRgb(128, 255, 128));
	lbPen = QPen(QColor::fromRgb(128, 128, 224));

	lrPen.setWidth(2);
	lgPen.setWidth(2);
	lbPen.setWidth(2);
}
//! [0]


void LineFit_Widget::Reset(void)
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

void LineFit_Widget::AddSample( LFSample &newSample , bool bRedraw )
{
	samples[ sampleIndex ] = newSample;
	sampleIndex = (sampleIndex+1) & 4095;
	if( samplesUsed < 4096 ) samplesUsed++;

	if(bRedraw) {
		ComputeLine();
		update();
	}
}


void LineFit_Widget::paintEvent(QPaintEvent * event)
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

	if(samplesUsed < 10) return;

	QPointF pt;

	for( int i = 0; i < samplesUsed; i++ )
	{
		float y, x = (samples[i].t - xmin) * xs;

		y = height() - (samples[i].x - ymin) * ys;
		p.setPen( lrPen );
		p.drawPoint(x, y);

		y = height() - (samples[i].y - ymin) * ys;
		p.setPen( lgPen );
		p.drawPoint(x, y);

		y = height() - (samples[i].z - ymin) * ys;
		p.setPen( lbPen );
		p.drawPoint(x, y);
	}

	p.setRenderHint(QPainter::Antialiasing, true);

	double sy, sx = xmin;
	double ey, ex = xmax;

	sy = dSlope.x * sx + dIntercept.x;
	ey = dSlope.x * ex + dIntercept.x;
	DrawInterceptLine( p, (float)sx, (float)sy, (float)ex, (float)ey, xs, ys, rPen );

	sy = dSlope.y * sx + dIntercept.y;
	ey = dSlope.y * ex + dIntercept.y;
	DrawInterceptLine( p, (float)sx, (float)sy, (float)ex, (float)ey, xs, ys, gPen );

	sy = dSlope.z * sx + dIntercept.z;
	ey = dSlope.z * ex + dIntercept.z;
	DrawInterceptLine( p, (float)sx, (float)sy, (float)ex, (float)ey, xs, ys, bPen );
}

void LineFit_Widget::DrawInterceptLine( QPainter &p, float sx, float sy, float ex, float ey, float xs, float ys, QPen & pen )
{
	sx = (sx - xmin) * xs;
	sy = height() - (sy - ymin) * ys;

	ex = (ex - xmin) * xs;
	ey = height() - (ey - ymin) * ys;

	QPointF p1(sx, sy);
	QPointF p2(ex, ey);

	p.setPen( pen );
	p.drawLine( p1, p2 );
}


void LineFit_Widget::ComputeLine(void)
{
	LFSampleD s, st;
	//LFSample minS, maxS;

	//xMin = yMin = 1000000.0f;
	//xMax = yMax = -1000000.0f;

	//if(samplesUsed > 0) {
	//	minS = samples[0];
	//	maxS = samples[0];
	//}

	s.x = s.y = s.z = s.t = 0.0;
	st.t = st.x = st.y = st.z = 1.0;

	for( int i = 0; i < samplesUsed; ++i )
	{
		s.t += samples[i].t;
		s.x += samples[i].x;
		s.y += samples[i].y;
		s.z += samples[i].z;

		//if(samples[i].t < minS.t) minS = samples[i];
		//if(samples[i].t > maxS.t) maxS = samples[i];

		/*	this code is only if you want the graph to auto-range
		xMin = Math.Min( xMin, Samples[i].t );
		xMax = Math.Max( xMax, Samples[i].t );
		yMin = Math.Min( yMin, Samples[i].x );
		yMax = Math.Max( yMax, Samples[i].x );
		yMin = Math.Min( yMin, Samples[i].y );
		yMax = Math.Max( yMax, Samples[i].y );
		yMin = Math.Min( yMin, Samples[i].z );
		yMax = Math.Max( yMax, Samples[i].z );
		//*/
	}

	for( int i = 0; i < samplesUsed; ++i )
	{
		double t = samples[i].t - s.t / (double)samplesUsed;
		st.t += t * t;
		st.x += t * samples[i].x;
		st.y += t * samples[i].y;
		st.z += t * samples[i].z;
	}

	dSlope.x = st.x / st.t;
	dSlope.y = st.y / st.t;
	dSlope.z = st.z / st.t;

	dIntercept.x = (s.x - s.t * dSlope.x) / (double)samplesUsed;
	dIntercept.y = (s.y - s.t * dSlope.y) / (double)samplesUsed;
	dIntercept.z = (s.z - s.t * dSlope.z) / (double)samplesUsed;
}
