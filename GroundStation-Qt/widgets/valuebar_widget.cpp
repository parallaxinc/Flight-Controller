#include "valuebar_widget.h"
#include <QPainter>
#include <QBitmap>


//! [0]
ValueBar_Widget::ValueBar_Widget(QWidget * parent) : QWidget(parent)
{
    valMin = -1024;
    valMax =  1024;
    value = 0;
	drawFrom = Left;
    buffer = 2;

    setBarColor( QColor::fromRgb(128, 255, 128) );

    lightPen = QColor::fromRgb(255,255,255);
    grayPen = QColor::fromRgb(160,160,160);
    blackPen = QColor::fromRgb(0,0,0);
}
//! [0]


//! [1]
QSize ValueBar_Widget::minimumSizeHint() const
{
    return QSize(30, 10);
}
//! [1]


//! [2]
QSize ValueBar_Widget::sizeHint() const
{
	return QSize(60, 10);
}
//! [2]

void ValueBar_Widget::setValue(float v)
{
    if( value != v ) {
        value = v;
        update();
    }
}

void ValueBar_Widget::setMinMax(float vMin, float vMax)
{
    valMin = vMin;
    valMax = vMax;
    update();
}

void ValueBar_Widget::setLeftLabel(const QString str)
{
    if( leftLabel.compare( str ) ) {
        leftLabel = str;
        update();
    }
}

void ValueBar_Widget::setRightLabel(const QString str)
{
    if( rightLabel.compare( str ) ) {
        rightLabel = str;
        update();
    }
}

void ValueBar_Widget::setLeftLabel(int labelVal)
{
    setLeftLabel( QString::number(labelVal) );
}

void ValueBar_Widget::setLeftLabel(const char * charStr)
{
    setLeftLabel( QString(charStr) );
}

void ValueBar_Widget::setRightLabel(int labelVal)
{
    setRightLabel( QString::number(labelVal) );
}

void ValueBar_Widget::setRightLabel(const char * charStr)
{
    setRightLabel( QString(charStr) );
}

void ValueBar_Widget::setOrigin(Origin org)
{
	drawFrom = org;
    update();
}

void ValueBar_Widget::setBarColor( const QColor & col )
{
    barColor = col;
    barBrush = QBrush(barColor);
}

static int clamp(int v, int mn, int mx)
{
    if( v < mn ) return mn;
    if( v > mx ) return mx;
    return v;
}

static float FAbs( float f )
{
	return f < 0.0f ? -f : f;
}


void ValueBar_Widget::paintEvent(QPaintEvent * event)
{
    (void)event;

    QPainter painter(this);

    int scale = width() - (buffer * 2);
    float l;

    int clampedVal = clamp( value, valMin, valMax );

    float fHeight = height();
    float fWidth = (clampedVal - valMin) * scale / (valMax - valMin);

	// Draw the border and fill the background
	painter.setRenderHint(QPainter::Antialiasing, false);

	painter.setBrush( painter.background() );
	painter.setPen( lightPen );
	painter.drawRect( 1, 1, width()-2, height()-2 );
	painter.setBrush( Qt::NoBrush );
	painter.setPen( grayPen );
	painter.drawRect( 0, 0, width()-2, height()-2 );

	painter.setRenderHint(QPainter::Antialiasing, true);
	painter.setBrush( barBrush );
	painter.setPen( Qt::NoPen );

	// Draw the bar
	if( drawFrom == Left ) {
        l = buffer;
		painter.drawRect( l, buffer, fWidth-1, fHeight-(buffer*2)-1 );
	}
	else if( drawFrom == Right ) {
        l = (width()-buffer) - fWidth;
		painter.drawRect( l, buffer, fWidth-1, fHeight-(buffer*2)-1 );
	}
	else if( drawFrom == Center ) {

		fWidth = FAbs( clampedVal * scale / valMax ) * 0.5f;
		if( value < 0.0f ) {
			l = (int)((width()-buffer) * 0.5f) - fWidth;
		}
		else {
			l = (width()-buffer) * 0.5f;
		}
		painter.drawRect( l, buffer, fWidth-1, fHeight-(buffer*2)-1 );
	}

    painter.setPen( blackPen );
    painter.setFont( font() );

    // Draw the left label
    painter.drawText( buffer+2, 0, width()-(buffer+2)*2, height()-2, Qt::AlignLeft | Qt::AlignVCenter , leftLabel );

    // Draw the right label
    painter.drawText( buffer+2, 0, width()-(buffer+2)*2, height()-2, Qt::AlignRight | Qt::AlignVCenter , rightLabel );
}
