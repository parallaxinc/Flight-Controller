#ifndef GAUGEWIDGET_H
#define GAUGEWIDGET_H

#include <QWidget>
#include "movingaverage.h"

class Gauge_Widget : public QWidget
{
    Q_OBJECT

public:
	Gauge_Widget(QWidget * parent = 0);

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

	float getMovingAverage(void) const  {return avg.Value();}

public slots:
    //void setShape(Shape shape);
    //void setPen(const QPen &pen);
    //void setBrush(const QBrush &brush);
    //void setAntialiased(bool antialiased);
    //void setTransformed(bool transformed);

	void setDisplayOffset(float f)	{ displayOffset = f; update(); }
	void setDisplayScale(float f)	{ displayScale = f; update(); }
	void setDisplayPostfix( const char * pStr ) {displayPostfix = pStr; update(); }

	void setRange(float f) { range = f; update(); }
	void setWrap(bool wrap) {if(wrap) gaugeCircle = 1.0f; else gaugeCircle = 0.8f; update(); }

	void setValue(float f) {value = f; avg.AddSample(f); update(); }

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
	float range, value;
	float gaugeCircle;

	float displayOffset, displayScale;
	QString displayPostfix;

	MovingAverage avg;
};

#endif // GAUGEWIDGET_H
