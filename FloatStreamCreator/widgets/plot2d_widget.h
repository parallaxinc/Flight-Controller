#ifndef LINEFITWIDGET_H
#define LINEFITWIDGET_H

#include <QWidget>
#include <QPen>
#include <QPointF>


class Plot2d_Widget : public QWidget
{
    Q_OBJECT

public:
	Plot2d_Widget(QWidget * parent = 0);

	//QSize minimumSizeHint() const Q_DECL_OVERRIDE;
	//QSize sizeHint() const Q_DECL_OVERRIDE;

	void Reset(void);
	void AddSample( QPointF &newSample , bool bRedraw );

	bool		drawCircle;
	QPointF		center;
	float		radius;

	void SetCircle( float x, float y, float r ) {center.setX(x); center.setY(y); radius = r; drawCircle = true;}

public slots:

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:

	QPointF		samples[4096];
	int			samplesUsed, sampleIndex;
	float		xmin, xmax;
	float		ymin, ymax;

	QPen		sPen;	// samples
	QPen		tPen;	// text
	QPen		cPen;	// circle
};

#endif // LINEFITWIDGET_H
