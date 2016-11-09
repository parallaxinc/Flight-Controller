#ifndef LINEFITWIDGET_H
#define LINEFITWIDGET_H

#include <QWidget>
#include <QPen>

struct LFSample {
	int	t, x, y, z;
};

struct LFSampleD {
	double	t, x, y, z;
};


class LineFit_Widget : public QWidget
{
    Q_OBJECT

public:
	LineFit_Widget(QWidget * parent = 0);

	//QSize minimumSizeHint() const Q_DECL_OVERRIDE;
	//QSize sizeHint() const Q_DECL_OVERRIDE;

	void Reset(void);
	void AddSample( LFSample &newSample , bool bRedraw );

	LFSampleD	dSlope;
	LFSampleD	dIntercept;

	double		noise;
	double		range;

public slots:

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
	void DrawInterceptLine( QPainter &p, float sx, float sy, float ex, float ey, float xs, float ys, QPen &pen );
	void ComputeLine(void);


	LFSample	samples[4096];
	int			samplesUsed, sampleIndex;
	float		xmin, xmax;
	float		ymin, ymax;

	QPen		rPen, gPen, bPen;
	QPen		lrPen, lgPen, lbPen;
};

#endif // LINEFITWIDGET_H
