#ifndef ALTIMETERWIDGET_H
#define ALTIMETERWIDGET_H

#include <QWidget>

class Altimeter_Widget : public QWidget
{
    Q_OBJECT

public:
    Altimeter_Widget(QWidget * parent = 0);

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

public slots:
    //void setShape(Shape shape);
    //void setPen(const QPen &pen);
    //void setBrush(const QBrush &brush);
    //void setAntialiased(bool antialiased);
    //void setTransformed(bool transformed);

    void setAltitude(float f);

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
	void ScrollCounter(QPainter & painter, float counterValue, float xOfs, float yOfs, float xScale, float yScale);

private:
    float altitude;

    QPixmap foreImage;
    QPixmap numbersImage;
    QPixmap smallNeedle;
    QPixmap largeNeedle;
};

#endif // ALTIMETERWIDGET_H
