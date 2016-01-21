#ifndef HORIZONWIDGET_H
#define HORIZONWIDGET_H

#include <QWidget>
#include <QBitmap>


class Horizon_Widget : public QWidget
{
    Q_OBJECT

public:
    Horizon_Widget(QWidget * parent = 0);

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

public slots:
    //void setShape(Shape shape);
    //void setPen(const QPen &pen);
    //void setBrush(const QBrush &brush);
    //void setAntialiased(bool antialiased);
    //void setTransformed(bool transformed);

    void setAngles(float roll, float pitch);

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
    float rollAngle, pitchAngle;

    QPixmap horizonImage;
    QPixmap foreImage;

    QBitmap foreMask;
};

#endif // HORIZONWIDGET_H
