#ifndef ANGLEWIDGET_H
#define ANGLEWIDGET_H

#include <QWidget>

class Angle_Widget : public QWidget
{
    Q_OBJECT

public:
    Angle_Widget(QWidget * parent = 0);

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

public slots:
    //void setShape(Shape shape);
    //void setPen(const QPen &pen);
    //void setBrush(const QBrush &brush);
    //void setAntialiased(bool antialiased);
    //void setTransformed(bool transformed);

    void setAngle(float f);

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
    float rotAngle;

    QPixmap backImage;
    QPixmap needleImage;
};

#endif // ANGLEWIDGET_H
