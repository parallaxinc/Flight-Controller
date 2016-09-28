#ifndef HEADINGWIDGET_H
#define HEADINGWIDGET_H

#include <QWidget>

class Heading_Widget : public QWidget
{
    Q_OBJECT

public:
    Heading_Widget(QWidget * parent = 0);

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

public slots:
    //void setShape(Shape shape);
    //void setPen(const QPen &pen);
    //void setBrush(const QBrush &brush);
    //void setAntialiased(bool antialiased);
    //void setTransformed(bool transformed);

    void setHeading(float f);

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
    float headingAngle;

    QPixmap backImage;
    QPixmap headingRing;
    QPixmap airplaneImage;
};

#endif // HEADINGWIDGET_H
