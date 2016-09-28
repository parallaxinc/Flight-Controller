#ifndef RADIOSTICKWIDGET_H
#define RADIOSTICKWIDGET_H

#include <QWidget>
#include <QBitmap>
#include <QPen>


class RadioStick_Widget : public QWidget
{
    Q_OBJECT

public:
    RadioStick_Widget(QWidget * parent = 0);

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

public slots:
    //void setShape(Shape shape);
    //void setPen(const QPen &pen);
    //void setBrush(const QBrush &brush);
    //void setAntialiased(bool antialiased);
    //void setTransformed(bool transformed);

    void setValues(float x, float y);

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
    float xValue, yValue;

    QPixmap backImage;
    QPen   stickPen;
    QBrush tipBrush;
};

#endif // ANGLEWIDGET_H
