#ifndef VALUEBARWIDGET_H
#define VALUEBARWIDGET_H

#include <QWidget>
#include <QBitmap>
#include <QPen>


class ValueBar_Widget : public QWidget
{
    Q_OBJECT

public:
    ValueBar_Widget(QWidget * parent = 0);

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;

public slots:

    void setValue(float v);
    void setMinMax(float vMin, float vMax);
    void setLeftLabel(const QString str);
    void setLeftLabel(int labelVal);
    void setLeftLabel(const char * charStr);

    void setRightLabel(const QString str);
    void setRightLabel(int labelVal);
    void setRightLabel(const char * charStr);

    void setFromLeft(bool bVal);

    void setBarColor( const QColor & col );

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
    float valMin, valMax;
    float value;
    bool fromLeft;
    int buffer;

    QColor barColor;
    QBrush barBrush;

    QString leftLabel;
    QString rightLabel;

    QPen lightPen, grayPen, blackPen;
};

#endif // VALUEBARWIDGET_H
