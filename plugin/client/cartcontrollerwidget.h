#ifndef CARTCONTROLLERWIDGET_H
#define CARTCONTROLLERWIDGET_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMouseEvent>

class CartControllerWidget : public QGraphicsView
{
    Q_OBJECT
public:
    explicit CartControllerWidget(QGraphicsView *parent = nullptr);

signals:

public slots:
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
private:
    QGraphicsScene * scene;
    bool mousePressed;
};

#endif // CARTCONTROLLERWIDGET_H
