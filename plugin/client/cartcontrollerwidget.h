#ifndef CARTCONTROLLERWIDGET_H
#define CARTCONTROLLERWIDGET_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QPoint>
#include "rospublisher.h"

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
    QPoint posToVector(QPointF p);
    QGraphicsScene * scene;
    RosPublisher pub;
    bool mousePressed;
};

#endif // CARTCONTROLLERWIDGET_H
