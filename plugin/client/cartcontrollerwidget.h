#ifndef CARTCONTROLLERWIDGET_H
#define CARTCONTROLLERWIDGET_H

#include "commonheader.h"
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
    explicit CartControllerWidget(std::shared_ptr<RosPublisher> rosPubPtr, QGraphicsView *parent = nullptr);

signals:

public slots:
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
private:
    QPointF posToVector(QPointF p);
    QGraphicsScene * scene;
    std::shared_ptr<RosPublisher> pub;
    bool mousePressed;
};

#endif // CARTCONTROLLERWIDGET_H
