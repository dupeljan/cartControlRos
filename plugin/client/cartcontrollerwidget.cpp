#include "cartcontrollerwidget.h"
#include <QPointF>
#include <iostream>
#include <string>
#include <QDebug>
#include <math.h>

CartControllerWidget::CartControllerWidget(std::shared_ptr<RosPublisher> rosPubPtr, QGraphicsView *parent)
        : QGraphicsView(parent)
{
    // Setup publisher
    pub = std::shared_ptr<RosPublisher>(rosPubPtr);

    // Setup widget
    mousePressed = false;
    scene = new QGraphicsScene();
    this->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
    int r(300);
    this->resize(r,r);

    this->setSceneRect(0, 0, width(), width());
    this->setScene(scene);

    // Draw circle
    scene->addEllipse(QRectF(0,0,width(),width()));

    // Draw main point
    int shift = 10;
    scene->addEllipse(QRectF(width()/2-shift,height()/2-shift,
                             2*shift,2*shift));

    // Get central cicrle sqr radius
    this->centreCircleRadiusSqr = std::pow(shift,2);
}

void CartControllerWidget::mousePressEvent(QMouseEvent *e)
{
    mousePressed = true;
    sendSpeed(e);


#if DEBUG == 1
    qDebug((std::to_string(pt.x()) + ' ' + std::to_string(pt.y()) + '\n').c_str());
#endif
}

void CartControllerWidget::mouseReleaseEvent(QMouseEvent *e)
{
    mousePressed = false;
}

void CartControllerWidget::sendSpeed(QMouseEvent *e)
{

    QPointF pt = mapToScene(e->pos());
    auto pos = posToVector(pt);
    // Check if it in centre
    if(std::pow(pos.x(),2) + std::pow(pos.y(),2) < this->centreCircleRadiusSqr)
    {
        pos.setX(0);
        pos.setY(0);
    }
    pub->setVelocity(pos);
    #if DEBUG == 1
            qDebug((std::to_string(pt.x()) + ' ' + std::to_string(pt.y()) + '\n').c_str());
    #endif
}

void CartControllerWidget::mouseMoveEvent(QMouseEvent *e)
{
    if(mousePressed)
        sendSpeed(e);
}


QPointF CartControllerWidget::posToVector(QPointF p)
{
    return QPointF(p.x() - width() / 2, p.y() - height() / 2);
}

