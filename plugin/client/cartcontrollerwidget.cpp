#include "cartcontrollerwidget.h"
#include <QPointF>
#include <iostream>
#include <string>
#include <QDebug>


CartControllerWidget::CartControllerWidget(QGraphicsView *parent)
        : QGraphicsView(parent)
{
    // Setup publisher
    pub = std::unique_ptr<RosPublisher>(new RosPublisher());
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
}

void CartControllerWidget::mousePressEvent(QMouseEvent * e)
{
    mousePressed = true;
    double rad = 1;
    QPointF pt = mapToScene(e->pos());
#if DEBUG == 1
    qDebug((std::to_string(pt.x()) + ' ' + std::to_string(pt.y()) + '\n').c_str());
#endif
    // Send position to cart
    pub->setVelocity(posToVector(pt));
    //std::cout << std::to_string(pt.x()) << ' ' << std::to_string(pt.y()) << '\n';
    //scene->addEllipse(pt.x()-rad, pt.y()-rad, rad*2.0, rad*2.0,
    //QPen(), QBrush(Qt::SolidPattern));
}

void CartControllerWidget::mouseReleaseEvent(QMouseEvent *e)
{
    mousePressed = false;
}

void CartControllerWidget::mouseMoveEvent(QMouseEvent *e)
{
    if(mousePressed)
    {
        QPointF pt = mapToScene(e->pos());
#if DEBUG == 1
        qDebug((std::to_string(pt.x()) + ' ' + std::to_string(pt.y()) + '\n').c_str());
#endif
        pub->setVelocity(posToVector(pt));
    }
}


QPointF CartControllerWidget::posToVector(QPointF p)
{
    return QPointF(p.x() - width() / 2, p.y() - height() / 2);
}

