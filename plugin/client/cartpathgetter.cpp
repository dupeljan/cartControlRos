#include "cartpathgetter.h"
#include <algorithm>
#include <binders.h>

#include <QDebug>

CartPathGetter::CartPathGetter(QGraphicsView *parent)
    : CartPathAbstract(parent)
{
    // Set collors
    robotPen =QPen(QBrush(Qt::blue),1);
    pathPen = QPen();
}

void CartPathGetter::drawSimulationPos(QPointF point)
{
    auto trPoint = translate(point);
    this->scene->addEllipse(QRectF(trPoint.x(),trPoint.y(),1,1),this->robotPen);
}

void CartPathGetter::drawAnaliticPath(std::vector<QPointF> v)
{
    // Path thransform
    this -> path = v;
    this->scene->clear();
    for(auto it = path.begin(); (it + 1) != path.end(); it++)    
        this->scene->addLine(QLineF(it[0],it[1]),this->pathPen);

}

QPointF CartPathGetter::translate(QPointF p)
{
    QPointF res;
    res.setX((p.x()+simulationStartPoint.x()) * width() / simulationFieldSize.width());
    res.setY((p.y()+simulationStartPoint.y()) * height() / simulationFieldSize.height());
    return res;
}
