#include "cartpathgetter.h"
#include <algorithm>
#include <binders.h>

#include <QDebug>

CartPathGetter::CartPathGetter(QGraphicsView *parent)
    : CartPathAbstract(parent)
{
    robotBrush.setColor(Qt::blue);
}

void CartPathGetter::drawSimulationPos(QPointF point)
{
    auto trPoint = translate(point);
    this->scene->addEllipse(QRectF(trPoint.x(),trPoint.y(),1,1),QPen(),robotBrush);
}

void CartPathGetter::drawAnaliticPath(std::vector<QPointF> v)
{
    // Path thransform
    /*
    std::transform(v.begin(),v.end(),this->path.begin(),
                   [this](QPointF p)
    {
        return this->translate(p);
    });
    */
    this -> path = v;
    this->scene->clear();
    for(auto it = path.begin(); (it + 1) != path.end(); it++)    
        this->scene->addLine(QLineF(it[0],it[1]));

}

QPointF CartPathGetter::translate(QPointF p)
{
    QPointF res;
    res.setX((p.x()+simulationStartPoint.x()) * width() / simulationFieldSize.width());
    res.setY((p.y()+simulationStartPoint.y()) * height() / simulationFieldSize.height());
    return res;
}
