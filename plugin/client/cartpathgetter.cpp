#include "cartpathgetter.h"
#include <algorithm>
#include <binders.h>

CartPathGetter::CartPathGetter(QGraphicsView *parent)
    : CartPathAbstract(parent)
{

}

void CartPathGetter::drawSimulationPos(QPointF point)
{

}

void CartPathGetter::drawAnaliticPath(std::vector<QPointF> v)
{
    std::transform(v.begin(),v.end(),this->path.begin(),
                   [this](QPointF p)
    {
        return this->translate(p);
    });
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
