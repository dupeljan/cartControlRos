#include "cartpathsetter.h"
#include <algorithm>
#include <QDebug>
#include <iostream>

CartPathSetter::CartPathSetter(QGraphicsView *parent)
 : CartPathAbstract(parent)
{

}

void CartPathSetter::mousePressEvent(QMouseEvent *e)
{
    if(e->button() == Qt::LeftButton)
    {
        auto pt = mapToScene(e->pos());
        this->path.push_back(pt);
        if ( path.size() == 1){
            this->scene->addEllipse(QRectF(pt.x(),pt.y(),1,1));
        }
        else
        {
            auto r = this->path.rbegin();
            this->scene->addLine(QLineF(r[0],r[1]));
        }
    }
    else if(e->button() == Qt::RightButton)
        clearScene();
}

void CartPathSetter::mouseMoveEvent(QMouseEvent *e)
{

}

void CartPathSetter::mouseReleaseEvent(QMouseEvent *e)
{

}

void CartPathSetter::sendPath()
{

  // Translate plot coords to simulation coords
  std::transform(path.begin(),path.end(),path.begin(),
                 [this](QPointF p){
    QPointF b;
    b.setX((simulationFieldSize.width() * p.x())/width() - simulationStartPoint.x());
    b.setY((simulationFieldSize.height() * p.y())/height() - simulationStartPoint.y());
    return b;
  });
  for(auto point : path)
      qDebug((std::to_string( point.x() ) + ' ' + std::to_string( point.y() ) + '\n').c_str());

  this->clearScene();
}

void CartPathSetter::clearScene()
{
    this->path.clear();
    this->scene->clear();
}
