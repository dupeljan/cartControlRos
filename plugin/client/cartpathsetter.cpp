#include "cartpathsetter.h"
#include "rospublisher.h"
#include "geometry_msgs/Pose.h"

#include <CartConrolPlugin/PathSrv.h>
#include <algorithm>
#include <QDebug>
#include <iostream>

CartPathSetter::CartPathSetter( QGraphicsView *parent)
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
  // Send path to getter
  emit pathChosen(path);
  // Translate plot coords to simulation coords
  std::transform(path.begin(),path.end(),path.begin(),
                 [this](QPointF p){
    QPointF b;
    b.setX((simulationFieldSize.width() * p.x())/width() - simulationStartPoint.x());
    b.setY((simulationFieldSize.height() * p.y())/height() - simulationStartPoint.y());
    return b;
  });
#if DEBUG == 1
  for(auto point : path)
      qDebug((std::to_string( point.x() ) + ' ' + std::to_string( point.y() ) + '\n').c_str());
#endif
  // send it to robot
  // stop joystic control




   auto n = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
   ros::ServiceClient client = n->serviceClient<CartConrolPlugin::PathSrv>("pathSrv");
   CartConrolPlugin::PathSrv srv;

   // Copy path to request
   geometry_msgs::Pose pose;
   for( auto point : path)
   {
     pose.position.x = point.x();
     pose.position.y = point.y();
     srv.request.path.push_back(pose);
   }

   if (client.call(srv))
   {
      ROS_INFO("Request to pthSrv succeded");
   }
   else
   {
     ROS_INFO("Can't do request to pthSrv");
   }
  /*
  auto th =
        std::thread(std::bind(&CartPathSetter::sendPathRoutine, this,msg));
  th.detach();
*/
  // Clear scene
  this->clearScene();
}

void CartPathSetter::clearScene()
{
    this->path.clear();
    this->scene->clear();
}

