#include "cartpathsetter.h"
#include "rospublisher.h"
#include "geometry_msgs/Pose.h"

#include <CartConrolPlugin/PathSrv.h>
#include <algorithm>
#include <QDebug>
#include <iostream>

CartPathSetter::CartPathSetter(std::shared_ptr<RosPublisher> pub ,QGraphicsView *parent)
 : CartPathAbstract(parent)
{
    this->pub = pub;
}

void CartPathSetter::mousePressEvent(QMouseEvent *e)
{
    // If left button click
    if(e->button() == Qt::LeftButton)
    {
        // Add new point to path
        auto pt = mapToScene(e->pos());
        this->path.push_back(pt);

        // Draw all path
        if ( path.size() == 1){
            this->scene->addEllipse(QRectF(pt.x(),pt.y(),1,1));
        }
        else
        {
            auto r = this->path.rbegin();
            this->scene->addLine(QLineF(r[0],r[1]));
        }
    }
    // If right button click - clear scene
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


   // Connect to  pathSrv service
   auto n = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
   ros::ServiceClient client = n->serviceClient<CartControlPlugin::PathSrv>("pathSrv");
   CartControlPlugin::PathSrv srv;

   // Copy path to request
   geometry_msgs::Pose pose;
   for( auto point : path)
   {
     pose.position.x = point.x();
     pose.position.y = point.y();
     srv.request.path.push_back(pose);
   }

   // Get answer and log it
   if (client.call(srv))
   {
      ROS_INFO("Request to pthSrv succeded");
      pub->setVelocity(QPointF(0.0,0.0));
   }
   else
   {
     ROS_INFO("Can't do request to pthSrv");
   }

  // Clear scene
  this->clearScene();
}

void CartPathSetter::clearScene()
{
    this->path.clear();
    this->scene->clear();
}

