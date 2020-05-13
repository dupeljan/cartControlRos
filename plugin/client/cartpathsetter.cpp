#include "cartpathsetter.h"
#include "rospublisher.h"
#include "geometry_msgs/Pose.h"

#include <algorithm>
#include <QDebug>
#include <iostream>

CartPathSetter::CartPathSetter(std::shared_ptr<RosPublisher> rosPubPtr, QGraphicsView *parent)
 : CartPathAbstract(parent)
{
  this->pub = std::shared_ptr<RosPublisher>(rosPubPtr);
  this->topicName = "/robot_mobile_wheel_3_omni_open_base/path";
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
  // send it to robot
  // stop joystic control
  CartConrolPlugin::Velocity v;
  v.left = 0.0;
  v.right = 0.0;
  v.back = 0.0;
  v.stop = true;
  this->pub->setVelocity(v);
  //this->pub->pause();

  // Prepare path
  CartConrolPlugin::PathMsg msg;
  geometry_msgs::Pose pose;
  for( auto point : path)
  {
    pose.position.x = point.x();
    pose.position.y = point.y();
    msg.path.push_back(pose);
  }

  auto th =
        std::thread(std::bind(&CartPathSetter::sendPathRoutine, this,msg));
  th.detach();

  //this->pub->resume();
  // Clear scene
  this->clearScene();
}

void CartPathSetter::clearScene()
{
    this->path.clear();
    this->scene->clear();
}

void CartPathSetter::sendPathRoutine(CartConrolPlugin::PathMsg msg)
{
    // Send Path
    // Prepare node
    ros::NodeHandle n;
    ros::Publisher pathPub = n.advertise<CartConrolPlugin::PathMsg>(this->topicName, 1000);


    auto rate = ros::Rate(1);
    while (!pathPub.getNumSubscribers())
    {
        rate.sleep();
    }
        // Send it
        pathPub.publish(msg);
}
