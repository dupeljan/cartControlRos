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
  this->statusTopicName = "/robot_mobile_wheel_3_omni_open_base/status_path";
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
#if DEBUG == 1
  for(auto point : path)
      qDebug((std::to_string( point.x() ) + ' ' + std::to_string( point.y() ) + '\n').c_str());
#endif
  // send it to robot
  // stop joystic control
  CartConrolPlugin::Velocity v;
  v.left = 0.0;
  v.right = 0.0;
  v.back = 0.0;
  v.stop = true;
  this->pub->setVelocity(v);

  // Prepare path
  CartConrolPlugin::PathMsg msg;
  geometry_msgs::Pose pose;
  for( auto point : path)
  {
    pose.position.x = point.x();
    pose.position.y = point.y();
    msg.path.push_back(pose);
  }

  this->pub->pause();

  auto th =
        std::thread(std::bind(&CartPathSetter::sendPathRoutine, this,msg));
  th.detach();

  this->pub->resume();
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
    ros::Publisher pathPub = n.advertise<CartConrolPlugin::PathMsg>(this->topicName,1);

    // Create promise and futute
    auto exitSignal = std::unique_ptr<std::promise<void>>(new std::promise<void>());
    auto futureObj = exitSignal->get_future();
    // Subscribe to status
    ros::Subscriber statusPathSub = n.subscribe<std_msgs::Bool>
            ("Path_status_reader",1000,std::bind(&statusSubCallback,this,exitSignal.get()));
    auto rate = ros::Rate(10);
    /*qDebug(("Before" + std::to_string(pathPub.getNumSubscribers())).c_str());
    do
    {
        rate.sleep();
    }while(!pathPub.getNumSubscribers())
     qDebug(("After" + std::to_string(pathPub.getNumSubscribers())).c_str());
     */   // Send it
    do
    {
        rate.sleep();
        pathPub.publish(msg);
    }while(futureObj.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout);

     pathPub.shutdown();
}

void CartPathSetter::statusSubCallback(std::promise<void> p, const std_msgs::Bool_::ConstPtr &msg)
{
    if(msg.data)
        p.set_value();
}
