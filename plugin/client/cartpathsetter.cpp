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
  CartConrolPlugin::VelocityWheels v;
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

// Send path to cart
void CartPathSetter::sendPathRoutine(CartConrolPlugin::PathMsg msg)
{
    // Prepare node
    auto n = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle);
    ros::Publisher pathPub = n->advertise<CartConrolPlugin::PathMsg>(this->topicName,1);

    // Subscribe to status
    // Create shared future and promise
    auto promise = std::shared_ptr<std::promise<void>>(new std::promise<void>);
    auto futureObj = std::shared_ptr<std::shared_future<void>>(new std::shared_future<void>(promise->get_future()));
    // Init status checker
    StatusChecker stCheck(n,statusTopicName);
    // Run subscribe spin
    stCheck.run(promise,futureObj);
    auto rate = ros::Rate(10);

    // Publish while promise doesn't set it's value
    do
    {
        rate.sleep();
        pathPub.publish(msg);
    }while(futureObj->wait_for(std::chrono::microseconds(1)) == std::future_status::timeout);

    // send zero velocity
    CartConrolPlugin::VelocityWheels v;
    v.back = 0.0;
    v.left = 0.0;
    v.right = 0.0;
    v.stop = false;
    this->pub->setVelocity(v);
    // Release
     pathPub.shutdown();
     n->shutdown();
}

// Status subsriber loop
void CartPathSetter::StatusChecker::runThread(std::shared_ptr<std::promise<void> > p,std::shared_ptr<std::shared_future<void>> f)
{
    auto statusPathSub = n->subscribe<std_msgs::Bool>
            (topicName, 1000,
             boost::bind(&StatusChecker::statusSubCallback,this,
                       p, _1));
    auto loopRate = ros::Rate(1000);

    // While promise doesn't set it's value
    while(f->wait_for(std::chrono::microseconds(1)) == std::future_status::timeout)
    {
        ros::spinOnce();
        loopRate.sleep();
    }


}

CartPathSetter::StatusChecker::StatusChecker(std::shared_ptr<ros::NodeHandle> n, std::string topicName)
{
    this->n = std::shared_ptr<ros::NodeHandle>(n);
    this->topicName = topicName;
}

// Run runthread in new thread
void CartPathSetter::StatusChecker::run(std::shared_ptr<std::promise<void> > p, std::shared_ptr<std::shared_future<void>> f)
{
   auto th  =
            std::thread(std::bind(&StatusChecker::runThread,this, p, f ));
   th.detach();
}

void CartPathSetter::StatusChecker::statusSubCallback(std::shared_ptr<std::promise<void>> p, const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data)
        p->set_value();
}
