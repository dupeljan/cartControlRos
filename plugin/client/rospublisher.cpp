#include "rospublisher.h"
#include <QDebug>
#include <functional>

RosPublisher::RosPublisher()
{
    // Init ros
    int argc = 0;
    char **argv = NULL;
    ros::init(argc,argv,"talker");
    // Create node handler
    node = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    // Setup topic name
    topicName = "/robot_mobile_wheel_3_omni_open_base/velocity";
    // Setup publisher
    VelocityPub = node->advertise<CartConrolPlugin::Velocity>(topicName,1000);
    // Setup loop rate
    loopRate = std::unique_ptr<ros::Rate>(new ros::Rate(10));
    // Setup velocity
    velocity.back = 0.0f;
    velocity.left = 0.0f;
    velocity.right = 0.0f;
    // run publisher
    auto x = std::thread(std::bind(&RosPublisher::sendRoutine,this));
}

RosPublisher::~RosPublisher()
{
    // Stop sendRoutine
    node->shutdown();
}

std::string RosPublisher::velocityToString(const CartConrolPlugin::Velocity v){
    return  "left " + std::to_string(v.left) +'\n' +
            "right " + std::to_string(v.right) + '\n' +
            "back " + std::to_string(v.back) + '\n';
}

void RosPublisher::sendRoutine()
{
    // Shut down when node shut down
    while (ros::ok())
         {

          // Access to shared velocity
          const std::lock_guard<std::mutex> lock(velocityMutex);
          qDebug((velocityToString(velocity) + "-----------\n").c_str());

          // Send velocity to the topic
          VelocityPub.publish(velocity);

          // Iterate
          ros::spinOnce();
          loopRate->sleep();
         }
}

void RosPublisher::setVelocity(CartConrolPlugin::Velocity v)
{
    // Access to shared velocity
    const std::lock_guard<std::mutex> lock(velocityMutex);
    // Set it
    velocity = v;

}

void RosPublisher::setVelocity(QPoint p)
{
    CartConrolPlugin::Velocity v;
    float s;
    if(p.x() > 0)
        s = 10.0;
    else
        s = -10.0;
    setVelocity(v);
}
