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
    //Fetch std::future object associated with promise
    exitSignal = std::unique_ptr<std::promise<void>>(new std::promise<void>());
    futureObj = exitSignal->get_future();
    // run publisher
    velocityPubThread = std::thread(&RosPublisher::sendRoutine,this,std::move(futureObj));
}

RosPublisher::~RosPublisher()
{
    // Stop sendRoutine
    exitSignal->set_value();
    velocityPubThread.join();
}

std::string RosPublisher::velocityToString(const CartConrolPlugin::Velocity v){
    return  "left " + std::to_string(v.left) +'\n' +
            "right " + std::to_string(v.right) + '\n' +
            "back " + std::to_string(v.back) + '\n';
}

void RosPublisher::sendRoutine(std::future<void> futureObj)
{

    while (futureObj.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout)
         {
          // Access to shared velocity
          //const std::lock_guard<std::mutex> lock(velocityMutex);
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
    //const std::lock_guard<std::mutex> lock(velocityMutex);
    //Set the value in promise
    exitSignal->set_value();
    velocityPubThread.join();
    // Set it
    velocity = v;
    // start again
    exitSignal = std::unique_ptr<std::promise<void>>(new std::promise<void>());
    futureObj = exitSignal->get_future();
    velocityPubThread = std::thread(&RosPublisher::sendRoutine,this,std::move(futureObj));
}

void RosPublisher::setVelocity(QPoint p)
{
    CartConrolPlugin::Velocity v;
    float s;
    if(p.x() > 0)
        s = 10.0;
    else
        s = -10.0;
    v.back = s;
    v.left = s;
    v.right = s;
    setVelocity(v);
}
