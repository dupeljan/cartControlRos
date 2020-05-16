#include "rospublisher.h"
#include "kinematic/cartkinematic.h"
#include <QDebug>
#include <functional>

RosPublisher::RosPublisher()
{
    // Init ros
    int argc = 0;
    char **argv = NULL;
    ros::init(argc,argv,"Cart_clietnt");
    // Create node handler
    node = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    // Setup topic name
    topicName = "/robot_mobile_wheel_3_omni_open_base/velocity";
    // Setup publisher
    VelocityPub = node->advertise<CartConrolPlugin::VelocityWheels>(topicName,1000);
    // Setup loop rate
    loopRate = std::unique_ptr<ros::Rate>(new ros::Rate(1000));
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

std::string RosPublisher::velocityToString(const CartConrolPlugin::VelocityWheels v){
    return  "left " + std::to_string(v.left) +'\n' +
            "right " + std::to_string(v.right) + '\n' +
            "back " + std::to_string(v.back) + '\n';
}

void RosPublisher::sendRoutine(std::future<void> futureObj)
{
    // while not set exitSignal
    while (futureObj.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout)
         {
#if DEBUG == 1
          qDebug((velocityToString(velocity) + "-----------\n").c_str());
#endif

          // Send velocity to the topic
          VelocityPub.publish(velocity);

          // Iterate
          ros::spinOnce();
          loopRate->sleep();
       }
}

void RosPublisher::setVelocity(CartConrolPlugin::VelocityWheels v)
{
    // Stop sendRoutine
    //Set the value in promise
    exitSignal->set_value();
    velocityPubThread.join();
    // Set velocity
    velocity = v;
    // start  sendRoutine again
    exitSignal = std::unique_ptr<std::promise<void>>(new std::promise<void>());
    futureObj = exitSignal->get_future();
    velocityPubThread = std::thread(&RosPublisher::sendRoutine,this,std::move(futureObj));
}

void RosPublisher::setVelocity(QPointF p)
{
    CartConrolPlugin::VelocityCart v;
    // Transform to velocity and send
   // p.setX(0);
    //p.setY(1);
    setVelocity(CartKinematic::getVelocity(CartKinematic::PointF(p.x(),p.y())));
    /*
    v.left = -1.;
    v.back = 0.;
    v.right = 1.;
    setVelocity(v);
    */

}

void RosPublisher::pause()
{
    exitSignal->set_value();
    velocityPubThread.join();
}

void RosPublisher::resume()
{
    exitSignal = std::unique_ptr<std::promise<void>>(new std::promise<void>());
    futureObj = exitSignal->get_future();
    velocityPubThread = std::thread(&RosPublisher::sendRoutine,this,std::move(futureObj));
}
