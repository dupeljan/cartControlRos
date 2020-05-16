#ifndef ROSPUBLISHER_H
#define ROSPUBLISHER_H

#include "commonheader.h"

#include "ros/ros.h"
#include <CartConrolPlugin/VelocityWheels.h>
#include <QPoint>
#include <thread>
#include <mutex>
#include <string>
#include <future>

class RosPublisher
{
private:
    // Node handler
    std::unique_ptr<ros::NodeHandle> node;
    // Publisher for velocity
    ros::Publisher VelocityPub;
    // Ros loop rate
    std::unique_ptr<ros::Rate> loopRate;
    // Topic name
    std::string topicName;
    // Shared with sendRoutine velocity
    CartConrolPlugin::VelocityWheels velocity;
    // Mutex for velocity
    std::mutex velocityMutex;
    // Thread for sendRoutine
    std::thread  velocityPubThread;
    // Create a std::promise object
    std::unique_ptr<std::promise<void>> exitSignal;
    //std::future object associated with promise
    std::future<void> futureObj;
    // Velocity to string converter
    std::string velocityToString(const CartConrolPlugin::VelocityWheels v);
    // Routine in which velocity sending to cart
    void sendRoutine(std::future<void> futureObj);
public:
     // Set velocity which publishing in SendRoutine
    void setVelocity(CartConrolPlugin::VelocityWheels v);
    void setVelocity(QPointF p);
    void pause();
    void resume();
    RosPublisher();
    ~RosPublisher();

};

#endif // ROSPUBLISHER_H
