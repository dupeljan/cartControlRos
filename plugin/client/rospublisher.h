#ifndef ROSPUBLISHER_H
#define ROSPUBLISHER_H

#include "ros/ros.h"
#include <CartConrolPlugin/Velocity.h>
#include <QPoint>
#include <thread>
#include <mutex>
#include <string>

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
    CartConrolPlugin::Velocity velocity;
    // Mutex for velocity
    std::mutex velocityMutex;
    // Velocity to string converter
    std::string velocityToString(const CartConrolPlugin::Velocity v);
    // Routine in which velocity sending to cart
    void sendRoutine();
public:
     // Set velocity which publishing in SendRoutine
    void setVelocity(CartConrolPlugin::Velocity v);
    void setVelocity(QPoint p);

    RosPublisher();
    ~RosPublisher();

};

#endif // ROSPUBLISHER_H
