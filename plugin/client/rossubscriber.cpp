#include "rossubscriber.h"

RosSubscriber::RosSubscriber()
{
    this->topicName = "/robot_mobile_wheel_3_omni_open_base/pos";

    this->future = this->promise.get_future();
    this->runThread =
         std::thread(std::bind(&RosSubscriber::runRoutine,this));
}


RosSubscriber::~RosSubscriber()
{
    this->promise.set_value();
    this->runThread.join();
    QObject::deleteLater();
}


void RosSubscriber::runRoutine()
{
    // Subsribe
    std::unique_ptr<ros::NodeHandle> n;
    n.reset(new ros::NodeHandle("Client_subscriber"));
    auto sub = n->subscribe<CartControlPlugin::Position>
            (topicName, 10000,
             boost::bind(&RosSubscriber::callBack,this,_1));

    // Setup loop rate
    auto loopRate = ros::Rate(10000);

    // While promise doesn't set it's value
    while(this->future.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout)
    {
        ros::spinOnce();
        loopRate.sleep();
    }

    n->shutdown();
}

void RosSubscriber::callBack(const CartControlPlugin::Position::ConstPtr &msg)
{
    QPointF p;
    p.setX(msg->x);
    p.setY(msg->y);
    emit getPos(p);
}
