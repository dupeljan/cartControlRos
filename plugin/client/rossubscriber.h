#ifndef ROSSUBSCRIBER_H
#define ROSSUBSCRIBER_H

#include <string>
#include <QDebug>
#include <QObject>
#include <QPointF>

#include <ros/ros.h>

#include <thread>
#include <future>
#include <boost/bind.hpp>

#include "CartConrolPlugin/Position.h"

class RosSubscriber : public QObject
{
    Q_OBJECT
public:
    RosSubscriber();
    ~RosSubscriber();


signals:
    void getPos(QPointF p);

public slots:


private:
    void runRoutine();
    void callBack(const CartConrolPlugin::Position::ConstPtr &msg);
    std::thread runThread;
    std::string topicName;
    std::promise<void> promise;
    std::future<void> future;
};

#endif // ROSSUBSCRIBER_H
