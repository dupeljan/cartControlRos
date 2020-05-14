#ifndef CARTPATHSETTER_H
#define CARTPATHSETTER_H

#include <thread>
#include <functional>
#include <future>
#include <boost/bind.hpp>

#include "commonheader.h"
#include "cartpathabstract.h"
#include "rospublisher.h"
#include "CartConrolPlugin/PathMsg.h"
#include "std_msgs/Bool.h"

class CartPathSetter: public CartPathAbstract
{
    Q_OBJECT
public:
    explicit CartPathSetter(std::shared_ptr<RosPublisher> rosPubPtr, QGraphicsView *parent = nullptr);
signals:

public slots:
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void sendPath();

private:
    class StatusChecker
    {
        /// Callback function for getting status
        void statusSubCallback(std::shared_ptr<std::promise<void> > p, const std_msgs::Bool::ConstPtr &msg);
        void runThread(std::shared_ptr<std::promise<void>> p);
        std::shared_ptr<ros::NodeHandle> n;
        std::string topicName;
    public:
        StatusChecker(std::shared_ptr<ros::NodeHandle> n, std::string topicName);
        void run(std::shared_ptr<std::promise<void>> p);
    };

    std::string topicName;
    std::string statusTopicName;
    std::shared_ptr<RosPublisher> pub;
    void clearScene();
    void sendPathRoutine(CartConrolPlugin::PathMsg msg);



};

#endif // CARTPATHSETTER_H
