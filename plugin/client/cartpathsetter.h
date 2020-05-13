#ifndef CARTPATHSETTER_H
#define CARTPATHSETTER_H

#include <thread>
#include <functional>

#include "commonheader.h"
#include "cartpathabstract.h"
#include "rospublisher.h"
#include "CartConrolPlugin/PathMsg.h"

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
    std::string topicName;
    std::shared_ptr<RosPublisher> pub;
    void clearScene();
    void sendPathRoutine(CartConrolPlugin::PathMsg msg);
};

#endif // CARTPATHSETTER_H
