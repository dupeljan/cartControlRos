#ifndef CARTPATHSETTER_H
#define CARTPATHSETTER_H

#include "commonheader.h"
#include "cartpathabstract.h"
#include "rospublisher.h"

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
};

#endif // CARTPATHSETTER_H
