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
    explicit CartPathSetter(QGraphicsView *parent = nullptr);
signals:
    // Send when user shouse path
    void pathChosen(std::vector<QPointF> p);

public slots:
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void sendPath();

private:

    void clearScene();
    void sendPathRoutine(CartConrolPlugin::PathMsg msg);



};

#endif // CARTPATHSETTER_H
