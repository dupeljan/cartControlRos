#ifndef CARTPATHSETTER_H
#define CARTPATHSETTER_H

#include "commonheader.h"
#include "cartpathabstract.h"


class CartPathSetter: public CartPathAbstract
{
    Q_OBJECT
public:
    explicit CartPathSetter(QGraphicsView *parent = nullptr);
signals:

public slots:
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void sendPath();

private:


    void clearScene();
};

#endif // CARTPATHSETTER_H
