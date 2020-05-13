#ifndef CARTPATHGETTER_H
#define CARTPATHGETTER_H

#include "commonheader.h"
#include "cartpathabstract.h"

class CartPathGetter : public CartPathAbstract
{
    Q_OBJECT
public:
    explicit CartPathGetter(QGraphicsView *parent = nullptr);

signals:

public slots:
    void drawSimulationPos(QPointF point);
    void drawAnaliticPath(std::vector<QPointF> v);
private:
    QPointF translate(QPointF p);
};

#endif // CARTPATHGETTER_H
