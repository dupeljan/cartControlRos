#include "cartpathabstract.h"

CartPathAbstract::CartPathAbstract(QGraphicsView *parent) : QGraphicsView(parent)
{
    // Prepare scene
    scene = new QGraphicsScene();
    this->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
    int r(500);
    this->resize(r,r);

    this->setSceneRect(0, 0, width(), width());
    this->setScene(scene);

    this->simulationFieldSize.setHeight(10.0);
    this->simulationFieldSize.setWidth(10.0);

    this->simulationStartPoint.setX(5.0);
    this->simulationStartPoint.setY(5.0);
}
