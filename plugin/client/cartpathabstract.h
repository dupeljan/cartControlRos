#ifndef CARTPATHABSTRACT_H
#define CARTPATHABSTRACT_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QPoint>
#include <QPointF>
#include <vector>

class CartPathAbstract : public QGraphicsView
{
    Q_OBJECT
public:
    explicit CartPathAbstract(QGraphicsView *parent = nullptr);

signals:

public slots:

protected:
    QGraphicsScene * scene;
    std::vector<QPointF> path;

    // Size of Gazebo scine
    QSizeF simulationFieldSize;

    // Gazebo coordinates in centre of the scene
    QPointF simulationStartPoint;
};

#endif // CARTPATHABSTRACT_H
