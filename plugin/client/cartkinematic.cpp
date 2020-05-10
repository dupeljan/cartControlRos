#include "cartkinematic.h"
#include <math.h>
#include <vector>


CartConrolPlugin::Velocity CartKinematic::getVelocity(QPointF p)
{
    CartConrolPlugin::Velocity v;
    // Normalize vector
    double len = std::sqrt(std::pow(p.x(),2) + std::pow(p.y(),2));
    p.setX(p.x() / len);
    p.setY(p.y() / len);
    // Strech velocity
    double strechC = 0.01;
    p.setX( p.x() * strechC);
    p.setY( p.y() * strechC);
    // Do transformations
    double vVector[wheelCount];
    for(int i = 0; i < wheelCount; i++ )
        vVector[i] = (- std::sin(fi +alpha[i] ) * p.x() + std::cos(fi + alpha[i] ) * p.y()  ) / r;
    v.left = vVector[0];
    v.back = vVector[1];
    v.right = vVector[2];
    return v;
}
