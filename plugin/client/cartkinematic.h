#ifndef CARTKINEMATIC_H
#define CARTKINEMATIC_H

#include "commonheader.h"

#include <CartConrolPlugin/Velocity.h>
#include <QPointF>
#include <vector>

namespace CartKinematic
{
    const double pi = 3.14159265358979311;
    // Angles between robot Y coord and wheel tangent
    const double alpha[] = { /*150.0*/ 5.0*pi/6.0, /*270.0*/ 3.0*pi/2.0, /*30.0*/ pi/6.0 };
    // Wheel count
    const int wheelCount = 3;
    // Wheel radius
    const double r = 0.01905;
    // Distance between body center and wheel center
    const double R = 0.04;
    // Global robot angle
    const double fi = 0.0;

    CartConrolPlugin::Velocity getVelocity(QPointF p, double anglVel = 0.0);
};

#endif // CARTKINEMATIC_H
