#ifndef CARTKINEMATIC_H
#define CARTKINEMATIC_H


#include <CartControlPlugin/VelocityWheels.h>
#include <CartControlPlugin/VelocityCart.h>
#include <vector>
#include <math.h>

#include "matrixproc.h"


/*
 * Function to compute
 * forward and reverse cart kinematic
*/
namespace CartKinematic
{

    struct PointF
    {
        PointF(double x, double y)
        {
            this->x = x;
            this->y = y;
        }

        PointF()
        {
            this->x = 0.0;
            this->y = 0.0;
        }

        PointF operator - ()
        {
            return PointF(-this->x,-this->y);
        }
        PointF operator + (PointF Point2)
        {
            return PointF(this->x + Point2.x,this->y + Point2.y);
        }
        PointF operator - (PointF Point2)
        {
            return (*this + (- Point2));
        }

        double x,y;
    };

    ///
    /// Cart constrains
    ///
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

    const double cmnF = ( r * 2.0 / ( R * 3.0 * std::sqrt(3.0) ) );

    const std::vector<std::vector<double>> reverseKinematixMat =
    {{ cmnF * ( -R * std::sqrt(3.0) / 2.0), cmnF * ( R * std::sqrt(3.0) ),   cmnF * ( - R * std::sqrt(3.0) / 2.0 )},
     { cmnF * ( -R * 3.0 / 2.0),            cmnF * ( 0.0 ),                  cmnF * ( R * 3.0 / 2.0 )},
     { cmnF * ( std::sqrt(3.0) / 2.0),      cmnF * ( std::sqrt(3.0) / 2.0 ), cmnF * ( std::sqrt(3.0) / 2.0 )}
    };

    // Return wheels velocty from linear cart velocity vector
    CartControlPlugin::VelocityWheels getVelocity(PointF p, double anglVel = 0.0);

    //
    CartControlPlugin::VelocityWheels getVelocity(CartControlPlugin::VelocityCart v);

    // Return linear cart velocity vector from  wheels velocty
    CartControlPlugin::VelocityCart getVelocityReverse(CartControlPlugin::VelocityWheels v);

    // Distance between two points
    double distance(PointF a,PointF b);

    // Distance between line a - b and point p
    // Positive if p left-handed, negative otherwise
    double signDistanceToLine(PointF a,PointF b, PointF p);

    // Get angle oposite to BC edge in triangle ABC
    double angleOpositeBC(PointF a, PointF b, PointF c);

    // Norm of cart velocity vector
    double norm(CartControlPlugin::VelocityCart p);

    // Scalar mul between two velocity vectors
    double scalarMul(CartControlPlugin::VelocityCart v1, CartControlPlugin::VelocityCart v2);
};

#endif // CARTKINEMATIC_H
