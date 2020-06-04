#include "cartkinematic.h"



CartControlPlugin::VelocityWheels CartKinematic::getVelocity(PointF p, double anglVel)
{
    CartControlPlugin::VelocityWheels v;
    // Normalize vector
    double len = std::sqrt(std::pow(p.x,2) + std::pow(p.y,2));
    p.x /= len;
    p.y /= len;
    // Strech velocity
    // strechC * ( 1.0 / 5.0 )= unit/sec
    double strechC = 0.5;
    p.x *= strechC;
    p.y *= strechC;
    // Do transformations
    double vVector[wheelCount];
    for(int i = 0; i < wheelCount; i++ )
        vVector[i] = (- std::sin(fi +alpha[i] ) * p.x + std::cos(fi + alpha[i] ) * p.y + R * anglVel ) / r;
    v.left = vVector[0];
    v.back = vVector[1];
    v.right = vVector[2];
    return v;
}

CartControlPlugin::VelocityWheels CartKinematic::getVelocity(CartControlPlugin::VelocityCart v)
{
    return CartKinematic::getVelocity(PointF(v.x,v.y),v.angle);
}

CartControlPlugin::VelocityCart CartKinematic::getVelocityReverse(CartControlPlugin::VelocityWheels v)
{
    std::vector<double> vel = { v.left, v.back, v.right };
    auto res = MatrixProc::composition<double>(reverseKinematixMat,vel);
    CartControlPlugin::VelocityCart vRes;
    vRes.x = res[0];
    vRes.y = res[1];
    vRes.angle = res[2];
    return vRes;
}


double CartKinematic::distance(CartKinematic::PointF a, CartKinematic::PointF b)
{
    return std::sqrt(std::pow(b.x - a.x,2) + std::pow(b.y - a.y,2));
}


double CartKinematic::norm(CartControlPlugin::VelocityCart p)
{
    return std::sqrt(std::pow(p.x,2) + std::pow(p.y,2) + std::pow(p.angle,2));
}

double CartKinematic::scalarMul(CartControlPlugin::VelocityCart v1, CartControlPlugin::VelocityCart v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.angle * v2.angle;
}

double CartKinematic::signDistanceToLine(CartKinematic::PointF a, CartKinematic::PointF b, CartKinematic::PointF p)
{
    return /*std::abs*/((b.y - a.y) * p.x - (b.x - a.x) * p.y + b.x * a.y - b.y * a.x) / distance(a,b);
}

double CartKinematic::angleOpositeBC(CartKinematic::PointF a, CartKinematic::PointF b, CartKinematic::PointF c)
{
    double A = distance(b,c);
    double B = distance(a,c);
    double C = distance(a,b);
    return std::acos((std::pow(B,2) + std::pow(C,2) - std::pow(A,2)) / (2.0 * B * C) );

}
