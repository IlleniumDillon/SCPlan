#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <iostream>
#include <cmath>
#include "Coordinate.hpp"

class Kinematics
{
public:
    double V, W, dt;
public:
    Kinematics(double V, double W, double dt)
        : V(V), W(W), dt(dt) {}
    void update(CoordD &position0, double &theta0, CoordD &position1, double &theta1);
};

#endif // KINEMATICS_HPP