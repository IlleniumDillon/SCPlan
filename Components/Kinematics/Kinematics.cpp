#include "Kinematics.hpp"

void Kinematics::update(CoordD &position0, double &theta0, CoordD &position1, double &theta1)
{
    if (theta0 >= 2 * M_PI)
    {
        theta0 -= 2 * M_PI;
    }
    if (theta0 < 0)
    {
        theta0 += 2 * M_PI;
    }
    if (theta1 >= 2 * M_PI)
    {
        theta1 -= 2 * M_PI;
    }
    if (theta1 < 0)
    {
        theta1 += 2 * M_PI;
    }

    theta1 = theta0 + W * dt;
    double dtheta = theta1 - theta0;
    if (abs(dtheta) < 1e-6)
    {
        double dx = V * dt * std::cos(theta0);
        double dy = V * dt * std::sin(theta0);

        position1.x = position0.x + dx;
        position1.y = position0.y + dy;
    }
    else
    {
        double R = V / W;
        double Choord = std::sqrt(
            2 * R * R * (1 - std::cos(dtheta))
        );
        double dx = Choord * std::cos(theta0 + dtheta / 2);
        double dy = Choord * std::sin(theta0 + dtheta / 2);

        position1.x = position0.x + dx;
        position1.y = position0.y + dy;
    }
    // position1.x = position0.x - R * std::cos(theta0) + R * std::cos(theta1);
    // position1.y = position0.y - R * std::sin(theta0) + R * std::sin(theta1);
}