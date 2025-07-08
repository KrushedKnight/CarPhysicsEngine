//
// Created by beast-machine-2 on 7/6/25.
//

#ifndef WHEEL_H
#define WHEEL_H

#include "constants.h"
#include "RigidBody.h"


class Wheel : public RigidBody {
public:
    double wheelAngle;

    double wheelRadius{Constants::WHEEL_RADIUS};

    double frictionCoefficient{Constants::WHEEL_FRICTION};

    double normalForce;

    Wheel();

    // Eigen::Vector2d calculateFriction();
};



#endif //WHEEL_H
