#ifndef WHEEL_H
#define WHEEL_H

#include "constants.h"
#include "RigidBody.h"

class Wheel : public RigidBody {
public:
    double wheelAngle;

    double wheelRadius{Constants::WHEEL_RADIUS};

    double frictionCoefficient{Constants::WHEEL_FRICTION};

    double normalForce{Constants::CAR_WEIGHT / 4.0};

    double gripLevel{0.0};  // 0.0 = no force, 1.0 = maximum grip (sliding)

    Wheel();

    Eigen::Vector2d calculateFriction(Eigen::Vector2d wheelVelocityLocal, double time_interval);

    double getLinearVelocity();
    void setLinearVelocity(double linearVelocity);

    void incrementTime(double time_interval);
};

#endif