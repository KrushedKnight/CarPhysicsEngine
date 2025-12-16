#ifndef WHEEL_H
#define WHEEL_H

#include "config/PhysicsConstants.h"
#include "core/RigidBody.h"

class Wheel : public RigidBody {
public:
    double wheelAngle;

    double wheelRadius{PhysicsConstants::WHEEL_RADIUS};

    double frictionCoefficient{PhysicsConstants::WHEEL_FRICTION};

    double normalForce{PhysicsConstants::CAR_WEIGHT / 4.0};

    double gripLevel{0.0};

    Eigen::Vector2d lastForce{0.0, 0.0};
    Eigen::Vector2d lastVelocity{0.0, 0.0};

    Eigen::Vector2d position{0.0, 0.0};

    double previousSlipError{0.0};
    double tcsInterference{0.0};
    double previousAbsSlipError{0.0};
    double absInterference{0.0};

    Wheel();

    Eigen::Vector2d calculateFriction(Eigen::Vector2d wheelVelocityLocal, double time_interval);

    double calculateSlipRatio(Eigen::Vector2d wheelVelocityLocal);

    double getLinearVelocity();
    void setLinearVelocity(double linearVelocity);

    void incrementTime(double time_interval);
};

#endif
