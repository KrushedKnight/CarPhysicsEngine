//
// Created by beast-machine-2 on 7/6/25.
//

#include "Wheel.h"

Wheel::Wheel() : wheelAngle(0) {}

Eigen::Vector2d Wheel::calculateFriction() {
    if (angular_velocity <= 0) {
        return Eigen::Vector2d::Zero();
    }
    double frictionForce = normalForce * frictionCoefficient;

    double x = sin(wheelAngle) * frictionForce;
    double y = cos(wheelAngle) * frictionForce;


    Eigen::Vector2d friction{x, y};

    if (angular_torque < frictionForce * Constants::WHEEL_RADIUS) {
        angular_torque = 0;
    }
    else {
        angular_torque -= frictionForce * Constants::WHEEL_RADIUS;
    }
    return friction;
}

Eigen::Vector2d Wheel::findInertialForce() {
    // return (-angular_torque/Constants::WHEEL_RADIUS);
}

double Wheel::getLinearVelocity() {
    return (angular_velocity*Constants::WHEEL_RADIUS);
}

void Wheel::setLinearVelocity(double linearVelocity) {
    angular_velocity = linearVelocity / Constants::WHEEL_RADIUS;
}