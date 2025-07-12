//
// Created by beast-machine-2 on 7/6/25.
//

#include "Wheel.h"

Wheel::Wheel() : wheelAngle(0) {}

Eigen::Vector2d Wheel::calculateFriction(double carVelocity, double enginePower) {
    if (angular_velocity <= 0) {
        return Eigen::Vector2d::Zero();
    }

    double frictionForce = normalForce * frictionCoefficient;

    angular_velocity += Constants::TIME_INTERVAL * (enginePower - frictionForce * Constants::WHEEL_RADIUS) / moment_of_inertia;
    double slip = Constants::WHEEL_RADIUS * angular_velocity - velocity.norm();

    double frictionSign =  (slip >= 0) ? -1.0 : 1.0;



    double x = sin(wheelAngle) * frictionForce * frictionSign;
    double y = cos(wheelAngle) * frictionForce * frictionSign;


    Eigen::Vector2d friction{x, y};

    if (angular_torque < 0) {
        angular_torque += frictionForce * Constants::WHEEL_RADIUS;
        angular_torque = std::min(angular_torque, 0.0);
    }
    else if (angular_torque > 0) {
        angular_torque -= frictionForce * Constants::WHEEL_RADIUS;
        angular_torque = std::max(angular_torque, 0.0);
    }

    return friction;
}



double Wheel::getLinearVelocity() {
    return (angular_velocity*Constants::WHEEL_RADIUS);
}

void Wheel::setLinearVelocity(double linearVelocity) {
    angular_velocity = linearVelocity / Constants::WHEEL_RADIUS;
}

void Wheel::incrementTime(double time_interval) {
    angular_acceleration = angular_torque / moment_of_inertia;
    angular_position += angular_velocity * time_interval + 0.5 * angular_acceleration * time_interval * time_interval;
    angular_velocity += angular_acceleration * time_interval;

    clearTorques();
}