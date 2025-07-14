//
// Created by beast-machine-2 on 7/6/25.
//

#include "Wheel.h"

Wheel::Wheel() : wheelAngle(0) {}

Eigen::Vector2d Wheel::calculateFriction(Eigen::Vector2d carVelocity) {



    double slip = Constants::WHEEL_RADIUS * angular_velocity - carVelocity.norm();

    double effective_mass = moment_of_inertia / (Constants::WHEEL_RADIUS * Constants::WHEEL_RADIUS);

    double requiredFrictionForce = (slip / Constants::TIME_INTERVAL) * effective_mass;
    double maxFrictionForce = normalForce * frictionCoefficient;
    double frictionForce = std::clamp(requiredFrictionForce, -maxFrictionForce, maxFrictionForce);


    if (std::abs(slip) < 1e-5) {
        return Eigen::Vector2d::Zero();
    }



    double x = sin(wheelAngle) * frictionForce;
    double y = cos(wheelAngle) * frictionForce;


    Eigen::Vector2d friction{x, y};

    double frictionTorque = frictionForce * wheelRadius;
    addTorque(-frictionTorque);


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