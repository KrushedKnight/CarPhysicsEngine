//
// Created by beast-machine-2 on 7/6/25.
//

#include "Wheel.h"

Wheel::Wheel() : wheelAngle(0) {
    mass = Constants::WHEEL_MASS;
    moment_of_inertia = Constants::WHEEL_MOMENT_OF_INERTIA;
}

Eigen::Vector2d Wheel::calculateFriction(Eigen::Vector2d carVelocity, double carAngularPosition, double time_interval) {
    // Wheel direction vector in world space (angles in radians)
    Eigen::Vector2d wheelDirection{sin(carAngularPosition + wheelAngle), cos(carAngularPosition + wheelAngle)};

    // Project car velocity onto wheel's rolling direction
    double carVelocityInWheelDir = carVelocity.dot(wheelDirection);

    // Calculate wheel's surface velocity from rotation
    double wheelLinearVelocity = wheelRadius * angular_velocity;
    double slip = wheelLinearVelocity - carVelocityInWheelDir;

    // Early exit for negligible slip to avoid numerical issues
    if (std::abs(slip) < 1e-5) {
        return Eigen::Vector2d::Zero();
    }

    // Effective mass for wheel rotation (converted to linear domain)
    double effective_mass = moment_of_inertia / (wheelRadius * wheelRadius);

    // Calculate required friction force to eliminate slip in one timestep
    double requiredFrictionForce = (slip / time_interval) * effective_mass;

    // Limit friction by Coulomb friction law: F_friction <= μ * F_normal
    double maxFrictionForce = normalForce * frictionCoefficient;
    double frictionForce = std::clamp(requiredFrictionForce, -maxFrictionForce, maxFrictionForce);

    // Apply friction force in wheel's rolling direction
    Eigen::Vector2d friction = wheelDirection * frictionForce;

    // Apply counter-torque to wheel (opposes slip)
    double frictionTorque = frictionForce * wheelRadius;
    addTorque(-frictionTorque);

    return friction;
}



double Wheel::getLinearVelocity() {
    return angular_velocity * wheelRadius;
}

void Wheel::setLinearVelocity(double linearVelocity) {
    angular_velocity = linearVelocity / wheelRadius;
}

void Wheel::incrementTime(double time_interval) {
    angular_acceleration = angular_torque / moment_of_inertia;
    angular_position += angular_velocity * time_interval + 0.5 * angular_acceleration * time_interval * time_interval;
    angular_velocity += angular_acceleration * time_interval;

    clearTorques();
}