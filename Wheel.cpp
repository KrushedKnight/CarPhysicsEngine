//
// Created by beast-machine-2 on 7/6/25.
//

#include "Wheel.h"

Wheel::Wheel() : wheelAngle(0) {
    mass = Constants::WHEEL_MASS;
    moment_of_inertia = Constants::WHEEL_MOMENT_OF_INERTIA;
}

Eigen::Vector2d Wheel::calculateFriction(Eigen::Vector2d carVelocity, double carAngularPosition, double time_interval) {
    // Wheel direction vectors in world space (angles in radians)
    Eigen::Vector2d wheelDirection{sin(carAngularPosition + wheelAngle), cos(carAngularPosition + wheelAngle)};
    Eigen::Vector2d lateralDirection{cos(carAngularPosition + wheelAngle), -sin(carAngularPosition + wheelAngle)};

    // LONGITUDINAL FRICTION (rolling direction)
    double carVelocityInWheelDir = carVelocity.dot(wheelDirection);
    double wheelLinearVelocity = wheelRadius * angular_velocity;
    double longitudinalSlip = wheelLinearVelocity - carVelocityInWheelDir;

    double effective_mass = moment_of_inertia / (wheelRadius * wheelRadius);
    double maxFrictionForce = normalForce * frictionCoefficient;

    double longitudinalFriction = 0.0;
    if (std::abs(longitudinalSlip) > 1e-5) {
        double requiredForce = (longitudinalSlip / time_interval) * effective_mass;
        longitudinalFriction = std::clamp(requiredForce, -maxFrictionForce, maxFrictionForce);

        double frictionTorque = longitudinalFriction * wheelRadius;
        addTorque(-frictionTorque);
    }

    // LATERAL FRICTION (sideways/perpendicular direction)
    double lateralVelocity = carVelocity.dot(lateralDirection);
    double lateralFriction = 0.0;

    if (std::abs(lateralVelocity) > 1e-5) {
        // Lateral friction resists sideways motion (no wheel rotation involved)
        double requiredLateralForce = -(lateralVelocity / time_interval) * effective_mass;
        lateralFriction = std::clamp(requiredLateralForce, -maxFrictionForce, maxFrictionForce);
    }

    // Combined friction force (Pythagorean limit to stay within friction circle)
    double combinedMagnitude = std::sqrt(longitudinalFriction * longitudinalFriction +
                                         lateralFriction * lateralFriction);

    if (combinedMagnitude > maxFrictionForce) {
        double scale = maxFrictionForce / combinedMagnitude;
        longitudinalFriction *= scale;
        lateralFriction *= scale;
    }

    return wheelDirection * longitudinalFriction + lateralDirection * lateralFriction;
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