#include "Wheel.h"

Wheel::Wheel() : wheelAngle(0) {
    mass = Constants::WHEEL_MASS;
    moment_of_inertia = Constants::WHEEL_MOMENT_OF_INERTIA;
}

Eigen::Vector2d Wheel::calculateFriction(Eigen::Vector2d wheelVelocityLocal, double time_interval) {
    Eigen::Vector2d wheelForward{sin(wheelAngle), cos(wheelAngle)};
    Eigen::Vector2d wheelRight{cos(wheelAngle), -sin(wheelAngle)};

    double velocityInWheelDir = wheelVelocityLocal.dot(wheelForward);
    double wheelLinearVelocity = wheelRadius * angular_velocity;
    double longitudinalSlip = wheelLinearVelocity - velocityInWheelDir;

    double wheelMass = normalForce / 9.81;
    double maxFrictionForce = normalForce * frictionCoefficient;

    double longitudinalFriction = 0.0;
    if (std::abs(longitudinalSlip) > 1e-5) {
        const double LONGITUDINAL_FRICTION_RESPONSE = 0.4;
        double requiredForce = (longitudinalSlip / time_interval) * wheelMass * LONGITUDINAL_FRICTION_RESPONSE;
        longitudinalFriction = std::clamp(requiredForce, -maxFrictionForce, maxFrictionForce);

        double wheelEffectiveMass = moment_of_inertia / (wheelRadius * wheelRadius);
        double frictionTorque = longitudinalFriction * wheelRadius * (wheelEffectiveMass / wheelMass);
        addTorque(-frictionTorque);
    }

    double lateralVelocity = wheelVelocityLocal.dot(wheelRight);
    double lateralFriction = 0.0;

    if (std::abs(lateralVelocity) > 1e-5) {
        const double LATERAL_FRICTION_RESPONSE = 0.35;
        double requiredLateralForce = -(lateralVelocity / time_interval) * wheelMass * LATERAL_FRICTION_RESPONSE;
        lateralFriction = std::clamp(requiredLateralForce, -maxFrictionForce, maxFrictionForce);
    }

    double combinedMagnitude = std::sqrt(longitudinalFriction * longitudinalFriction +
                                         lateralFriction * lateralFriction);

    if (combinedMagnitude > maxFrictionForce) {
        double scale = maxFrictionForce / combinedMagnitude;
        longitudinalFriction *= scale;
        lateralFriction *= scale;
    }

    // Calculate grip level (0.0 = no force, 1.0 = at friction limit)
    gripLevel = (maxFrictionForce > 0.0) ? (combinedMagnitude / maxFrictionForce) : 0.0;

    return wheelForward * longitudinalFriction + wheelRight * lateralFriction;
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