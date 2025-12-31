#include "vehicle/Wheel.h"

Wheel::Wheel() : wheelAngle(0) {
    mass = PhysicsConstants::WHEEL_MASS;
    moment_of_inertia = PhysicsConstants::WHEEL_MOMENT_OF_INERTIA;
}

double Wheel::calculateSlipRatio(Eigen::Vector2d wheelVelocityLocal) {
    Eigen::Vector2d wheelForward{sin(wheelAngle), cos(wheelAngle)};

    double vehicleSpeed = wheelVelocityLocal.dot(wheelForward);

    double wheelSpeed = wheelRadius * angular_velocity;


    if (std::abs(vehicleSpeed) < 0.1) {
        return 0.0;
    }

    double slipRatio = (wheelSpeed - vehicleSpeed) / std::abs(vehicleSpeed);

    return slipRatio;
}

Eigen::Vector2d Wheel::calculateFriction(Eigen::Vector2d wheelVelocityLocal, double time_interval) {
    Eigen::Vector2d wheelForward{sin(wheelAngle), cos(wheelAngle)};
    Eigen::Vector2d wheelRight{cos(wheelAngle), -sin(wheelAngle)};

    double velocityInWheelDir = wheelVelocityLocal.dot(wheelForward);
    double wheelLinearVelocity = wheelRadius * angular_velocity;
    double longitudinalSlip = wheelLinearVelocity - velocityInWheelDir;

    double wheelMass = normalForce / 9.81;

    double nominalLoad = 2943.0;
    double loadSensitivity = 0.9;
    double loadFactor = std::pow(normalForce / nominalLoad, loadSensitivity);
    double maxFrictionForce = nominalLoad * frictionCoefficient * loadFactor;

    double longitudinalFriction = 0.0;
    if (std::abs(longitudinalSlip) > 1e-5) {
        const double LONGITUDINAL_FRICTION_RESPONSE = 0.6;
        double requiredForce = (longitudinalSlip / time_interval) * wheelMass * LONGITUDINAL_FRICTION_RESPONSE;
        longitudinalFriction = std::clamp(requiredForce, -maxFrictionForce, maxFrictionForce);

        double wheelEffectiveMass = moment_of_inertia / (wheelRadius * wheelRadius);
        double frictionTorque = longitudinalFriction * wheelRadius * (wheelEffectiveMass / wheelMass);
        addTorque(-frictionTorque);
    }

    double lateralVelocity = wheelVelocityLocal.dot(wheelRight);
    double lateralFriction = 0.0;

    if (std::abs(lateralVelocity) > 1e-5) {
        double speed = std::sqrt(velocityInWheelDir * velocityInWheelDir +
                                 lateralVelocity * lateralVelocity);

        if (speed < PhysicsConstants::TIRE_LOW_SPEED_THRESHOLD) {
            const double LATERAL_FRICTION_RESPONSE = 0.45;
            double requiredLateralForce = -(lateralVelocity / time_interval) * wheelMass * LATERAL_FRICTION_RESPONSE;
            lateralFriction = std::clamp(requiredLateralForce, -maxFrictionForce, maxFrictionForce);
        } else {
            double slipAngle = std::atan2(std::abs(lateralVelocity), std::abs(velocityInWheelDir));

            double forceMagnitude = 0.0;

            if (slipAngle <= PhysicsConstants::TIRE_PEAK_SLIP_ANGLE) {
                forceMagnitude = (slipAngle / PhysicsConstants::TIRE_PEAK_SLIP_ANGLE) * maxFrictionForce;
            } else if (slipAngle <= PhysicsConstants::TIRE_TRANSITION_SLIP_ANGLE) {
                double peakForce = maxFrictionForce;
                double slideForce = maxFrictionForce * PhysicsConstants::TIRE_SLIDE_RATIO;
                double t = (slipAngle - PhysicsConstants::TIRE_PEAK_SLIP_ANGLE) /
                          (PhysicsConstants::TIRE_TRANSITION_SLIP_ANGLE - PhysicsConstants::TIRE_PEAK_SLIP_ANGLE);
                forceMagnitude = peakForce - t * (peakForce - slideForce);
            } else {
                forceMagnitude = maxFrictionForce * PhysicsConstants::TIRE_SLIDE_RATIO;
            }

            lateralFriction = -std::copysign(forceMagnitude, lateralVelocity);
        }
    }

    double combinedMagnitude = std::sqrt(longitudinalFriction * longitudinalFriction +
                                         lateralFriction * lateralFriction);

    if (combinedMagnitude > maxFrictionForce) {
        double scale = maxFrictionForce / combinedMagnitude;
        longitudinalFriction *= scale;
        lateralFriction *= scale;
    }

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
