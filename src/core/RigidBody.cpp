#include "core/RigidBody.h"

#include <cmath>
#include <iostream>

#include "config/PhysicsConstants.h"
#include "config/RenderingConstants.h"

RigidBody::RigidBody()
    : velocity(Eigen::Vector2d::Zero()), acceleration(Eigen::Vector2d::Zero()), forces(Eigen::Vector2d::Zero()),
    angular_position(0), angular_velocity(0), angular_acceleration(0), angular_torque(0) {

    mass = PhysicsConstants::CAR_MASS;
    moment_of_inertia = RenderingConstants::CAR_MOMENT_OF_INERTIA;
}

int RigidBody::getPositionX() {
    return std::floor(pos_x);
}

int RigidBody::getPositionY() {
    return std::floor(pos_y);
}

void RigidBody::addForce(Eigen::Vector2d force, const std::string& name) {
    forces += force;

    if (!name.empty()) {
        namedForces[name] += force;
    }
}

void RigidBody::addTorque(double torque) {
    angular_torque += torque;
}

void RigidBody::clearForces() {
    forces.setZero();
    namedForces.clear();
}

void RigidBody::clearTorques() {
    angular_torque = 0;
}

const std::map<std::string, Eigen::Vector2d>& RigidBody::getNamedForces() const {
    return namedForces;
}

void RigidBody::incrementTime(double time_interval) {
    acceleration = forces / mass;

    double dx_meters = velocity.x() * time_interval + 0.5 * acceleration.x() * time_interval * time_interval;
    double dy_meters = velocity.y() * time_interval + 0.5 * acceleration.y() * time_interval * time_interval;

    pos_x += dx_meters * PhysicsConstants::PIXELS_PER_METER;
    pos_y -= dy_meters * PhysicsConstants::PIXELS_PER_METER;

    velocity = velocity + acceleration * time_interval;

    angular_acceleration = angular_torque / moment_of_inertia;
    angular_position += angular_velocity * time_interval + 0.5 * angular_acceleration * time_interval * time_interval;
    angular_velocity += angular_acceleration * time_interval;

    if (std::isfinite(angular_position)) {
        angular_position = std::remainder(angular_position, 2.0 * M_PI);
    }

    static int frame_count = 0;
    if (frame_count++ % 30 == 0) {
        std::cout << "Angle: " << angular_position * PhysicsConstants::RAD_TO_DEG
                  << "° | AngVel: " << angular_velocity
                  << " | Torque: " << angular_torque << std::endl;
    }

    clearForces();
    clearTorques();
}
