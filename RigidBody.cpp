//
// Created by beast-machine-2 on 7/6/25.
//

#include "RigidBody.h"

#include <cmath>
#include <iostream>

#include "constants.h"


RigidBody::RigidBody()
    : velocity(Eigen::Vector2d::Zero()), acceleration(Eigen::Vector2d::Zero()), forces(Eigen::Vector2d::Zero()),
    angular_position(0), angular_velocity(0), angular_acceleration(0), angular_torque(0) {

    mass = Constants::CAR_MASS;
    moment_of_inertia = Constants::CAR_MOMENT_OF_INERTIA;
}


int RigidBody::getPositionX() {
    return std::floor(pos_x);
}

int RigidBody::getPositionY() {
    return std::floor(pos_y);
}

void RigidBody::addForce(Eigen::Vector2d force) {
    forces += force;
}

void RigidBody::addTorque(double torque) {
    angular_torque += torque;
}


void RigidBody::clearForces() {
    forces.setZero();
}

void RigidBody::clearTorques() {
    angular_torque = 0;
}

void RigidBody::incrementTime(double time_interval) {
    acceleration = forces / mass;

    // Y-axis inverted for SDL screen coordinates (positive Y is down on screen)
    pos_x += velocity.x() * time_interval + 0.5 * acceleration.x() * time_interval * time_interval;
    pos_y -= velocity.y() * time_interval + 0.5 * acceleration.y() * time_interval * time_interval;
    velocity = velocity + acceleration * time_interval;

    angular_acceleration = angular_torque / moment_of_inertia;
    angular_position += angular_velocity * time_interval + 0.5 * angular_acceleration * time_interval * time_interval;
    angular_velocity += angular_acceleration * time_interval;

    clearForces();
    clearTorques();
}





