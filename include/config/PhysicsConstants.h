#ifndef PHYSICSCONSTANTS_H
#define PHYSICSCONSTANTS_H

#include <cmath>

#include "vehicle/Gearbox.h"

namespace PhysicsConstants {
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;

    constexpr double PIXELS_PER_METER = 10.0;

    constexpr double TIME_INTERVAL = 0.016;
    constexpr int SDL_TIME_INTERVAL = 16;

    constexpr double WHEEL_RADIUS = 0.33;
    constexpr double WHEEL_FRICTION = 1.0;
    constexpr double WHEEL_MASS = 20.0;
    constexpr double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;

    constexpr double TIRE_PEAK_SLIP_ANGLE = 8.0 * DEG_TO_RAD;
    constexpr double TIRE_TRANSITION_SLIP_ANGLE = 15.0 * DEG_TO_RAD;
    constexpr double TIRE_SLIDE_RATIO = 0.75;
    constexpr double TIRE_LOW_SPEED_THRESHOLD = 0.5;
    constexpr double TIRE_SLIP_SETPOINT = 0.1;
    constexpr double TIRE_TCS_kP = 2.0;
    constexpr double TIRE_TCS_kD = 0.5;
    constexpr double ABS_SLIP_SETPOINT = -0.2;
    constexpr double ABS_kP = 2.0;
    constexpr double ABS_kD = 0.5;

    constexpr double CAR_POWER = 4000.0;
    constexpr double BRAKING_POWER = 4000.0;

    constexpr double CAR_MASS = 1200.0;
    constexpr double CAR_WEIGHT = CAR_MASS * 9.81;
    constexpr double CAR_TOP_SPEED = 50.0;

    constexpr double STEERING_RACK = 1.5;
    constexpr double MAX_STEERING_ANGLE = 55.0 * DEG_TO_RAD;
    constexpr double FORCE_FEEDBACK_DECAY = 0.97;

    constexpr double CG_HEIGHT = 0.5;
    constexpr double CLUTCH_MAX_TORQUE = 700.0;
    constexpr double CLUTCH_SLIP_K = 150.0;
    constexpr double TRANS_INERTIA = 0.1;
}

#endif
