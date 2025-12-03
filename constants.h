//
// Created by beast-machine-2 on 7/6/25.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
#include <Eigen/Core>

namespace Constants {
    extern const double TIME_INTERVAL;
    extern const int SDL_TIME_INTERVAL;

    extern const double SCALING_FACTOR;

    extern const int SDL_WINDOW_X;
    extern const int SDL_WINDOW_Y;

    extern const int SDL_WINDOW_WIDTH;
    extern const int SDL_WINDOW_LENGTH;


    extern const double CENTER_X;
    extern const double CENTER_Y;


    extern const int CAR_WIDTH;
    extern const int CAR_LENGTH;


    extern const double DIST_TO_WHEEL;

    extern const double WHEEL_RADIUS;
    extern const double WHEEL_FRICTION;
    extern const double WHEEL_MASS;
    extern const double WHEEL_MOMENT_OF_INERTIA;

    extern const double CAR_POWER;
    extern const double BRAKING_POWER;

    extern const double CAR_MASS;
    extern const double CAR_WEIGHT;
    extern const double CAR_MOMENT_OF_INERTIA;


    extern const double STEERING_RACK;
    extern const double MAX_STEERING_ANGLE;

    extern const double CAR_TOP_SPEED;
}
#endif //CONSTANTS_H
