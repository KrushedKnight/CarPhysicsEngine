//
// Created by beast-machine-2 on 7/6/25.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
#include <eigen-3.4.0/Eigen/Core>


//TODO: where should constants be>
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

    extern const double CAR_POWER;
    extern const double BRAKING_POWER;

    extern const double CAR_MASS;
    extern const double CAR_MOMENT_OF_INERTIA;

    extern const double WHEEL_RADIUS;
    extern const double WHEEL_FRICTION;

    extern const double STEERING_RACK;


    // Eigen::Vector2d UP_VECTOR{0, 1};
    // Eigen::Vector2d DOWN_VECTOR{0, -1};
    // Eigen::Vector2d LEFT_VECTOR{-1, 0};
    // Eigen::Vector2d RIGHT_VECTOR{1, 0};
    //
    // Eigen::Vector2d ZERO_VECTOR{0, 0};
}
#endif //CONSTANTS_H
