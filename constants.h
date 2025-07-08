//
// Created by beast-machine-2 on 7/6/25.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
#include <eigen-3.4.0/Eigen/Core>


//TODO: where should constants be>
namespace Constants {
    double TIME_INTERVAL = 0.016;
    int SDL_TIME_INTERVAL = 16;

    double SCALING_FACTOR = 1.0;

    int SDL_WINDOW_X = 640.0 * SCALING_FACTOR;
    int SDL_WINDOW_Y = 480.0 * SCALING_FACTOR;

    int SDL_WINDOW_WIDTH = 640.0 * SCALING_FACTOR;
    int SDL_WINDOW_LENGTH = 480.0 * SCALING_FACTOR;


    double CENTER_X = SDL_WINDOW_WIDTH / 2.0;
    double CENTER_Y = SDL_WINDOW_LENGTH / 2.0;


    int CAR_WIDTH = std::floor(25.0 * SCALING_FACTOR);
    int CAR_LENGTH = std::floor(45.0 * SCALING_FACTOR);


    double DIST_TO_WHEEL = sqrt(CAR_WIDTH * CAR_WIDTH + CAR_LENGTH * CAR_LENGTH);

    double CAR_POWER = 100;
    double BRAKING_POWER = 150;

    double CAR_MASS = 1.0;
    double CAR_MOMENT_OF_INERTIA = 1.0;

    double WHEEL_RADIUS = 0.5;
    double WHEEL_FRICTION;

    double STEERING_RACK = 0.5;


    Eigen::Vector2d UP_VECTOR{0, 1};
    Eigen::Vector2d DOWN_VECTOR{0, -1};
    Eigen::Vector2d LEFT_VECTOR{-1, 0};
    Eigen::Vector2d RIGHT_VECTOR{1, 0};

    Eigen::Vector2d ZERO_VECTOR{0, 0};
}
#endif //CONSTANTS_H
