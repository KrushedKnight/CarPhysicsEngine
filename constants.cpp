//
// Created by beast-machine-2 on 7/8/25.
//

#include <cmath>
#include "constants.h"

namespace Constants {

const double TIME_INTERVAL = 0.016;
const int SDL_TIME_INTERVAL = 8;

const double SCALING_FACTOR = 1.0;

const int SDL_WINDOW_X = 640.0 * SCALING_FACTOR;
const int SDL_WINDOW_Y = 480.0 * SCALING_FACTOR;

const int SDL_WINDOW_WIDTH = 640.0 * SCALING_FACTOR;
const int SDL_WINDOW_LENGTH = 480.0 * SCALING_FACTOR;


const double CENTER_X = SDL_WINDOW_WIDTH / 2.0;
const double CENTER_Y = SDL_WINDOW_LENGTH / 2.0;


const int CAR_WIDTH = std::floor(25.0 * SCALING_FACTOR);
const int CAR_LENGTH = std::floor(45.0 * SCALING_FACTOR);


const double DIST_TO_WHEEL = std::sqrt(CAR_WIDTH * CAR_WIDTH + CAR_LENGTH * CAR_LENGTH);
const double WHEEL_RADIUS = 0.5;
const double WHEEL_FRICTION = 0.7;
const double WHEEL_MASS = 0.1;
const double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;

const double CAR_POWER = 150;
const double BRAKING_POWER = -30;

const double CAR_MASS = 1.0;
const double CAR_WEIGHT = 10.0;
const double CAR_MOMENT_OF_INERTIA = (CAR_MASS / 12.0) * (CAR_WIDTH * CAR_WIDTH + CAR_LENGTH * CAR_LENGTH);
const double CAR_TOP_SPEED = 100;


const double STEERING_RACK = 40.0 / 540.0;
const double MAX_STEERING_ANGLE = 540;
}

