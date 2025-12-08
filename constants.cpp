#include <cmath>
#include "constants.h"

namespace Constants {

const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

const double PIXELS_PER_METER = 10.0;

const double TIME_INTERVAL = 0.016;
const int SDL_TIME_INTERVAL = 16;

const double SCALING_FACTOR = 1.0;
const int SDL_WINDOW_X = 100;
const int SDL_WINDOW_Y = 100;
const int SDL_WINDOW_WIDTH = 1920;
const int SDL_WINDOW_LENGTH = 1080;
const double CENTER_X = SDL_WINDOW_WIDTH / 2.0;
const double CENTER_Y = SDL_WINDOW_LENGTH / 2.0;

const int CAR_WIDTH = std::floor(25.0 * SCALING_FACTOR);
const int CAR_LENGTH = std::floor(45.0 * SCALING_FACTOR);

const double WHEEL_RADIUS = 0.33;
const double WHEEL_FRICTION = 1.0;
const double WHEEL_MASS = 20.0;
const double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;


const double TIRE_PEAK_SLIP_ANGLE = 8.0 * DEG_TO_RAD;  
const double TIRE_TRANSITION_SLIP_ANGLE = 15.0 * DEG_TO_RAD; 
const double TIRE_SLIDE_RATIO = 0.75;                 
const double TIRE_LOW_SPEED_THRESHOLD = 0.5;               

const double CAR_POWER = 4000.0;
const double BRAKING_POWER = 4000.0;

const double CAR_MASS = 1200.0;
const double CAR_WEIGHT = CAR_MASS * 9.81;
const double CAR_MOMENT_OF_INERTIA = (CAR_MASS / 12.0) * (
    (CAR_WIDTH / 10.0) * (CAR_WIDTH / 10.0) +
    (CAR_LENGTH / 10.0) * (CAR_LENGTH / 10.0)
);
const double CAR_TOP_SPEED = 50.0;

const double STEERING_RACK = 1.0;
const double MAX_STEERING_ANGLE = 45.0 * DEG_TO_RAD;
const double FORCE_FEEDBACK_DECAY = 0.97;
}