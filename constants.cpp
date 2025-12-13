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

// Default values, will be updated at runtime
int SDL_WINDOW_WIDTH = 1920;
int SDL_WINDOW_LENGTH = 1080;
double CENTER_X = SDL_WINDOW_WIDTH / 2.0;
double CENTER_Y = SDL_WINDOW_LENGTH / 2.0;

int CAR_WIDTH = std::floor(25.0 * SCALING_FACTOR);
int CAR_LENGTH = std::floor(45.0 * SCALING_FACTOR);

const double WHEEL_RADIUS = 0.33;
const double WHEEL_FRICTION = 1.0;
const double WHEEL_MASS = 20.0;
const double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;


const double TIRE_PEAK_SLIP_ANGLE = 8.0 * DEG_TO_RAD;
const double TIRE_TRANSITION_SLIP_ANGLE = 15.0 * DEG_TO_RAD;
const double TIRE_SLIDE_RATIO = 0.75;
const double TIRE_LOW_SPEED_THRESHOLD = 0.5;
const double TIRE_SLIP_SETPOINT = 0.1;

const double CAR_POWER = 4000.0;
const double BRAKING_POWER = 4000.0;

const double CAR_MASS = 1200.0;
const double CAR_WEIGHT = CAR_MASS * 9.81;
double CAR_MOMENT_OF_INERTIA = (CAR_MASS / 12.0) * (
    (CAR_WIDTH / 10.0) * (CAR_WIDTH / 10.0) +
    (CAR_LENGTH / 10.0) * (CAR_LENGTH / 10.0)
);
const double CAR_TOP_SPEED = 50.0;

const double STEERING_RACK = 1.5;
const double MAX_STEERING_ANGLE = 55.0 * DEG_TO_RAD;
const double FORCE_FEEDBACK_DECAY = 0.97;

double WHEELBASE = (CAR_LENGTH / PIXELS_PER_METER);
double TRACK_WIDTH = (CAR_WIDTH / PIXELS_PER_METER);
const double CG_HEIGHT = 0.5;

void initializeScreenDependentConstants(int screenWidth, int screenHeight) {
    SDL_WINDOW_WIDTH = screenWidth;
    SDL_WINDOW_LENGTH = screenHeight;
    CENTER_X = SDL_WINDOW_WIDTH / 2.0;
    CENTER_Y = SDL_WINDOW_LENGTH / 2.0;

    // Recalculate car size based on screen dimensions
    // Use a fraction of screen size for car dimensions (scaled up by ~2x)
    CAR_WIDTH = std::floor(std::min(screenWidth, screenHeight) * 0.025);  // ~2.5% of smaller dimension (was 1.3%)
    CAR_LENGTH = std::floor(CAR_WIDTH * 1.8);  // Maintain aspect ratio

    // Recalculate dependent constants
    WHEELBASE = (CAR_LENGTH / PIXELS_PER_METER);
    TRACK_WIDTH = (CAR_WIDTH / PIXELS_PER_METER);
    CAR_MOMENT_OF_INERTIA = (CAR_MASS / 12.0) * (
        (CAR_WIDTH / 10.0) * (CAR_WIDTH / 10.0) +
        (CAR_LENGTH / 10.0) * (CAR_LENGTH / 10.0)
    );
}
}