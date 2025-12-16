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

    extern int SDL_WINDOW_WIDTH;
    extern int SDL_WINDOW_LENGTH;
    extern double CENTER_X;
    extern double CENTER_Y;

    extern int CAR_WIDTH;
    extern int CAR_LENGTH;

    extern const double WHEEL_RADIUS;
    extern const double WHEEL_FRICTION;
    extern const double WHEEL_MASS;
    extern const double WHEEL_MOMENT_OF_INERTIA;

    extern const double TIRE_PEAK_SLIP_ANGLE;
    extern const double TIRE_TRANSITION_SLIP_ANGLE;
    extern const double TIRE_SLIDE_RATIO;
    extern const double TIRE_LOW_SPEED_THRESHOLD;
    extern const double TIRE_SLIP_SETPOINT;
    extern const double TIRE_TCS_kP;
    extern const double TIRE_TCS_kD;
    extern const double ABS_SLIP_SETPOINT;
    extern const double ABS_kP;
    extern const double ABS_kD;

    extern const double CAR_POWER;
    extern const double BRAKING_POWER;

    extern const double CAR_MASS;
    extern const double CAR_WEIGHT;
    extern double CAR_MOMENT_OF_INERTIA;
    extern const double CAR_TOP_SPEED;

    extern const double STEERING_RACK;
    extern const double MAX_STEERING_ANGLE;
    extern const double FORCE_FEEDBACK_DECAY;

    extern double WHEELBASE;
    extern double TRACK_WIDTH;
    extern const double CG_HEIGHT;

    extern const double DEG_TO_RAD;
    extern const double RAD_TO_DEG;

    extern const double PIXELS_PER_METER;

    void initializeScreenDependentConstants(int screenWidth, int screenHeight);
}
#endif
