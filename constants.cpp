//
// Created by beast-machine-2 on 7/8/25.
//

#include <cmath>
#include "constants.h"

namespace Constants {

// Math constants
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

// Time constants - MUST MATCH for consistent physics
const double TIME_INTERVAL = 0.016;                    // 16ms = 62.5 FPS
const int SDL_TIME_INTERVAL = 16;                      // 16ms (matches physics timestep)

// Display constants
const double SCALING_FACTOR = 1.0;
const int SDL_WINDOW_X = 640.0 * SCALING_FACTOR;
const int SDL_WINDOW_Y = 480.0 * SCALING_FACTOR;
const int SDL_WINDOW_WIDTH = 640.0 * SCALING_FACTOR;
const int SDL_WINDOW_LENGTH = 480.0 * SCALING_FACTOR;
const double CENTER_X = SDL_WINDOW_WIDTH / 2.0;
const double CENTER_Y = SDL_WINDOW_LENGTH / 2.0;

// Car dimensions (pixels for rendering, approximate 1:10 scale from meters)
const int CAR_WIDTH = std::floor(25.0 * SCALING_FACTOR);   // ~2.5m real width
const int CAR_LENGTH = std::floor(45.0 * SCALING_FACTOR);  // ~4.5m real length

// Distance from car center to wheel (pixels) - half diagonal for corner-mounted wheels
const double DIST_TO_WHEEL = std::sqrt(CAR_WIDTH * CAR_WIDTH + CAR_LENGTH * CAR_LENGTH) / 2.0;

// Wheel physical properties (realistic scaled values)
const double WHEEL_RADIUS = 0.33;                      // 0.33m = 330mm (realistic car wheel)
const double WHEEL_FRICTION = 0.7;                     // Dry asphalt tire friction
const double WHEEL_MASS = 20.0;                        // 20kg (wheel + tire)
const double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS; // I = 0.5*m*r² (solid disk)

// Engine and braking (torque values in N⋅m)
const double CAR_POWER = 200.0;                        // 200 N⋅m engine torque
const double BRAKING_POWER = 150.0;                    // 150 N⋅m braking torque (positive, sign applied in code)

// Car physical properties
const double CAR_MASS = 1200.0;                        // 1200kg (realistic mid-size car)
const double CAR_WEIGHT = CAR_MASS * 9.81;             // Weight = mass × gravity
// Moment of inertia for rectangular plate: I = (m/12) * (w² + h²)
// Using physics meters (pixels / 10) for dimensional consistency
const double CAR_MOMENT_OF_INERTIA = (CAR_MASS / 12.0) * (
    (CAR_WIDTH / 10.0) * (CAR_WIDTH / 10.0) +
    (CAR_LENGTH / 10.0) * (CAR_LENGTH / 10.0)
);
const double CAR_TOP_SPEED = 50.0;                     // 50 m/s = 180 km/h

// Steering properties
const double STEERING_RACK = 1.0;                      // 1:1 input to wheel angle ratio
const double MAX_STEERING_ANGLE = 35.0 * DEG_TO_RAD;   // 35 degrees max wheel angle (realistic)
const double FORCE_FEEDBACK_DECAY = 0.95;              // 5% centering force per timestep
}

