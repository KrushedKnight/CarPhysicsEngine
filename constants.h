//
// Created by beast-machine-2 on 7/6/25.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
#include <Eigen/Core>

namespace Constants {
    // Time constants
    extern const double TIME_INTERVAL;          // Physics timestep (seconds)
    extern const int SDL_TIME_INTERVAL;         // Render delay (milliseconds)

    // Display constants
    extern const double SCALING_FACTOR;         // Visual scaling factor
    extern const int SDL_WINDOW_X;              // Window position X (pixels)
    extern const int SDL_WINDOW_Y;              // Window position Y (pixels)
    extern const int SDL_WINDOW_WIDTH;          // Window width (pixels)
    extern const int SDL_WINDOW_LENGTH;         // Window height (pixels)
    extern const double CENTER_X;               // Window center X (pixels)
    extern const double CENTER_Y;               // Window center Y (pixels)

    // Car dimensions (pixels for rendering)
    extern const int CAR_WIDTH;
    extern const int CAR_LENGTH;
    extern const double DIST_TO_WHEEL;          // Distance from center to wheel (pixels)

    // Wheel physical properties
    extern const double WHEEL_RADIUS;           // Wheel radius (meters)
    extern const double WHEEL_FRICTION;         // Tire-road friction coefficient (dimensionless)
    extern const double WHEEL_MASS;             // Wheel mass (kg)
    extern const double WHEEL_MOMENT_OF_INERTIA; // Wheel rotational inertia (kg⋅m²)

    // Engine and braking
    extern const double CAR_POWER;              // Engine force (N)
    extern const double BRAKING_POWER;          // Braking force (N)

    // Car physical properties
    extern const double CAR_MASS;               // Car body mass (kg)
    extern const double CAR_WEIGHT;             // Car weight force (N)
    extern const double CAR_MOMENT_OF_INERTIA;  // Car rotational inertia (kg⋅m²)
    extern const double CAR_TOP_SPEED;          // Maximum speed (m/s)

    // Steering properties
    extern const double STEERING_RACK;          // Steering ratio (dimensionless)
    extern const double MAX_STEERING_ANGLE;     // Maximum wheel angle (radians)
    extern const double FORCE_FEEDBACK_DECAY;   // Steering return rate (dimensionless)

    // Math constants
    extern const double DEG_TO_RAD;             // Degree to radian conversion
    extern const double RAD_TO_DEG;             // Radian to degree conversion

    // Unit conversion
    extern const double PIXELS_PER_METER;       // Pixels per meter (for physics/rendering conversion)
}
#endif //CONSTANTS_H
