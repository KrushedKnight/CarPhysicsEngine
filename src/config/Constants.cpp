#include <cmath>
#include <algorithm>
#include "config/RenderingConstants.h"
#include "config/PhysicsConstants.h"

namespace RenderingConstants {
    int SDL_WINDOW_WIDTH = 1920;
    int SDL_WINDOW_LENGTH = 1080;
    double CENTER_X = SDL_WINDOW_WIDTH / 2.0;
    double CENTER_Y = SDL_WINDOW_LENGTH / 2.0;

    int CAR_WIDTH = static_cast<int>(std::floor(25.0 * SCALING_FACTOR));
    int CAR_LENGTH = static_cast<int>(std::floor(45.0 * SCALING_FACTOR));

    double WHEELBASE = (CAR_LENGTH / PhysicsConstants::PIXELS_PER_METER);
    double TRACK_WIDTH = (CAR_WIDTH / PhysicsConstants::PIXELS_PER_METER);
    double CAR_MOMENT_OF_INERTIA = (PhysicsConstants::CAR_MASS / 12.0) * (
        (CAR_WIDTH / 10.0) * (CAR_WIDTH / 10.0) +
        (CAR_LENGTH / 10.0) * (CAR_LENGTH / 10.0)
    );

    void initializeScreenDependentConstants(int screenWidth, int screenHeight) {
        SDL_WINDOW_WIDTH = screenWidth;
        SDL_WINDOW_LENGTH = screenHeight;
        CENTER_X = SDL_WINDOW_WIDTH / 2.0;
        CENTER_Y = SDL_WINDOW_LENGTH / 2.0;

        CAR_WIDTH = std::floor(std::min(screenWidth, screenHeight) * 0.025);
        CAR_LENGTH = std::floor(CAR_WIDTH * 1.8);

        WHEELBASE = (CAR_LENGTH / PhysicsConstants::PIXELS_PER_METER);
        TRACK_WIDTH = (CAR_WIDTH / PhysicsConstants::PIXELS_PER_METER);
        CAR_MOMENT_OF_INERTIA = (PhysicsConstants::CAR_MASS / 12.0) * (
            (CAR_WIDTH / 10.0) * (CAR_WIDTH / 10.0) +
            (CAR_LENGTH / 10.0) * (CAR_LENGTH / 10.0)
        );
    }
}
