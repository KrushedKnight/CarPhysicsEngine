//
// Created by beast-machine-2 on 7/6/25.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H

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

    int CAR_POWER = 1;


    Eigen::Vector2d UP_VECTOR{0, 1};
    Eigen::Vector2d DOWN_VECTOR{0, -1};
    Eigen::Vector2d LEFT_VECTOR{-1, 0};
    Eigen::Vector2d RIGHT_VECTOR{1, 0};
    Eigen::Vector2d ZERO_VECTOR{0, 0};
}
#endif //CONSTANTS_H
