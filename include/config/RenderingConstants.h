#ifndef RENDERINGCONSTANTS_H
#define RENDERINGCONSTANTS_H

namespace RenderingConstants {
    constexpr double SCALING_FACTOR = 1.0;
    constexpr int SDL_WINDOW_X = 100;
    constexpr int SDL_WINDOW_Y = 100;

    extern int SDL_WINDOW_WIDTH;
    extern int SDL_WINDOW_LENGTH;
    extern double CENTER_X;
    extern double CENTER_Y;

    extern int CAR_WIDTH;
    extern int CAR_LENGTH;

    extern double WHEELBASE;
    extern double TRACK_WIDTH;
    extern double CAR_MOMENT_OF_INERTIA;

    constexpr double WHEEL_WIDTH_INSET = 0.1;
    constexpr double WHEEL_LENGTH_INSET = 0.15;

    void initializeScreenDependentConstants(int screenWidth, int screenHeight);
}

#endif
