#include <SDL2/SDL.h>
#include <iostream>

#include "vehicle/Car.h"
#include "ui/GUI.h"
#include <Eigen/Dense>

#include "config/PhysicsConstants.h"
#include "config/RenderingConstants.h"

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
        std::cerr << "SLD_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_DisplayMode displayMode;
    if (SDL_GetCurrentDisplayMode(0, &displayMode) != 0) {
        std::cerr << "SDL_GetCurrentDisplayMode Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    int screenWidth = displayMode.w;
    int screenHeight = displayMode.h;

    RenderingConstants::initializeScreenDependentConstants(screenWidth, screenHeight);

    SDL_Window* win = SDL_CreateWindow("Car Game",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        RenderingConstants::SDL_WINDOW_WIDTH, RenderingConstants::SDL_WINDOW_LENGTH,
        SDL_WINDOW_FULLSCREEN_DESKTOP);
    if (!win) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    if (!renderer) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
    }

    SDL_Joystick *joystick;

    SDL_JoystickEventState(SDL_ENABLE);
    joystick = SDL_JoystickOpen(0);

    SDL_Event event;

    Car car{RenderingConstants::CENTER_X, RenderingConstants::CENTER_Y, RenderingConstants::CAR_WIDTH, RenderingConstants::CAR_LENGTH};

    GUI gui;
    int fontSize = std::max(12, screenHeight / 60);
    if (!gui.initialize(nullptr, fontSize)) {
        std::cerr << "Warning: Failed to initialize GUI" << std::endl;
    }

    const double STEERING_INCREMENT = 2.0 * PhysicsConstants::DEG_TO_RAD;

    bool running = true;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q) {
                    running = false;
                } else if (event.key.keysym.sym == SDLK_v) {
                    car.showDebugVectors = !car.showDebugVectors;
                } else if (event.key.keysym.sym == SDLK_h) {
                    gui.toggleHUD();
                } else if (event.key.keysym.sym == SDLK_g) {
                    gui.toggleGraphs();
                }
            }
        }

        const Uint8* keystate = SDL_GetKeyboardState(NULL);

        double throttle = 0.0;
        double brake = 0.0;
        double steering = 0.0;

        if (keystate[SDL_SCANCODE_W]) {
            car.applyEngineTorque();
            throttle = 1.0;
        }
        if (keystate[SDL_SCANCODE_S]) {
            car.applyBrakes();
            brake = 1.0;
        }
        if (keystate[SDL_SCANCODE_A]) {
            car.applySteering(STEERING_INCREMENT);
            steering = 1.0;
        }
        if (keystate[SDL_SCANCODE_D]) {
            car.applySteering(-STEERING_INCREMENT);
            steering = -1.0;
        }

        car.sumWheelForces();

        gui.updateGraphs(car, throttle, brake, steering);

        car.eraseCar(renderer);
        car.drawCar(renderer);
        gui.drawHUD(renderer, car);
        SDL_RenderPresent(renderer);

        car.incrementTime(PhysicsConstants::TIME_INTERVAL);
        car.moveWheels();

        SDL_Delay(PhysicsConstants::SDL_TIME_INTERVAL);
    }

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}