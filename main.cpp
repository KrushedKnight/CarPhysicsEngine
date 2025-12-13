#include <SDL2/SDL.h>
#include <iostream>

#include "Car.h"
#include "GUI.h"
#include <Eigen/Dense>

#include "constants.h"

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
        std::cerr << "SLD_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

SDL_Window* win = SDL_CreateWindow("Car Game", Constants::SDL_WINDOW_X, Constants::SDL_WINDOW_Y, Constants::SDL_WINDOW_WIDTH, Constants::SDL_WINDOW_LENGTH, SDL_WINDOW_SHOWN);
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

    Car car{Constants::CENTER_X, Constants::CENTER_Y, Constants::CAR_WIDTH, Constants::CAR_LENGTH};

    // Initialize GUI
    GUI gui;
    if (!gui.initialize()) {
        std::cerr << "Warning: Failed to initialize GUI" << std::endl;
    }

    const double STEERING_INCREMENT = 2.0 * Constants::DEG_TO_RAD;

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
                }
            }
        }

        const Uint8* keystate = SDL_GetKeyboardState(NULL);

        if (keystate[SDL_SCANCODE_W]) {
            car.applyEngineTorque();
        }
        if (keystate[SDL_SCANCODE_S]) {
            car.applyBrakes();
        }
        if (keystate[SDL_SCANCODE_A]) {
            car.applySteering(STEERING_INCREMENT);
        }
        if (keystate[SDL_SCANCODE_D]) {
            car.applySteering(-STEERING_INCREMENT);
        }

        car.sumWheelForces();

        car.eraseCar(renderer);
        car.drawCar(renderer);
        gui.drawHUD(renderer, car);
        SDL_RenderPresent(renderer);

        car.incrementTime(Constants::TIME_INTERVAL);
        car.moveWheels();

        SDL_Delay(Constants::SDL_TIME_INTERVAL);
    }

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}