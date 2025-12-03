#include <SDL2/SDL.h>
#include <iostream>

#include "Car.h"
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

    //TODO: redo this mess
    Car car{Constants::CENTER_X, Constants::CENTER_Y, Constants::CAR_WIDTH, Constants::CAR_LENGTH};



    for (int i =0; i < 10000; i++) {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_KEYDOWN:
                    switch (event.key.keysym.sym) {
                        case SDLK_w:
                            car.applyEngineTorque();
                            break;
                        case SDLK_s:
                            car.applyBrakes();
                            break;

                        case SDLK_a:
                            car.applySteering(-50);
                            break;
                        case SDLK_d:
                            car.applySteering(50);
                            break;
                    }
                    break;

                case SDL_KEYUP:
                    break;




                case SDL_QUIT:
                    break;
            }
        }
        car.eraseCar(renderer);
        car.drawCar(renderer);
        SDL_Delay(Constants::SDL_TIME_INTERVAL);

        car.sumWheelForces();
        car.incrementTime(Constants::TIME_INTERVAL);
        car.moveWheels();



    }



    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
