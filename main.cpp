#include <SDL2/SDL.h>
#include <iostream>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "vehicle/Car.h"
#include "ui/GUI.h"
#include "rendering/Camera.h"
#include "rendering/Ground.h"
#include <Eigen/Dense>

#include "config/PhysicsConstants.h"
#include "config/RenderingConstants.h"

struct GameState {
    SDL_Window* win;
    SDL_Renderer* renderer;
    Car* car;
    Camera* camera;
    Ground* ground;
    GUI* gui;
    bool running;
};

GameState* g_gameState = nullptr;

void mainLoop() {
    if (!g_gameState || !g_gameState->running) {
#ifdef __EMSCRIPTEN__
        emscripten_cancel_main_loop();
#endif
        return;
    }

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            g_gameState->running = false;
        } else if (event.type == SDL_KEYDOWN) {
            if (event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q) {
                g_gameState->running = false;
            } else if (event.key.keysym.sym == SDLK_v) {
                g_gameState->car->showDebugVectors = !g_gameState->car->showDebugVectors;
            } else if (event.key.keysym.sym == SDLK_h) {
                g_gameState->gui->toggleHUD();
            } else if (event.key.keysym.sym == SDLK_g) {
                g_gameState->gui->toggleGraphs();
            } else if (event.key.keysym.sym == SDLK_e) {
                g_gameState->car->shiftUp();
            } else if (event.key.keysym.sym == SDLK_c) {
                g_gameState->car->shiftDown();
            }
        }
    }

    const Uint8* keystate = SDL_GetKeyboardState(NULL);

    double throttle = 0.0;
    double brake = 0.0;
    double steering = 0.0;

    if (keystate[SDL_SCANCODE_W]) {
        throttle = 1.0;
    }
    if (keystate[SDL_SCANCODE_S]) {
        brake = 1.0;
    }
    if (keystate[SDL_SCANCODE_A]) {
        steering = 1.0;
    }
    if (keystate[SDL_SCANCODE_D]) {
        steering = -1.0;
    }

    g_gameState->car->setThrottle(throttle);
    g_gameState->car->setBrake(brake);
    g_gameState->car->setSteering(steering);

    if (keystate[SDL_SCANCODE_LSHIFT]) {
        g_gameState->car->holdClutch();
    } else {
        g_gameState->car->releaseClutch();
    }

    g_gameState->car->updateInputs(PhysicsConstants::TIME_INTERVAL);
    g_gameState->car->updateEngine(throttle);
    g_gameState->car->applyBrakes();
    g_gameState->car->sumWheelForces();
    g_gameState->car->updateAcceleration();

    g_gameState->gui->updateGraphs(*g_gameState->car, g_gameState->car->actualThrottle, g_gameState->car->actualBrake, g_gameState->car->actualSteering);

    g_gameState->camera->followTargetSmooth(g_gameState->car->pos_x, g_gameState->car->pos_y);

    g_gameState->car->eraseCar(g_gameState->renderer);
    g_gameState->ground->draw(g_gameState->renderer, g_gameState->camera, RenderingConstants::SDL_WINDOW_WIDTH, RenderingConstants::SDL_WINDOW_LENGTH);
    g_gameState->car->drawCar(g_gameState->renderer, g_gameState->camera);
    g_gameState->gui->drawHUD(g_gameState->renderer, *g_gameState->car, g_gameState->car->actualThrottle);
    SDL_RenderPresent(g_gameState->renderer);

    g_gameState->car->incrementTime(PhysicsConstants::TIME_INTERVAL);
    g_gameState->car->moveWheels();
}

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

#ifndef __EMSCRIPTEN__
    if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
        std::cerr << "SLD_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }
#endif

    int screenWidth;
    int screenHeight;

#ifdef __EMSCRIPTEN__
    screenWidth = 1280;
    screenHeight = 720;
#else
    SDL_DisplayMode displayMode;
    if (SDL_GetCurrentDisplayMode(0, &displayMode) != 0) {
        std::cerr << "SDL_GetCurrentDisplayMode Error: " << SDL_GetError() << std::endl;
        return 1;
    }
    screenWidth = displayMode.w;
    screenHeight = displayMode.h;
#endif

    RenderingConstants::initializeScreenDependentConstants(screenWidth, screenHeight);

#ifdef __EMSCRIPTEN__
    SDL_Window* win = SDL_CreateWindow("Car Game",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        screenWidth, screenHeight,
        SDL_WINDOW_SHOWN);
#else
    SDL_Window* win = SDL_CreateWindow("Car Game",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        RenderingConstants::SDL_WINDOW_WIDTH, RenderingConstants::SDL_WINDOW_LENGTH,
        SDL_WINDOW_FULLSCREEN_DESKTOP);
#endif

    if (!win) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    if (!renderer) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
    }

#ifndef __EMSCRIPTEN__
    SDL_Joystick *joystick;
    SDL_JoystickEventState(SDL_ENABLE);
    joystick = SDL_JoystickOpen(0);
#endif

    Car* car = new Car(RenderingConstants::CENTER_X, RenderingConstants::CENTER_Y, RenderingConstants::CAR_WIDTH, RenderingConstants::CAR_LENGTH);
    Camera* camera = new Camera(car->pos_x, car->pos_y, 0.1);
    Ground* ground = new Ground(100);

    GUI* gui = new GUI();
    int fontSize = std::max(12, screenHeight / 60);
    if (!gui->initialize(nullptr, fontSize)) {
        std::cerr << "Warning: Failed to initialize GUI" << std::endl;
    }

    g_gameState = new GameState{win, renderer, car, camera, ground, gui, true};

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop(mainLoop, 60, 1);
#else
    while (g_gameState->running) {
        mainLoop();
        SDL_Delay(PhysicsConstants::SDL_TIME_INTERVAL);
    }
#endif

    delete gui;
    delete ground;
    delete camera;
    delete car;
    delete g_gameState;

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
