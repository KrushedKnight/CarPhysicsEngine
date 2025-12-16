#ifndef GUI_H
#define GUI_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <vector>
#include "vehicle/Car.h"
#include "ui/Graph.h"

class GUI {
public:
    GUI();
    ~GUI();

    bool initialize(const char* fontPath = nullptr, int fontSize = 16);

    void drawHUD(SDL_Renderer* renderer, const Car& car);

    void updateGraphs(const Car& car, double throttle, double brake, double steering);

    void drawText(SDL_Renderer* renderer, const std::string& text, int x, int y,
                  SDL_Color color = {255, 255, 255, 255});

    void toggleHUD();
    bool isVisible() const { return visible; }

    void toggleGraphs();
    bool areGraphsVisible() const { return showGraphs; }

private:
    TTF_Font* font;
    bool visible;
    bool showGraphs;
    int fontSize;

    std::vector<Graph> graphs;

    std::vector<std::string> formatCarStats(const Car& car);

    void drawGraphs(SDL_Renderer* renderer);
};

#endif
