#ifndef GUI_H
#define GUI_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <vector>
#include <map>
#include "vehicle/Car.h"
#include "ui/Graph.h"
#include "ui/Dial.h"

class GUI {
public:
    GUI();
    ~GUI();

    bool initialize(const char* fontPath = nullptr, int fontSize = 16);

    void drawHUD(SDL_Renderer* renderer, const Car& car, double throttle);

    void updateGraphs(const Car& car, double throttle, double brake, double steering);

    void drawText(SDL_Renderer* renderer, const std::string& text, int x, int y,
                  SDL_Color color = {255, 255, 255, 255});

    void toggleHUD();
    bool isVisible() const { return visible; }

    void toggleGraphs();
    bool areGraphsVisible() const { return showGraphs; }

    void toggleDials();
    bool areDialsVisible() const { return showDials; }

private:
    TTF_Font* font;
    TTF_Font* dialFont;
    bool visible;
    bool showGraphs;
    bool showDials;
    int fontSize;

    struct TextCacheEntry {
        SDL_Texture* texture;
        int width;
        int height;
    };
    std::map<std::string, TextCacheEntry> textCache;
    void clearTextCache();

    double currentThrottle;
    double currentBrake;
    double currentSteering;
    double currentClutch;

    std::vector<Graph> graphs;
    Dial rpmDial;
    Dial torqueDial;
    Dial airFlowDial;
    Dial manifoldPressureDial;
    Dial speedDial;
    Dial volEffDial;
    Dial afrDial;
    Dial powerDial;

    std::vector<std::string> formatCarStats(const Car& car, double throttle);

    void drawGraphs(SDL_Renderer* renderer);
    void drawDials(SDL_Renderer* renderer, const Car& car);
    void drawInputSliders(SDL_Renderer* renderer);
};

#endif
