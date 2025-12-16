#ifndef GUI_H
#define GUI_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <vector>
#include <deque>
#include "Car.h"

struct GraphData {
    std::deque<double> values;
    std::string label;
    SDL_Color color;
    double minValue;
    double maxValue;
    size_t maxPoints;

    GraphData(const std::string& lbl, SDL_Color col, double minVal, double maxVal, size_t maxPts = 150)
        : label(lbl), color(col), minValue(minVal), maxValue(maxVal), maxPoints(maxPts) {}

    void addPoint(double value) {
        values.push_back(value);
        if (values.size() > maxPoints) {
            values.pop_front();
        }
    }

    void clear() {
        values.clear();
    }
};

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

    std::vector<GraphData> graphs;

    std::vector<std::string> formatCarStats(const Car& car);

    void drawGraph(SDL_Renderer* renderer, const GraphData& graph, int x, int y, int width, int height);

    void drawGraphs(SDL_Renderer* renderer);
};

#endif