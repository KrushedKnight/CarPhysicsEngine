#ifndef GRAPH_H
#define GRAPH_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <deque>

class Graph {
public:
    Graph(const std::string& label, SDL_Color color, double minValue, double maxValue, size_t maxPoints = 150);

    void addDataPoint(double value);
    void clear();
    void render(SDL_Renderer* renderer, int x, int y, int width, int height, TTF_Font* font);

private:
    std::deque<double> values;
    std::string label;
    SDL_Color color;
    double minValue;
    double maxValue;
    size_t maxPoints;
};

#endif
