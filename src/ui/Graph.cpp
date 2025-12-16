#include "ui/Graph.h"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>

Graph::Graph(const std::string& label, SDL_Color color, double minValue, double maxValue, size_t maxPoints)
    : label(label), color(color), minValue(minValue), maxValue(maxValue), maxPoints(maxPoints) {}

void Graph::addDataPoint(double value) {
    values.push_back(value);
    if (values.size() > maxPoints) {
        values.pop_front();
    }
}

void Graph::clear() {
    values.clear();
}

void Graph::render(SDL_Renderer* renderer, int x, int y, int width, int height, TTF_Font* font) {
    if (values.empty()) return;

    SDL_Rect panel = {x, y, width, height};
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 180);
    SDL_RenderFillRect(renderer, &panel);

    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawRect(renderer, &panel);

    int textPadding = std::max(3, width / 60);

    if (font) {
        SDL_Surface* surface = TTF_RenderText_Blended(font, label.c_str(), {200, 200, 200, 255});
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
            if (texture) {
                SDL_Rect dstrect = {x + textPadding, y + textPadding, surface->w, surface->h};
                SDL_RenderCopy(renderer, texture, nullptr, &dstrect);
                SDL_DestroyTexture(texture);
            }
            SDL_FreeSurface(surface);
        }
    }

    if (minValue < 0 && maxValue > 0) {
        double zeroY = y + height - ((-minValue) / (maxValue - minValue)) * height;
        SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
        SDL_RenderDrawLine(renderer, x, static_cast<int>(zeroY), x + width, static_cast<int>(zeroY));
    }

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    double xStep = static_cast<double>(width) / maxPoints;
    double range = maxValue - minValue;

    for (size_t i = 1; i < values.size(); i++) {
        double val1 = std::clamp(values[i - 1], minValue, maxValue);
        double val2 = std::clamp(values[i], minValue, maxValue);

        double norm1 = (val1 - minValue) / range;
        double norm2 = (val2 - minValue) / range;

        int x1 = x + static_cast<int>((i - 1) * xStep);
        int y1 = y + height - static_cast<int>(norm1 * height);
        int x2 = x + static_cast<int>(i * xStep);
        int y2 = y + height - static_cast<int>(norm2 * height);

        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    }

    if (!values.empty() && font) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << values.back();
        int valueTextOffset = std::max(30, width / 6);

        SDL_Surface* surface = TTF_RenderText_Blended(font, oss.str().c_str(), color);
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
            if (texture) {
                SDL_Rect dstrect = {x + width - valueTextOffset, y + textPadding, surface->w, surface->h};
                SDL_RenderCopy(renderer, texture, nullptr, &dstrect);
                SDL_DestroyTexture(texture);
            }
            SDL_FreeSurface(surface);
        }
    }
}
