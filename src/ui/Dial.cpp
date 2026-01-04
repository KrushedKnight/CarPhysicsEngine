#include "ui/Dial.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

Dial::Dial(double minValue, double maxValue, const std::string& label, const std::string& unit)
    : minValue(minValue), maxValue(maxValue), currentValue(minValue), displayValue(minValue),
      label(label), unit(unit), startAngle(-225.0 * M_PI / 180.0), endAngle(45.0 * M_PI / 180.0),
      smoothingFactor(0.15), backgroundTexture(nullptr), cachedRadius(0),
      labelTexture(nullptr), labelWidth(0), labelHeight(0),
      unitTexture(nullptr), unitWidth(0), unitHeight(0) {
}

Dial::~Dial() {
    clearCache();
}

void Dial::clearCache() {
    if (backgroundTexture != nullptr) {
        SDL_DestroyTexture(backgroundTexture);
        backgroundTexture = nullptr;
    }
    if (labelTexture != nullptr) {
        SDL_DestroyTexture(labelTexture);
        labelTexture = nullptr;
    }
    if (unitTexture != nullptr) {
        SDL_DestroyTexture(unitTexture);
        unitTexture = nullptr;
    }
    for (auto& entry : valueCache) {
        if (entry.second != nullptr) {
            SDL_DestroyTexture(entry.second);
        }
    }
    valueCache.clear();
}

void Dial::setValue(double value) {
    currentValue = std::clamp(value, minValue, maxValue);
    displayValue = lerp(displayValue, currentValue, smoothingFactor);
}

double Dial::lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

double Dial::valueToAngle(double value) {
    double normalized = (value - minValue) / (maxValue - minValue);
    return startAngle + normalized * (endAngle - startAngle);
}

void Dial::drawArc(SDL_Renderer* renderer, int centerX, int centerY, int radius,
                   double startAngle, double endAngle, int segments) {
    for (int i = 0; i < segments; i++) {
        double angle1 = startAngle + (endAngle - startAngle) * i / segments;
        double angle2 = startAngle + (endAngle - startAngle) * (i + 1) / segments;

        int x1 = centerX + static_cast<int>(radius * cos(angle1));
        int y1 = centerY + static_cast<int>(radius * sin(angle1));
        int x2 = centerX + static_cast<int>(radius * cos(angle2));
        int y2 = centerY + static_cast<int>(radius * sin(angle2));

        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    }
}

void Dial::drawTicks(SDL_Renderer* renderer, int centerX, int centerY, int radius) {
    double range = maxValue - minValue;
    int majorTickInterval = 1000;
    int minorTickInterval = 250;

    if (range <= 100) {
        majorTickInterval = 20;
        minorTickInterval = 5;
    } else if (range <= 500) {
        majorTickInterval = 100;
        minorTickInterval = 25;
    } else if (range <= 2000) {
        majorTickInterval = 500;
        minorTickInterval = 100;
    }

    for (double value = minValue; value <= maxValue; value += minorTickInterval) {
        double angle = valueToAngle(value);
        bool isMajor = (static_cast<int>(value) % majorTickInterval == 0);

        int tickLength = isMajor ? 15 : 8;
        int tickWidth = isMajor ? 2 : 1;
        int innerRadius = radius - tickLength;

        int x1 = centerX + static_cast<int>(innerRadius * cos(angle));
        int y1 = centerY + static_cast<int>(innerRadius * sin(angle));
        int x2 = centerX + static_cast<int>(radius * cos(angle));
        int y2 = centerY + static_cast<int>(radius * sin(angle));

        SDL_Color tickColor = isMajor ? SDL_Color{220, 220, 220, 255} : SDL_Color{140, 140, 140, 180};
        SDL_SetRenderDrawColor(renderer, tickColor.r, tickColor.g, tickColor.b, tickColor.a);

        for (int w = 0; w < tickWidth; w++) {
            SDL_RenderDrawLine(renderer, x1 + w, y1, x2 + w, y2);
        }
    }

    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 100);
    drawArc(renderer, centerX, centerY, radius - 2, startAngle, endAngle, 80);
    drawArc(renderer, centerX, centerY, radius - 25, startAngle, endAngle, 80);
}

void Dial::drawNumbers(SDL_Renderer* renderer, int centerX, int centerY, int radius, TTF_Font* font) {
    if (font == nullptr) return;

    double range = maxValue - minValue;
    int numberInterval = 1000;

    if (range <= 100) {
        numberInterval = 20;
    } else if (range <= 500) {
        numberInterval = 100;
    } else if (range <= 2000) {
        numberInterval = 500;
    }

    for (double value = minValue; value <= maxValue; value += numberInterval) {
        double angle = valueToAngle(value);
        int textRadius = radius - 35;

        int x = centerX + static_cast<int>(textRadius * cos(angle));
        int y = centerY + static_cast<int>(textRadius * sin(angle));

        std::ostringstream oss;
        oss << static_cast<int>(value);
        std::string text = oss.str();

        SDL_Color numberColor = {200, 200, 200, 255};
        drawText(renderer, text, x - 10, y - 8, numberColor, font);
    }
}

void Dial::drawNeedle(SDL_Renderer* renderer, int centerX, int centerY, int radius) {
    double angle = valueToAngle(displayValue);
    int needleLength = radius - 20;
    int needleTailLength = 20;

    int tipX = centerX + static_cast<int>(needleLength * cos(angle));
    int tipY = centerY + static_cast<int>(needleLength * sin(angle));
    int tailX = centerX - static_cast<int>(needleTailLength * cos(angle));
    int tailY = centerY - static_cast<int>(needleTailLength * sin(angle));

    SDL_SetRenderDrawColor(renderer, 255, 100, 80, 255);
    SDL_RenderDrawLine(renderer, tailX, tailY, tipX, tipY);
    SDL_RenderDrawLine(renderer, tailX + 1, tailY, tipX + 1, tipY);

    SDL_Rect centerRect = {centerX - 4, centerY - 4, 8, 8};
    SDL_SetRenderDrawColor(renderer, 255, 100, 80, 255);
    SDL_RenderFillRect(renderer, &centerRect);

    SDL_Rect innerRect = {centerX - 2, centerY - 2, 4, 4};
    SDL_SetRenderDrawColor(renderer, 40, 40, 40, 255);
    SDL_RenderFillRect(renderer, &innerRect);
}

void Dial::drawLabel(SDL_Renderer* renderer, int centerX, int centerY, int radius, TTF_Font* font) {
    if (font == nullptr || label.empty()) return;

    if (labelTexture == nullptr) {
        SDL_Color labelColor = {160, 160, 160, 255};
        SDL_Surface* surface = TTF_RenderText_Blended(font, label.c_str(), labelColor);
        if (surface != nullptr) {
            labelTexture = SDL_CreateTextureFromSurface(renderer, surface);
            labelWidth = surface->w;
            labelHeight = surface->h;
            SDL_FreeSurface(surface);
        }
    }

    if (labelTexture != nullptr) {
        int labelY = centerY + radius / 3;
        SDL_Rect dstRect = {centerX - labelWidth / 2, labelY, labelWidth, labelHeight};
        SDL_RenderCopy(renderer, labelTexture, nullptr, &dstRect);
    }
}

void Dial::drawReadout(SDL_Renderer* renderer, int centerX, int centerY, int radius, TTF_Font* font) {
    if (font == nullptr) return;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(0) << displayValue;
    std::string valueText = oss.str();

    int readoutY = centerY + radius / 2;

    if (valueText != lastValueText) {
        auto it = valueCache.find(valueText);
        if (it == valueCache.end()) {
            SDL_Color readoutColor = {240, 240, 240, 255};
            SDL_Surface* valueSurface = TTF_RenderText_Blended(font, valueText.c_str(), readoutColor);
            if (valueSurface != nullptr) {
                SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, valueSurface);
                valueCache[valueText] = tex;
                SDL_FreeSurface(valueSurface);
            }
        }
        lastValueText = valueText;
    }

    auto it = valueCache.find(valueText);
    if (it != valueCache.end() && it->second != nullptr) {
        int w, h;
        SDL_QueryTexture(it->second, nullptr, nullptr, &w, &h);
        SDL_Rect valueRect = {centerX - w / 2, readoutY, w, h};
        SDL_RenderCopy(renderer, it->second, nullptr, &valueRect);
    }

    if (!unit.empty()) {
        if (unitTexture == nullptr) {
            SDL_Color unitColor = {140, 140, 140, 255};
            SDL_Surface* unitSurface = TTF_RenderText_Blended(font, unit.c_str(), unitColor);
            if (unitSurface != nullptr) {
                unitTexture = SDL_CreateTextureFromSurface(renderer, unitSurface);
                unitWidth = unitSurface->w;
                unitHeight = unitSurface->h;
                SDL_FreeSurface(unitSurface);
            }
        }

        if (unitTexture != nullptr) {
            int unitY = readoutY + 15;
            SDL_Rect unitRect = {centerX - unitWidth / 2, unitY, unitWidth, unitHeight};
            SDL_RenderCopy(renderer, unitTexture, nullptr, &unitRect);
        }
    }
}

void Dial::drawText(SDL_Renderer* renderer, const std::string& text, int x, int y,
                    SDL_Color color, TTF_Font* font) {
    if (font == nullptr || text.empty()) return;

    SDL_Surface* surface = TTF_RenderText_Blended(font, text.c_str(), color);
    if (surface == nullptr) return;

    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (texture == nullptr) {
        SDL_FreeSurface(surface);
        return;
    }

    SDL_Rect dstrect = {x, y, surface->w, surface->h};
    SDL_RenderCopy(renderer, texture, nullptr, &dstrect);

    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);
}

void Dial::renderBackground(SDL_Renderer* renderer, int radius, TTF_Font* font) {
    int size = radius * 2 + 10;
    backgroundTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                          SDL_TEXTUREACCESS_TARGET, size, size);
    SDL_SetTextureBlendMode(backgroundTexture, SDL_BLENDMODE_BLEND);

    SDL_SetRenderTarget(renderer, backgroundTexture);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);

    int center = size / 2;
    drawTicks(renderer, center, center, radius);
    drawNumbers(renderer, center, center, radius, font);

    SDL_SetRenderTarget(renderer, nullptr);
    cachedRadius = radius;
}

void Dial::draw(SDL_Renderer* renderer, int centerX, int centerY, int radius, TTF_Font* font) {
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    if (backgroundTexture == nullptr || cachedRadius != radius) {
        if (backgroundTexture != nullptr) {
            SDL_DestroyTexture(backgroundTexture);
        }
        renderBackground(renderer, radius, font);
    }

    if (backgroundTexture != nullptr) {
        int size = radius * 2 + 10;
        SDL_Rect destRect = {centerX - size/2, centerY - size/2, size, size};
        SDL_RenderCopy(renderer, backgroundTexture, nullptr, &destRect);
    }

    drawNeedle(renderer, centerX, centerY, radius);
    drawLabel(renderer, centerX, centerY, radius, font);
    drawReadout(renderer, centerX, centerY, radius, font);
}