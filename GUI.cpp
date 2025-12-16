#include "GUI.h"
#include "constants.h"
#include <sstream>
#include <iomanip>
#include <cmath>

GUI::GUI() : font(nullptr), visible(true), showGraphs(true), fontSize(16) {
    graphs.push_back(GraphData("Speed (m/s)", {0, 255, 0, 255}, 0.0, 50.0));
    graphs.push_back(GraphData("Throttle/Brake", {255, 165, 0, 255}, -1.0, 1.0));
    graphs.push_back(GraphData("Steering", {100, 200, 255, 255}, -1.0, 1.0));
    graphs.push_back(GraphData("FL Grip", {255, 100, 100, 255}, 0.0, 1.0));
    graphs.push_back(GraphData("FR Grip", {255, 150, 100, 255}, 0.0, 1.0));
    graphs.push_back(GraphData("RL Grip", {200, 100, 255, 255}, 0.0, 1.0));
    graphs.push_back(GraphData("RR Grip", {150, 100, 255, 255}, 0.0, 1.0));
}

GUI::~GUI() {
    if (font != nullptr) {
        TTF_CloseFont(font);
    }
}

bool GUI::initialize(const char* fontPath, int fontSize) {
    this->fontSize = fontSize;

    if (TTF_Init() == -1) {
        SDL_Log("TTF_Init Error: %s", TTF_GetError());
        return false;
    }

    if (fontPath != nullptr) {
        font = TTF_OpenFont(fontPath, fontSize);
    }

    if (font == nullptr) {
        const char* fallbackFonts[] = {
            "/System/Library/Fonts/Menlo.ttc",
            "/System/Library/Fonts/Courier.dfont",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
            "C:\\Windows\\Fonts\\consola.ttf",
            nullptr
        };

        for (int i = 0; fallbackFonts[i] != nullptr && font == nullptr; i++) {
            font = TTF_OpenFont(fallbackFonts[i], fontSize);
        }
    }

    if (font == nullptr) {
        SDL_Log("Failed to load any font: %s", TTF_GetError());
        return false;
    }

    return true;
}

void GUI::drawText(SDL_Renderer* renderer, const std::string& text, int x, int y, SDL_Color color) {
    if (font == nullptr || text.empty()) return;

    SDL_Surface* surface = TTF_RenderText_Blended(font, text.c_str(), color);
    if (surface == nullptr) {
        return;
    }

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

std::vector<std::string> GUI::formatCarStats(const Car& car) {
    std::vector<std::string> stats;
    std::ostringstream oss;

    oss << std::fixed << std::setprecision(1);
    oss << "Position: (" << car.pos_x << ", " << car.pos_y << ")";
    stats.push_back(oss.str());
    oss.str("");

    double speed = car.velocity.norm();
    oss << "Speed: " << speed << " m/s";
    stats.push_back(oss.str());
    oss.str("");

    oss << "Velocity: (" << car.velocity.x() << ", " << car.velocity.y() << ")";
    stats.push_back(oss.str());
    oss.str("");

    double accel_mag = car.acceleration.norm();
    oss << "Acceleration: " << accel_mag << " m/s²";
    stats.push_back(oss.str());
    oss.str("");

    double angleDeg = car.angular_position * Constants::RAD_TO_DEG;
    while (angleDeg < 0) angleDeg += 360;
    while (angleDeg >= 360) angleDeg -= 360;
    oss << "Angle: " << angleDeg << "°";
    stats.push_back(oss.str());
    oss.str("");

    oss << std::setprecision(2);
    oss << "Angular Vel: " << car.angular_velocity << " rad/s";
    stats.push_back(oss.str());
    oss.str("");

    double steeringDeg = car.steering_angle * Constants::RAD_TO_DEG;
    oss << "Steering: " << steeringDeg << "°";
    stats.push_back(oss.str());
    oss.str("");

    oss << std::setprecision(1);
    oss << "Forces: (" << car.forces.x() << ", " << car.forces.y() << ")";
    stats.push_back(oss.str());
    oss.str("");

    oss << std::setprecision(1);
    oss << "Torque: " << car.angular_torque << " N·m";
    stats.push_back(oss.str());
    oss.str("");

    stats.push_back("");
    stats.push_back("--- Tire Grip ---");

    oss << std::setprecision(0);
    oss << "FL: " << (car.frontLeft->gripLevel * 100) << "% (" << car.frontLeft->normalForce << "N)";
    stats.push_back(oss.str());
    oss.str("");

    oss << "FR: " << (car.frontRight->gripLevel * 100) << "% (" << car.frontRight->normalForce << "N)";
    stats.push_back(oss.str());
    oss.str("");

    oss << "RL: " << (car.backLeft->gripLevel * 100) << "% (" << car.backLeft->normalForce << "N)";
    stats.push_back(oss.str());
    oss.str("");

    oss << "RR: " << (car.backRight->gripLevel * 100) << "% (" << car.backRight->normalForce << "N)";
    stats.push_back(oss.str());

    return stats;
}

void GUI::drawHUD(SDL_Renderer* renderer, const Car& car) {
    if (!visible) return;

    int windowWidth, windowHeight;
    SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

    std::vector<std::string> stats = formatCarStats(car);

    int padding = std::max(5, windowHeight / 100);
    int lineHeight = fontSize + std::max(2, windowHeight / 250);
    int panelWidth = std::max(240, windowWidth / 5);
    int panelHeight = stats.size() * lineHeight + 2 * padding;

    int marginX = std::max(5, windowWidth / 200);
    int marginY = std::max(5, windowHeight / 200);

    SDL_Rect panel = {marginX, marginY, panelWidth, panelHeight};
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 180);
    SDL_RenderFillRect(renderer, &panel);

    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_RenderDrawRect(renderer, &panel);

    SDL_Color textColor = {255, 255, 255, 255};
    int textX = panel.x + padding;
    int textY = panel.y + padding;

    int tireGripStartLine = -1;
    for (size_t i = 0; i < stats.size(); i++) {
        if (stats[i] == "--- Tire Grip ---") {
            tireGripStartLine = static_cast<int>(i);
            break;
        }
    }

    for (size_t i = 0; i < stats.size(); i++) {
        SDL_Color color = textColor;

        if (tireGripStartLine >= 0 && i > static_cast<size_t>(tireGripStartLine + 1)) {
            int tireIndex = static_cast<int>(i) - tireGripStartLine - 2;
            const Wheel* tire = nullptr;

            switch (tireIndex) {
                case 0: tire = car.frontLeft; break;
                case 1: tire = car.frontRight; break;
                case 2: tire = car.backLeft; break;
                case 3: tire = car.backRight; break;
            }

            if (tire != nullptr) {
                if (tire->gripLevel < 0.70) {
                    color = {0, 255, 0, 255};
                } else if (tire->gripLevel < 0.90) {
                    color = {255, 255, 0, 255};
                } else {
                    color = {255, 0, 0, 255};
                }
            }
        }

        drawText(renderer, stats[i], textX, textY, color);
        textY += lineHeight;
    }

    double maxTcsInterference = std::max(car.backLeft->tcsInterference, car.backRight->tcsInterference);

    int barY = panel.y + panel.h + padding;
    int barHeight = std::max(15, windowHeight / 50);
    int barWidth = panelWidth - 2 * padding;
    int barX = panel.x + padding;

    drawText(renderer, "TCS:", barX, barY, {200, 200, 200, 255});

    int labelWidth = 35;
    int actualBarX = barX + labelWidth;
    int actualBarWidth = barWidth - labelWidth;

    SDL_Rect barBackground = {actualBarX, barY, actualBarWidth, barHeight};
    SDL_SetRenderDrawColor(renderer, 40, 40, 40, 200);
    SDL_RenderFillRect(renderer, &barBackground);

    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawRect(renderer, &barBackground);

    if (maxTcsInterference > 0.0) {
        int fillWidth = static_cast<int>(std::min(1.0, maxTcsInterference / 30.0) * actualBarWidth);
        SDL_Rect barFill = {actualBarX, barY, fillWidth, barHeight};

        SDL_Color barColor;
        if (maxTcsInterference < 10) {
            barColor = {0, 255, 0, 255};
        } else if (maxTcsInterference < 20) {
            barColor = {255, 255, 0, 255};
        } else {
            barColor = {255, 0, 0, 255};
        }

        SDL_SetRenderDrawColor(renderer, barColor.r, barColor.g, barColor.b, barColor.a);
        SDL_RenderFillRect(renderer, &barFill);
    }

    double maxAbsInterference = std::max({car.frontLeft->absInterference, car.frontRight->absInterference,
                                           car.backLeft->absInterference, car.backRight->absInterference});

    int absBarY = barY + barHeight + padding;

    drawText(renderer, "ABS:", barX, absBarY, {200, 200, 200, 255});

    SDL_Rect absBarBackground = {actualBarX, absBarY, actualBarWidth, barHeight};
    SDL_SetRenderDrawColor(renderer, 40, 40, 40, 200);
    SDL_RenderFillRect(renderer, &absBarBackground);

    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawRect(renderer, &absBarBackground);

    if (maxAbsInterference > 0.0) {
        int fillWidth = static_cast<int>(std::min(1.0, maxAbsInterference / 30.0) * actualBarWidth);
        SDL_Rect barFill = {actualBarX, absBarY, fillWidth, barHeight};

        SDL_Color barColor;
        if (maxAbsInterference < 10) {
            barColor = {0, 255, 0, 255};
        } else if (maxAbsInterference < 20) {
            barColor = {255, 255, 0, 255};
        } else {
            barColor = {255, 0, 0, 255};
        }

        SDL_SetRenderDrawColor(renderer, barColor.r, barColor.g, barColor.b, barColor.a);
        SDL_RenderFillRect(renderer, &barFill);
    }

    drawGraphs(renderer);
}

void GUI::toggleHUD() {
    visible = !visible;
}

void GUI::toggleGraphs() {
    showGraphs = !showGraphs;
}

void GUI::updateGraphs(const Car& car, double throttle, double brake, double steering) {
    graphs[0].addPoint(car.velocity.norm());

    double throttleBrake = throttle - brake;
    graphs[1].addPoint(throttleBrake);

    graphs[2].addPoint(steering);

    graphs[3].addPoint(car.frontLeft->gripLevel);
    graphs[4].addPoint(car.frontRight->gripLevel);
    graphs[5].addPoint(car.backLeft->gripLevel);
    graphs[6].addPoint(car.backRight->gripLevel);
}

void GUI::drawGraph(SDL_Renderer* renderer, const GraphData& graph, int x, int y, int width, int height) {
    if (graph.values.empty()) return;

    SDL_Rect panel = {x, y, width, height};
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 180);
    SDL_RenderFillRect(renderer, &panel);

    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawRect(renderer, &panel);

    int textPadding = std::max(3, width / 60);

    drawText(renderer, graph.label, x + textPadding, y + textPadding, {200, 200, 200, 255});

    if (graph.minValue < 0 && graph.maxValue > 0) {
        double zeroY = y + height - ((-graph.minValue) / (graph.maxValue - graph.minValue)) * height;
        SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
        SDL_RenderDrawLine(renderer, x, static_cast<int>(zeroY), x + width, static_cast<int>(zeroY));
    }

    SDL_SetRenderDrawColor(renderer, graph.color.r, graph.color.g, graph.color.b, graph.color.a);

    double xStep = static_cast<double>(width) / graph.maxPoints;
    double range = graph.maxValue - graph.minValue;

    for (size_t i = 1; i < graph.values.size(); i++) {
        double val1 = std::clamp(graph.values[i - 1], graph.minValue, graph.maxValue);
        double val2 = std::clamp(graph.values[i], graph.minValue, graph.maxValue);

        double norm1 = (val1 - graph.minValue) / range;
        double norm2 = (val2 - graph.minValue) / range;

        int x1 = x + static_cast<int>((i - 1) * xStep);
        int y1 = y + height - static_cast<int>(norm1 * height);
        int x2 = x + static_cast<int>(i * xStep);
        int y2 = y + height - static_cast<int>(norm2 * height);

        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    }

    if (!graph.values.empty()) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << graph.values.back();
        int valueTextOffset = std::max(30, width / 6);
        drawText(renderer, oss.str(), x + width - valueTextOffset, y + textPadding, graph.color);
    }
}

void GUI::drawGraphs(SDL_Renderer* renderer) {
    if (!showGraphs) return;

    int windowWidth, windowHeight;
    SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

    int graphWidth = std::max(200, windowWidth / 6);
    int graphHeight = std::max(50, windowHeight / 13);
    int graphPadding = std::max(5, windowHeight / 100);
    int marginX = std::max(5, windowWidth / 200);
    int marginY = std::max(5, windowHeight / 200);
    int startX = windowWidth - graphWidth - marginX;
    int startY = marginY;

    for (size_t i = 0; i < graphs.size(); i++) {
        int graphY = startY + static_cast<int>(i) * (graphHeight + graphPadding);
        drawGraph(renderer, graphs[i], startX, graphY, graphWidth, graphHeight);
    }
}