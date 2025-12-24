#include "ui/GUI.h"
#include "config/PhysicsConstants.h"
#include <sstream>
#include <iomanip>
#include <cmath>

GUI::GUI() : font(nullptr), dialFont(nullptr), visible(true), showGraphs(true), showDials(true), fontSize(16),
             rpmDial(0.0, 8000.0, "RPM", ""),
             torqueDial(0.0, 400.0, "TORQUE", "Nm"),
             airFlowDial(0.0, 0.05, "AIR FLOW", "kg/s"),
             manifoldPressureDial(0.0, 150.0, "MANIFOLD", "kPa") {
    graphs.emplace_back("Speed (m/s)", SDL_Color{0, 255, 0, 255}, 0.0, 50.0);
    graphs.emplace_back("Throttle/Brake", SDL_Color{255, 165, 0, 255}, -1.0, 1.0);
    graphs.emplace_back("Steering", SDL_Color{100, 200, 255, 255}, -1.0, 1.0);
    graphs.emplace_back("Clutch Slip (rad/s)", SDL_Color{255, 255, 0, 255}, -500.0, 500.0);
    graphs.emplace_back("Clutch Engagement", SDL_Color{255, 200, 100, 255}, 0.0, 1.0);
    graphs.emplace_back("FL Grip", SDL_Color{255, 100, 100, 255}, 0.0, 1.0);
    graphs.emplace_back("FR Grip", SDL_Color{255, 150, 100, 255}, 0.0, 1.0);
    graphs.emplace_back("RL Grip", SDL_Color{200, 100, 255, 255}, 0.0, 1.0);
    graphs.emplace_back("RR Grip", SDL_Color{150, 100, 255, 255}, 0.0, 1.0);
}

GUI::~GUI() {
    if (font != nullptr) {
        TTF_CloseFont(font);
    }
    if (dialFont != nullptr) {
        TTF_CloseFont(dialFont);
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

    int dialFontSize = std::max(8, fontSize / 2);
    if (fontPath != nullptr) {
        dialFont = TTF_OpenFont(fontPath, dialFontSize);
    }

    if (dialFont == nullptr) {
        const char* fallbackFonts[] = {
            "/System/Library/Fonts/Menlo.ttc",
            "/System/Library/Fonts/Courier.dfont",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
            "C:\\Windows\\Fonts\\consola.ttf",
            nullptr
        };

        for (int i = 0; fallbackFonts[i] != nullptr && dialFont == nullptr; i++) {
            dialFont = TTF_OpenFont(fallbackFonts[i], dialFontSize);
        }
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

std::vector<std::string> GUI::formatCarStats(const Car& car, double throttle) {
    std::vector<std::string> stats;
    std::ostringstream oss;

    const Engine& engine = car.getEngine();
    const Gearbox& gearbox = car.getGearbox();

    stats.push_back("--- Engine ---");
    oss << std::fixed << std::setprecision(1);
    oss << "Power: " << (engine.getCurrentPower() / 1000.0) << " kW";
    stats.push_back(oss.str());
    oss.str("");

    oss << std::setprecision(2);
    oss << "Clutch Eng: " << (gearbox.getClutchEngagement() * 100) << "%";
    stats.push_back(oss.str());
    oss.str("");

    oss << std::setprecision(1);
    oss << "Clutch Torque: " << gearbox.getClutchTorque() << " N·m";
    stats.push_back(oss.str());
    oss.str("");

    oss << std::setprecision(2);
    oss << "AFR: " << engine.getAirFuelRatioValue();
    stats.push_back(oss.str());
    oss.str("");

    oss << "Vol Eff: " << (engine.getVolumetricEfficiencyValue() * 100) << "%";
    stats.push_back(oss.str());
    oss.str("");

    stats.push_back("");
    stats.push_back("--- Vehicle ---");

    oss << std::setprecision(1);
    oss << "Position: (" << car.pos_x << ", " << car.pos_y << ")";
    stats.push_back(oss.str());
    oss.str("");

    oss << "Velocity: (" << car.velocity.x() << ", " << car.velocity.y() << ")";
    stats.push_back(oss.str());
    oss.str("");

    double accel_mag = car.acceleration.norm();
    oss << "Acceleration: " << accel_mag << " m/s²";
    stats.push_back(oss.str());
    oss.str("");

    double angleDeg = car.angular_position * PhysicsConstants::RAD_TO_DEG;
    while (angleDeg < 0) angleDeg += 360;
    while (angleDeg >= 360) angleDeg -= 360;
    oss << "Angle: " << angleDeg << "°";
    stats.push_back(oss.str());
    oss.str("");

    oss << std::setprecision(2);
    oss << "Angular Vel: " << car.angular_velocity << " rad/s";
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
    stats.push_back("--- Tire Load ---");

    oss << std::setprecision(0);
    oss << "FL: " << car.frontLeft->normalForce << "N";
    stats.push_back(oss.str());
    oss.str("");

    oss << "FR: " << car.frontRight->normalForce << "N";
    stats.push_back(oss.str());
    oss.str("");

    oss << "RL: " << car.backLeft->normalForce << "N";
    stats.push_back(oss.str());
    oss.str("");

    oss << "RR: " << car.backRight->normalForce << "N";
    stats.push_back(oss.str());

    return stats;
}

void GUI::drawHUD(SDL_Renderer* renderer, const Car& car, double throttle) {
    if (!visible) return;

    int windowWidth, windowHeight;
    SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

    std::vector<std::string> stats = formatCarStats(car, throttle);

    int padding = std::max(5, windowHeight / 100);
    int lineHeight = fontSize + std::max(2, windowHeight / 250);
    int panelWidth = std::max(240, windowWidth / 5);
    int panelHeight = stats.size() * lineHeight + 2 * padding;

    int marginX = std::max(5, windowWidth / 200);
    int marginY = std::max(5, windowHeight / 200);

    SDL_Rect panel = {marginX, marginY, panelWidth, panelHeight};
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 13, 13, 13, 220);
    SDL_RenderFillRect(renderer, &panel);

    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_RenderDrawRect(renderer, &panel);

    SDL_Color textColor = {255, 255, 255, 255};
    int textX = panel.x + padding;
    int textY = panel.y + padding;

    for (size_t i = 0; i < stats.size(); i++) {
        drawText(renderer, stats[i], textX, textY, textColor);
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
        int fillWidth = static_cast<int>(std::min(1.0, maxTcsInterference / 120.0) * actualBarWidth);
        SDL_Rect barFill = {actualBarX, barY, fillWidth, barHeight};

        SDL_Color barColor;
        if (maxTcsInterference < 40) {
            barColor = {0, 255, 0, 255};
        } else if (maxTcsInterference < 80) {
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

    int currentGear = car.getCurrentGear();

    int speedPanelWidth = std::max(180, windowWidth / 7);
    int speedPanelHeight = lineHeight * 6;
    int speedPanelX = windowWidth - speedPanelWidth - marginX;
    int speedPanelY = windowHeight / 2 - speedPanelHeight / 2;

    SDL_Rect speedPanel = {speedPanelX, speedPanelY, speedPanelWidth, speedPanelHeight};
    SDL_SetRenderDrawColor(renderer, 13, 13, 13, 220);
    SDL_RenderFillRect(renderer, &speedPanel);
    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_RenderDrawRect(renderer, &speedPanel);

    std::ostringstream speedOss;
    speedOss << std::fixed << std::setprecision(1) << (car.velocity.norm() * 3.6);
    std::string speedText = speedOss.str();

    int speedTextX = speedPanelX + padding;
    int speedTextY = speedPanelY + padding;

    drawText(renderer, "SPEED", speedTextX, speedTextY, {150, 150, 150, 255});
    drawText(renderer, speedText + " km/h", speedTextX, speedTextY + lineHeight, {0, 255, 0, 255});

    speedTextY += lineHeight * 3;
    drawText(renderer, "GEAR", speedTextX, speedTextY, {150, 150, 150, 255});

    if (currentGear == -2) {
        drawText(renderer, "R", speedTextX, speedTextY + lineHeight, {255, 100, 100, 255});
    } else if (currentGear == -1) {
        drawText(renderer, "N", speedTextX, speedTextY + lineHeight, {200, 200, 200, 255});
    } else {
        std::string gearNum = std::to_string(currentGear + 1);
        drawText(renderer, gearNum, speedTextX, speedTextY + lineHeight, {0, 255, 0, 255});
    }

    drawGraphs(renderer);
    drawDials(renderer, car);
}

void GUI::toggleHUD() {
    visible = !visible;
}

void GUI::toggleGraphs() {
    showGraphs = !showGraphs;
}

void GUI::toggleDials() {
    showDials = !showDials;
}

void GUI::updateGraphs(const Car& car, double throttle, double brake, double steering) {
    graphs[0].addDataPoint(car.velocity.norm());

    double throttleBrake = throttle - brake;
    graphs[1].addDataPoint(throttleBrake);

    graphs[2].addDataPoint(steering);

    const Gearbox& gearbox = car.getGearbox();
    graphs[3].addDataPoint(gearbox.getClutchSlip());
    graphs[4].addDataPoint(gearbox.getClutchEngagement());

    graphs[5].addDataPoint(car.frontLeft->gripLevel);
    graphs[6].addDataPoint(car.frontRight->gripLevel);
    graphs[7].addDataPoint(car.backLeft->gripLevel);
    graphs[8].addDataPoint(car.backRight->gripLevel);
}

void GUI::drawGraphs(SDL_Renderer* renderer) {
    if (!showGraphs) return;

    int windowWidth, windowHeight;
    SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

    int graphWidth = std::max(200, windowWidth / 6);
    int graphHeight = std::max(50, windowHeight / 13);
    int graphPadding = std::max(5, windowHeight / 100);
    int marginX = std::max(10, windowWidth / 200);
    int marginY = std::max(5, windowHeight / 200);

    int leftStartX = marginX;
    int leftStartY = windowHeight / 2 + windowHeight / 8;

    graphs[3].render(renderer, leftStartX, leftStartY, graphWidth, graphHeight, font);

    int baseRadius = std::max(50, std::min(windowWidth, windowHeight) / 16);
    int dialRadius = static_cast<int>(baseRadius * 1.55);
    int dialBottomY = marginY + dialRadius * 2 + marginY;

    int rightStartX = windowWidth - graphWidth - marginX;
    int rightStartY = marginY * 3;

    graphs[0].render(renderer, rightStartX, rightStartY, graphWidth, graphHeight, font);
    graphs[1].render(renderer, rightStartX, rightStartY + (graphHeight + graphPadding), graphWidth, graphHeight, font);
    graphs[2].render(renderer, rightStartX, rightStartY + 2 * (graphHeight + graphPadding), graphWidth, graphHeight, font);
    graphs[4].render(renderer, rightStartX, rightStartY + 3 * (graphHeight + graphPadding), graphWidth, graphHeight, font);

    int bottomStartY = windowHeight - (graphHeight + graphPadding) * 4 - marginY;
    int bottomStartX = marginX;

    graphs[5].render(renderer, bottomStartX, bottomStartY, graphWidth, graphHeight, font);
    graphs[6].render(renderer, bottomStartX, bottomStartY + (graphHeight + graphPadding), graphWidth, graphHeight, font);
    graphs[7].render(renderer, bottomStartX, bottomStartY + 2 * (graphHeight + graphPadding), graphWidth, graphHeight, font);
    graphs[8].render(renderer, bottomStartX, bottomStartY + 3 * (graphHeight + graphPadding), graphWidth, graphHeight, font);
}

void GUI::drawDials(SDL_Renderer* renderer, const Car& car) {
    if (!showDials) return;

    int windowWidth, windowHeight;
    SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

    int baseRadius = std::max(50, std::min(windowWidth, windowHeight) / 16);
    int dialRadius = static_cast<int>(baseRadius * 1.55);
    int spacing = dialRadius / 3;
    int marginY = std::max(10, windowHeight / 100);

    int totalWidth = dialRadius * 8 + spacing * 3;
    int startX = (windowWidth - totalWidth) / 2 + dialRadius;
    int startY = marginY + dialRadius;

    const Engine& engine = car.getEngine();

    rpmDial.setValue(engine.getRPM());
    rpmDial.draw(renderer, startX, startY, dialRadius, dialFont);

    torqueDial.setValue(engine.getEngineTorque());
    torqueDial.draw(renderer, startX + dialRadius * 2 + spacing, startY, dialRadius, dialFont);

    airFlowDial.setValue(engine.getAirFlowRateValue());
    airFlowDial.draw(renderer, startX + (dialRadius * 2 + spacing) * 2, startY, dialRadius, dialFont);

    double manifoldPressure = 101.325;
    manifoldPressureDial.setValue(manifoldPressure);
    manifoldPressureDial.draw(renderer, startX + (dialRadius * 2 + spacing) * 3, startY, dialRadius, dialFont);
}
