#include "ui/GUI.h"
#include "config/PhysicsConstants.h"
#include <sstream>
#include <iomanip>
#include <cmath>

GUI::GUI() : font(nullptr), dialFont(nullptr), visible(true), showGraphs(true), showDials(true), fontSize(16),
             currentThrottle(0.0), currentBrake(0.0), currentSteering(0.0), currentClutch(0.0),
             rpmDial(0.0, 8000.0, "RPM", ""),
             torqueDial(0.0, 400.0, "TORQUE", "Nm"),
             airFlowDial(0.0, 0.05, "AIR FLOW", "kg/s"),
             manifoldPressureDial(0.0, 150.0, "MANIFOLD", "kPa"),
             speedDial(0.0, 300.0, "SPEED", "km/h"),
             volEffDial(0.0, 1.0, "VOL EFF", "%"),
             afrDial(0.0, 20.0, "AFR", ""),
             powerDial(0.0, 200.0, "POWER", "kW") {
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
#ifdef __EMSCRIPTEN__
        const char* fallbackFonts[] = {
            "/assets/DejaVuSansMono.ttf",
            nullptr
        };
#else
        const char* fallbackFonts[] = {
            "/System/Library/Fonts/Menlo.ttc",
            "/System/Library/Fonts/Courier.dfont",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
            "C:\\Windows\\Fonts\\consola.ttf",
            nullptr
        };
#endif

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

    stats.push_back("--- Vehicle ---");

    oss << std::fixed << std::setprecision(1);
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

    int padding = std::max(5, windowHeight / 100);
    int lineHeight = fontSize + std::max(2, windowHeight / 250);
    int marginX = std::max(5, windowWidth / 200);
    int marginY = std::max(5, windowHeight / 200);

    int currentGear = car.getCurrentGear();

    int speedPanelWidth = std::max(180, windowWidth / 7);
    int speedPanelHeight = lineHeight * 6;
    int speedPanelX = windowWidth - speedPanelWidth - marginX;

    int baseRadius = std::max(50, std::min(windowWidth, windowHeight) / 16);
    int primaryDialRadius = static_cast<int>(baseRadius * 1.55);
    int dialBottomY = marginY + primaryDialRadius * 2 + marginY;

    int speedPanelY = dialBottomY + padding * 2;

    SDL_Rect speedPanel = {speedPanelX, speedPanelY, speedPanelWidth, speedPanelHeight};
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 28, 28, 28, 200);
    SDL_RenderFillRect(renderer, &speedPanel);
    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_RenderDrawRect(renderer, &speedPanel);

    std::ostringstream speedOss;
    speedOss << std::fixed << std::setprecision(1) << (car.velocity.norm() * 3.6);
    std::string speedText = speedOss.str();

    int speedTextX = speedPanelX + padding;
    int speedTextY = speedPanelY + padding;

    drawText(renderer, "SPEED", speedTextX, speedTextY, {208, 208, 208, 255});
    drawText(renderer, speedText + " km/h", speedTextX, speedTextY + lineHeight, {79, 163, 99, 255});

    speedTextY += lineHeight * 3;
    drawText(renderer, "GEAR", speedTextX, speedTextY, {208, 208, 208, 255});

    if (currentGear == -2) {
        drawText(renderer, "R", speedTextX, speedTextY + lineHeight, {194, 92, 92, 255});
    } else if (currentGear == -1) {
        drawText(renderer, "N", speedTextX, speedTextY + lineHeight, {208, 208, 208, 255});
    } else {
        std::string gearNum = std::to_string(currentGear + 1);
        drawText(renderer, gearNum, speedTextX, speedTextY + lineHeight, {79, 163, 99, 255});
    }

    drawGraphs(renderer);
    drawDials(renderer, car);
    drawInputSliders(renderer);
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
    currentThrottle = throttle;
    currentBrake = brake;
    currentSteering = steering;

    const Gearbox& gearbox = car.getGearbox();
    currentClutch = gearbox.getClutchEngagement();

    graphs[0].addDataPoint(car.velocity.norm());

    double throttleBrake = throttle - brake;
    graphs[1].addDataPoint(throttleBrake);

    graphs[2].addDataPoint(steering);

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
    int graphPadding = std::max(10, windowHeight / 80);
    int marginX = std::max(15, windowWidth / 150);
    int marginY = std::max(10, windowHeight / 100);

    int baseRadius = std::max(50, std::min(windowWidth, windowHeight) / 16);
    int dialRadius = static_cast<int>(baseRadius * 1.55);
    int dialBottomY = marginY + dialRadius * 2 + marginY;

    int lineHeight = fontSize + std::max(2, windowHeight / 250);
    int speedPanelHeight = lineHeight * 6;
    int speedPanelBottomY = dialBottomY + graphPadding * 2 + speedPanelHeight;

    int bottomStartY = windowHeight - (graphHeight + graphPadding) * 4 - marginY;
    int bottomStartX = marginX;

    int clutchSlipY = bottomStartY - graphHeight - graphPadding * 3;

    SDL_Rect clutchSlipPanel = {
        bottomStartX - graphPadding,
        clutchSlipY - graphPadding,
        graphWidth + graphPadding * 2,
        graphHeight + graphPadding * 2
    };
    SDL_SetRenderDrawColor(renderer, 28, 28, 28, 200);
    SDL_RenderFillRect(renderer, &clutchSlipPanel);

    graphs[3].render(renderer, bottomStartX, clutchSlipY, graphWidth, graphHeight, font);

    SDL_Rect gripGraphsPanel = {
        bottomStartX - graphPadding,
        bottomStartY - graphPadding,
        graphWidth + graphPadding * 2,
        graphHeight * 4 + graphPadding * 5
    };
    SDL_SetRenderDrawColor(renderer, 28, 28, 28, 200);
    SDL_RenderFillRect(renderer, &gripGraphsPanel);

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
    int primaryDialRadius = static_cast<int>(baseRadius * 1.55);
    int secondaryDialRadius = static_cast<int>(baseRadius * 1.0);
    int spacing = primaryDialRadius / 2;
    int marginY = std::max(10, windowHeight / 100);
    int marginX = std::max(15, windowWidth / 150);

    const Engine& engine = car.getEngine();

    int primaryStartX = windowWidth - marginX - primaryDialRadius - (primaryDialRadius * 2 + spacing);
    int primaryStartY = marginY + primaryDialRadius;

    SDL_Rect primaryDialsPanel = {
        primaryStartX - primaryDialRadius - marginX / 2,
        marginY - marginY / 2,
        primaryDialRadius * 4 + spacing + marginX,
        primaryDialRadius * 2 + marginY
    };
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 28, 28, 28, 200);
    SDL_RenderFillRect(renderer, &primaryDialsPanel);
    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_RenderDrawRect(renderer, &primaryDialsPanel);

    speedDial.setValue(car.velocity.norm() * 3.6);
    speedDial.draw(renderer, primaryStartX, primaryStartY, primaryDialRadius, dialFont);

    rpmDial.setValue(engine.getRPM());
    rpmDial.draw(renderer, primaryStartX + primaryDialRadius * 2 + spacing, primaryStartY, primaryDialRadius, dialFont);

    int secondaryStartX = windowWidth / 2 - (secondaryDialRadius * 6 + spacing * 2.5) - windowWidth / 10;
    int secondaryStartY = primaryStartY;

    int graphPadding = std::max(10, windowHeight / 80);
    SDL_Rect secondaryDialsPanel = {
        secondaryStartX - secondaryDialRadius - graphPadding,
        secondaryStartY - secondaryDialRadius - graphPadding,
        (secondaryDialRadius * 2 + spacing) * 5 + secondaryDialRadius * 2 + graphPadding * 2,
        secondaryDialRadius * 2 + graphPadding * 2
    };
    SDL_SetRenderDrawColor(renderer, 28, 28, 28, 200);
    SDL_RenderFillRect(renderer, &secondaryDialsPanel);
    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_RenderDrawRect(renderer, &secondaryDialsPanel);

    torqueDial.setValue(engine.getEngineTorque());
    torqueDial.draw(renderer, secondaryStartX, secondaryStartY, secondaryDialRadius, dialFont);

    airFlowDial.setValue(engine.getAirFlowRateValue());
    airFlowDial.draw(renderer, secondaryStartX + secondaryDialRadius * 2 + spacing, secondaryStartY, secondaryDialRadius, dialFont);

    double manifoldPressure = 101.325;
    manifoldPressureDial.setValue(manifoldPressure);
    manifoldPressureDial.draw(renderer, secondaryStartX + (secondaryDialRadius * 2 + spacing) * 2, secondaryStartY, secondaryDialRadius, dialFont);

    volEffDial.setValue(engine.getVolumetricEfficiencyValue() * 100);
    volEffDial.draw(renderer, secondaryStartX + (secondaryDialRadius * 2 + spacing) * 3, secondaryStartY, secondaryDialRadius, dialFont);

    afrDial.setValue(engine.getAirFuelRatioValue());
    afrDial.draw(renderer, secondaryStartX + (secondaryDialRadius * 2 + spacing) * 4, secondaryStartY, secondaryDialRadius, dialFont);

    powerDial.setValue(engine.getCurrentPower() / 1000.0);
    powerDial.draw(renderer, secondaryStartX + (secondaryDialRadius * 2 + spacing) * 5, secondaryStartY, secondaryDialRadius, dialFont);
}

void GUI::drawInputSliders(SDL_Renderer* renderer) {
    int windowWidth, windowHeight;
    SDL_GetRendererOutputSize(renderer, &windowWidth, &windowHeight);

    int sliderWidth = std::max(150, windowWidth / 8);
    int sliderHeight = std::max(20, windowHeight / 40);
    int sliderPadding = std::max(8, windowHeight / 100);
    int marginX = std::max(15, windowWidth / 150);
    int marginY = std::max(10, windowHeight / 100);

    int baseRadius = std::max(50, std::min(windowWidth, windowHeight) / 16);
    int dialRadius = static_cast<int>(baseRadius * 1.55);
    int dialBottomY = marginY + dialRadius * 2 + marginY;
    int lineHeight = fontSize + std::max(2, windowHeight / 250);
    int speedPanelHeight = lineHeight * 6;
    int speedPanelBottomY = dialBottomY + sliderPadding * 2 + speedPanelHeight;

    int startX = windowWidth - sliderWidth - marginX * 2;
    int startY = speedPanelBottomY + sliderPadding * 3;

    int labelWidth = 80;
    int textSliderGap = 15;

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    auto drawSlider = [&](const char* label, double value, int y, SDL_Color fillColor) {
        drawText(renderer, label, startX - labelWidth - textSliderGap, y + sliderHeight / 4, {208, 208, 208, 255});

        SDL_Rect background = {startX, y, sliderWidth, sliderHeight};
        SDL_SetRenderDrawColor(renderer, 40, 40, 40, 200);
        SDL_RenderFillRect(renderer, &background);

        SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
        SDL_RenderDrawRect(renderer, &background);

        if (value > 0.0) {
            int fillWidth = static_cast<int>(value * sliderWidth);
            SDL_Rect fill = {startX, y, fillWidth, sliderHeight};
            SDL_SetRenderDrawColor(renderer, fillColor.r, fillColor.g, fillColor.b, fillColor.a);
            SDL_RenderFillRect(renderer, &fill);
        }
    };

    auto drawCenteredSlider = [&](const char* label, double value, int y, SDL_Color fillColor) {
        drawText(renderer, label, startX - labelWidth - textSliderGap, y + sliderHeight / 4, {208, 208, 208, 255});

        SDL_Rect background = {startX, y, sliderWidth, sliderHeight};
        SDL_SetRenderDrawColor(renderer, 40, 40, 40, 200);
        SDL_RenderFillRect(renderer, &background);

        SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
        SDL_RenderDrawRect(renderer, &background);

        int centerX = startX + sliderWidth / 2;
        if (value < 0.0) {
            int fillWidth = static_cast<int>(-value * sliderWidth / 2);
            SDL_Rect fill = {centerX - fillWidth, y, fillWidth, sliderHeight};
            SDL_SetRenderDrawColor(renderer, fillColor.r, fillColor.g, fillColor.b, fillColor.a);
            SDL_RenderFillRect(renderer, &fill);
        } else if (value > 0.0) {
            int fillWidth = static_cast<int>(value * sliderWidth / 2);
            SDL_Rect fill = {centerX, y, fillWidth, sliderHeight};
            SDL_SetRenderDrawColor(renderer, fillColor.r, fillColor.g, fillColor.b, fillColor.a);
            SDL_RenderFillRect(renderer, &fill);
        }

        SDL_SetRenderDrawColor(renderer, 150, 150, 150, 255);
        SDL_RenderDrawLine(renderer, centerX, y, centerX, y + sliderHeight);
    };

    drawSlider("Throttle", currentThrottle, startY, {79, 163, 99, 255});
    drawCenteredSlider("Steering", -currentSteering, startY + sliderHeight + sliderPadding, {75, 151, 179, 255});
    drawSlider("Brake", currentBrake, startY + (sliderHeight + sliderPadding) * 2, {194, 92, 92, 255});
    drawSlider("Clutch", currentClutch, startY + (sliderHeight + sliderPadding) * 3, {58, 58, 58, 255});
}
