//
// Created by beast-machine-2 on 7/2/25.
//

#include "Car.h"
#include "constants.h"
#include <cmath>



Car::Car(double x, double y, int w, int h) : width(w), height(h), steering_angle(0) {
    pos_x = x;
    pos_y = y;

    frontLeft = new Wheel();
    frontRight = new Wheel();
    backLeft = new Wheel();
    backRight = new Wheel();

    wheels.assign({frontLeft, frontRight, backLeft, backRight});
}

Car::~Car() {
    delete frontLeft;
    delete frontRight;
    delete backLeft;
    delete backRight;

    if (carTexture != nullptr) {
        SDL_DestroyTexture(carTexture);
    }
}


const int Car::getWidth() {
    return width;
}

const int Car::getHeight() {
    return height;
}

void Car::applySteering(double amount) {

    steering_angle += amount;
    steering_angle = std::clamp(steering_angle, -Constants::MAX_STEERING_ANGLE, Constants::MAX_STEERING_ANGLE);


    frontLeft->wheelAngle = steering_angle * Constants::STEERING_RACK;
    frontRight->wheelAngle = steering_angle * Constants::STEERING_RACK;
}

void Car::applyForceFeedback()
{
    steering_angle *= Constants::FORCE_FEEDBACK_DECAY;

    frontLeft->wheelAngle = steering_angle * Constants::STEERING_RACK;
    frontRight->wheelAngle = steering_angle * Constants::STEERING_RACK;
}

void Car::applyEngineTorque() {
    for (Wheel* wheel : wheels) {
        // Only apply torque if wheel hasn't reached top speed
        if (wheel->angular_velocity * wheel->wheelRadius < Constants::CAR_TOP_SPEED) {
            wheel->addTorque(wheel->wheelRadius * engine_power);  // Fixed: use wheel, not frontLeft
        }
    }
}

void Car::applyBrakes() {
    for (Wheel* wheel : wheels) {
        if (std::abs(wheel->angular_velocity) > 1e-5) {
            double brakingTorque = -std::abs(braking_power * wheel->wheelRadius) * std::copysign(1.0, wheel->angular_velocity);
            wheel->addTorque(brakingTorque);
        }
    }
}

void Car::sumWheelForces() {
    // Calculate wheel positions in world space (meters)
    double halfWidth = (Constants::CAR_WIDTH / 10.0) / 2.0;   // Convert pixels to meters
    double halfLength = (Constants::CAR_LENGTH / 10.0) / 2.0;

    // Wheel positions relative to car center in car's local frame
    Eigen::Vector2d wheelPositions[4] = {
        {-halfWidth, halfLength},   // Front-left
        {halfWidth, halfLength},    // Front-right
        {-halfWidth, -halfLength},  // Back-left
        {halfWidth, -halfLength}    // Back-right
    };

    // Pre-calculate rotation transformation for efficiency
    double cos_angle = cos(angular_position);
    double sin_angle = sin(angular_position);

    for (int i = 0; i < 4; i++) {
        Wheel* wheel = wheels[i];

        // Transform wheel position from local to world coordinates
        Eigen::Vector2d wheelPosWorld(
            cos_angle * wheelPositions[i].x() - sin_angle * wheelPositions[i].y(),
            sin_angle * wheelPositions[i].x() + cos_angle * wheelPositions[i].y()
        );

        // Calculate rotational velocity in world coordinates: v_rot = ω × r_world
        // In 2D: v_rot = [-ω*r_y, ω*r_x]
        Eigen::Vector2d rotationalVel(-angular_velocity * wheelPosWorld.y(),
                                       angular_velocity * wheelPosWorld.x());

        // Total wheel velocity in world coordinates
        Eigen::Vector2d wheelVelocity = velocity + rotationalVel;

        // Calculate friction force from wheel-ground interaction (returns force in world coordinates)
        Eigen::Vector2d wheelForce = wheel->calculateFriction(wheelVelocity, angular_position, Constants::TIME_INTERVAL);

        // Calculate torque: τ = r × F (2D cross product: r_x * F_y - r_y * F_x)
        // Both vectors are in world coordinates
        double torque = wheelPosWorld.x() * wheelForce.y() - wheelPosWorld.y() * wheelForce.x();

        addTorque(torque);
        addForce(wheelForce);
    }
}

void Car::moveWheels() {
    for (Wheel* wheel : wheels) {
        // wheel->setLinearVelocity(velocity.norm());
        wheel->incrementTime(Constants::TIME_INTERVAL);
    }
    applyForceFeedback();
}

void Car::drawCar(SDL_Renderer* renderer) {
    SDL_Texture* tex = getRectangleTexture(renderer);
    SDL_Rect rect{getPositionX(), getPositionY(), getWidth(), getHeight()};

    // Convert radians to degrees for SDL rendering
    double angleDegrees = angular_position * Constants::RAD_TO_DEG;

    SDL_RenderCopyEx(renderer, tex, NULL, &rect, angleDegrees, NULL, SDL_FLIP_NONE);

    // Draw debug vectors
    drawDebugVectors(renderer);

    SDL_RenderPresent(renderer);
}

void Car::drawDebugVectors(SDL_Renderer* renderer) {
    if (!showDebugVectors) return;

    // Car center position in pixels
    int centerX = getPositionX() + getWidth() / 2;
    int centerY = getPositionY() + getHeight() / 2;

    // Scale factors to make vectors visible (pixels per m/s)
    const double velocityScale = 5.0;
    const double accelScale = 20.0;

    // Draw velocity vector (GREEN)
    if (velocity.norm() > 0.01) {
        int velEndX = centerX + static_cast<int>(velocity.x() * velocityScale);
        int velEndY = centerY - static_cast<int>(velocity.y() * velocityScale); // Y inverted

        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);  // Green
        SDL_RenderDrawLine(renderer, centerX, centerY, velEndX, velEndY);

        // Draw arrowhead for velocity
        double angle = std::atan2(-velocity.y(), velocity.x());
        int arrowSize = 8;
        int arrow1X = velEndX - arrowSize * std::cos(angle - 0.5);
        int arrow1Y = velEndY - arrowSize * std::sin(angle - 0.5);
        int arrow2X = velEndX - arrowSize * std::cos(angle + 0.5);
        int arrow2Y = velEndY - arrowSize * std::sin(angle + 0.5);

        SDL_RenderDrawLine(renderer, velEndX, velEndY, arrow1X, arrow1Y);
        SDL_RenderDrawLine(renderer, velEndX, velEndY, arrow2X, arrow2Y);
    }

    // Draw acceleration vector (RED)
    if (acceleration.norm() > 0.01) {
        int accelEndX = centerX + static_cast<int>(acceleration.x() * accelScale);
        int accelEndY = centerY - static_cast<int>(acceleration.y() * accelScale); // Y inverted

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);  // Red
        SDL_RenderDrawLine(renderer, centerX, centerY, accelEndX, accelEndY);

        // Draw arrowhead for acceleration
        double angle = std::atan2(-acceleration.y(), acceleration.x());
        int arrowSize = 8;
        int arrow1X = accelEndX - arrowSize * std::cos(angle - 0.5);
        int arrow1Y = accelEndY - arrowSize * std::sin(angle - 0.5);
        int arrow2X = accelEndX - arrowSize * std::cos(angle + 0.5);
        int arrow2Y = accelEndY - arrowSize * std::sin(angle + 0.5);

        SDL_RenderDrawLine(renderer, accelEndX, accelEndY, arrow1X, arrow1Y);
        SDL_RenderDrawLine(renderer, accelEndX, accelEndY, arrow2X, arrow2Y);
    }
}

// Cached texture creation - only creates once
SDL_Texture* Car::getRectangleTexture(SDL_Renderer* renderer) {
    if (carTexture == nullptr) {
        carTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                       SDL_TEXTUREACCESS_TARGET, width, height);
        SDL_SetTextureBlendMode(carTexture, SDL_BLENDMODE_BLEND);
        SDL_SetRenderTarget(renderer, carTexture);
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);  // Red car
        SDL_RenderClear(renderer);
        SDL_SetRenderTarget(renderer, NULL);
    }
    return carTexture;
}

void Car::eraseCar(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
}

double Car::getAngleToWheel(Wheel* wheel) {
    // return(angular_position - wheel->wheelAngle);
    //TODO: clean up later
    return steering_angle * Constants::STEERING_RACK;
}