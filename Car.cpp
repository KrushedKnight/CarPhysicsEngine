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
    for (Wheel* wheel : wheels) {
        // Calculate friction force from wheel-ground interaction
        Eigen::Vector2d wheelHeading = wheel->calculateFriction(velocity, angular_position, Constants::TIME_INTERVAL);

        // Decompose wheel force into longitudinal (along car heading) and lateral components
        Eigen::Vector2d heading{sin(angular_position), cos(angular_position)};
        Eigen::Vector2d projection = (wheelHeading.dot(heading) / heading.squaredNorm() * heading);
        Eigen::Vector2d lateralForce = wheelHeading - projection;

        // Calculate torque from lateral forces (2D cross product)
        double signedTorque = heading.x() * lateralForce.y() - heading.y() * lateralForce.x();

        addTorque(signedTorque * Constants::DIST_TO_WHEEL);
        addForce(wheelHeading);
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
    SDL_RenderPresent(renderer);
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