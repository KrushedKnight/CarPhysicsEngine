//
// Created by beast-machine-2 on 7/2/25.
//

#include "Car.h"
#include "constants.h"



Car::Car(double x, double y, int w, int h) : width(w), height(h), steering_angle(0){
    pos_x = x;
    pos_y = y;

    frontLeft = new Wheel();
    frontRight = new Wheel();
    backLeft = new Wheel();
    backRight = new Wheel();

    wheels.assign({frontLeft, frontLeft, backLeft, backRight});
}


const int Car::getWidth() {
    return width;
}

const int Car::getHeight() {
    return height;
}

void Car::applySteering(double amount) {

    steering_angle += amount;
    frontLeft->wheelAngle = steering_angle * Constants::STEERING_RACK;
    frontRight->wheelAngle = steering_angle * Constants::STEERING_RACK;
}

void Car::applyEngineTorque() {
    backRight->addTorque(backRight->wheelRadius * engine_power);
    backLeft->addTorque(backLeft->wheelRadius * engine_power);
}

void Car::applyBrakes() {
    backRight->addTorque(backRight->wheelRadius * braking_power);
    backLeft->addTorque(backLeft->wheelRadius * braking_power);
}

void Car::sumWheelForces() {
    for (Wheel* wheel : wheels) {
        //Replace with weight distribution
        double angleToWheel = getAngleToWheel(wheel);
        double frictionalForce = wheel->calculateFriction().norm();

        Eigen::Vector2d wheelHeading{frictionalForce * cos(angular_position + angleToWheel), frictionalForce * sin(angular_position + angleToWheel)};
        Eigen::Vector2d heading{cos(angular_position), sin(angular_position)};
        Eigen::Vector2d projection = (wheelHeading.dot(heading) / heading.squaredNorm() * heading);


        addTorque((wheelHeading - projection).norm() * Constants::DIST_TO_WHEEL);
        addForce(projection);
    }
}

void Car::drawCar(SDL_Renderer* renderer) {

    SDL_Texture* tex = getRectangleTexture(renderer);
    SDL_Rect rect{getPositionX(), getPositionY(), getWidth(), getHeight()};
    double angle = angular_position;

    SDL_RenderCopyEx(renderer, tex, NULL, &rect, angle, NULL, SDL_FLIP_NONE);
    SDL_RenderPresent(renderer);
}

SDL_Texture* Car::getRectangleTexture(SDL_Renderer* renderer) {
    SDL_Texture* tex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                         SDL_TEXTUREACCESS_TARGET, width, height);
    SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND);
    SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND);
    SDL_SetRenderTarget(renderer, tex);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

    SDL_RenderClear(renderer);
    SDL_SetRenderTarget(renderer, NULL);
    return tex;
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