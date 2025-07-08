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
}
void Car::sumWheelForces() {
    // for (Wheel* wheel : wheels) {
    //     //TODO: Fix this torque stuff
    //     addTorque(wheel->calculateFriction().x() * 0.6 * Constants::CAR_LENGTH);
    //     Eigen::Vector2d force(0, wheel->calculateFriction().y());
    //     addForce(force);
    // }
}

void Car::drawCar(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 0);
    SDL_Rect rect{getPositionX(), getPositionY(), getWidth(), getHeight()};


    SDL_RenderFillRect(renderer, &rect);
    SDL_RenderPresent(renderer);
}

void Car::eraseCar(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
}