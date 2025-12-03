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
    for (Wheel* wheel : wheels) {
        if (wheel->angular_velocity * wheel->wheelRadius < Constants::CAR_TOP_SPEED) {
            wheel->addTorque(frontLeft->wheelRadius * engine_power);
        }
    }
}

void Car::applyBrakes() {
    for (Wheel* wheel : wheels) {
        if (wheel->angular_velocity > 0) {
            wheel->addTorque(frontLeft->wheelRadius * braking_power);
        }
    }
}

void Car::sumWheelForces() {
    for (Wheel* wheel : wheels) {
        //should this be moved?

        double angleToWheel = getAngleToWheel(wheel);
        // double frictionalForce = wheel->calculateFriction(velocity.norm(), engine_power).norm();
        // Eigen::Vector2d wheelHeading{frictionalForce * sin(angular_position + angleToWheel), frictionalForce * cos(angular_position + angleToWheel)};


        Eigen::Vector2d wheelHeading = wheel->calculateFriction(velocity, angular_position);
        Eigen::Vector2d heading{sin(angular_position), cos(angular_position)};
        Eigen::Vector2d projection = (wheelHeading.dot(heading) / heading.squaredNorm() * heading);
        Eigen::Vector2d appliedTorque = wheelHeading - projection;

        double direction;
        //TODO: need to account for direction here




        addTorque((wheelHeading - projection).norm() * Constants::DIST_TO_WHEEL);
        addForce(projection);
    }
}

void Car::moveWheels() {
    for (Wheel* wheel : wheels) {
        // wheel->setLinearVelocity(velocity.norm());
        wheel->incrementTime(Constants::TIME_INTERVAL);
    }
}

void Car::drawCar(SDL_Renderer* renderer) {

    SDL_Texture* tex = getRectangleTexture(renderer);
    SDL_Rect rect{getPositionX(), getPositionY(), getWidth(), getHeight()};
    double angle = angular_position;

    SDL_RenderCopyEx(renderer, tex, NULL, &rect, angle, NULL, SDL_FLIP_NONE);
    SDL_RenderPresent(renderer);
}

// This method was GPT-generated
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