#include "Car.h"
#include "constants.h"
#include <cmath>
#include <iostream>

Car::Car(double x, double y, int w, int h) : width(w), height(h) {
    pos_x = x;
    pos_y = y;

    double halfWidth = (Constants::CAR_WIDTH / 10.0) / 2.0;
    double halfLength = (Constants::CAR_LENGTH / 10.0) / 2.0;

    frontLeft = new Wheel();
    frontLeft->position = Eigen::Vector2d(-halfWidth, halfLength);

    frontRight = new Wheel();
    frontRight->position = Eigen::Vector2d(halfWidth, halfLength);

    backLeft = new Wheel();
    backLeft->position = Eigen::Vector2d(-halfWidth, -halfLength);

    backRight = new Wheel();
    backRight->position = Eigen::Vector2d(halfWidth, -halfLength);

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

Eigen::Vector2d Car::calculateWheelVelocityLocal(Eigen::Vector2d wheelPosition) {
    double cos_angle = cos(angular_position);
    double sin_angle = sin(angular_position);
    Eigen::Vector2d velocityLocal(
        velocity.x() * cos_angle - velocity.y() * sin_angle,
        velocity.x() * sin_angle + velocity.y() * cos_angle
    );

    Eigen::Vector2d rotationalVelLocal(
        -angular_velocity * wheelPosition.y(),
        angular_velocity * wheelPosition.x()
    );

    return velocityLocal + rotationalVelLocal;
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
    Wheel* rearWheels[] = {backLeft, backRight};

    for (Wheel* wheel : rearWheels) {
        if (wheel->angular_velocity * wheel->wheelRadius < Constants::CAR_TOP_SPEED) {
            Eigen::Vector2d wheelVelocityLocal = calculateWheelVelocityLocal(wheel->position);
            double slipRatio = wheel->calculateSlipRatio(wheelVelocityLocal);
            double error = Constants::TIRE_SLIP_SETPOINT - slipRatio;
            double changeInSlip = slipRatio - wheel->previousSlipError;
            double baseTorque = Constants::CAR_POWER * wheel->wheelRadius;
            double adjustedTorque = baseTorque + Constants::TIRE_TCS_kP * error - Constants::TIRE_TCS_kD * changeInSlip;

            double reduction = baseTorque - adjustedTorque;
            if (reduction > 0) {
                wheel->tcsInterference = (reduction / baseTorque) * 100.0;
            } else {
                wheel->tcsInterference = 0.0;
            }

            wheel->addTorque(adjustedTorque);
            wheel->previousSlipError = slipRatio;
        } else {
            wheel->tcsInterference = 0.0;
        }
    }
}

void Car::applyBrakes() {
    for (Wheel* wheel : wheels) {
        Eigen::Vector2d wheelVelocityLocal = calculateWheelVelocityLocal(wheel->position);
        Eigen::Vector2d wheelForward{sin(wheel->wheelAngle), cos(wheel->wheelAngle)};
        double vehicleSpeed = std::abs(wheelVelocityLocal.dot(wheelForward));

        if (std::abs(wheel->angular_velocity) < 1e-3) {
            wheel->angular_velocity = 0.0;
            wheel->absInterference = 0.0;
        } else if (vehicleSpeed < 0.1) {
            double baseBrakeTorque = -std::abs(braking_power * wheel->wheelRadius) * std::copysign(1.0, wheel->angular_velocity);
            wheel->addTorque(baseBrakeTorque);
            wheel->absInterference = 0.0;
        } else {
            double slipRatio = wheel->calculateSlipRatio(wheelVelocityLocal);
            double error = Constants::ABS_SLIP_SETPOINT - slipRatio;
            double changeInSlip = slipRatio - wheel->previousAbsSlipError;
            double baseBrakeTorque = -std::abs(braking_power * wheel->wheelRadius) * std::copysign(1.0, wheel->angular_velocity);
            double adjustedBrakeTorque = baseBrakeTorque + Constants::ABS_kP * error - Constants::ABS_kD * changeInSlip;

            if (wheel->angular_velocity > 0 && adjustedBrakeTorque > 0) {
                adjustedBrakeTorque = 0.0;
            } else if (wheel->angular_velocity < 0 && adjustedBrakeTorque < 0) {
                adjustedBrakeTorque = 0.0;
            }

            double reduction = std::abs(baseBrakeTorque) - std::abs(adjustedBrakeTorque);
            if (reduction > 0) {
                wheel->absInterference = (reduction / std::abs(baseBrakeTorque)) * 100.0;
            } else {
                wheel->absInterference = 0.0;
            }

            wheel->addTorque(adjustedBrakeTorque);
            wheel->previousAbsSlipError = slipRatio;
        }
    }
}

void Car::sumWheelForces() {
    updateLoadTransfer();

    double cos_angle = cos(angular_position);
    double sin_angle = sin(angular_position);

    Eigen::Vector2d totalForceLocal = Eigen::Vector2d::Zero();
    double totalTorque = 0.0;

    for (Wheel* wheel : wheels) {
        Eigen::Vector2d wheelVelocityLocal = calculateWheelVelocityLocal(wheel->position);

        Eigen::Vector2d wheelForceLocal = wheel->calculateFriction(wheelVelocityLocal, Constants::TIME_INTERVAL);

        double torque = wheel->position.x() * wheelForceLocal.y() - wheel->position.y() * wheelForceLocal.x();

        Eigen::Vector2d wheelVelocityWorld(
            wheelVelocityLocal.x() * cos_angle + wheelVelocityLocal.y() * sin_angle,
            -wheelVelocityLocal.x() * sin_angle + wheelVelocityLocal.y() * cos_angle
        );

        Eigen::Vector2d wheelForceWorld(
            wheelForceLocal.x() * cos_angle + wheelForceLocal.y() * sin_angle,
            -wheelForceLocal.x() * sin_angle + wheelForceLocal.y() * cos_angle
        );

        wheel->lastVelocity = wheelVelocityWorld;
        wheel->lastForce = wheelForceWorld / wheel->mass;

        totalForceLocal += wheelForceLocal;
        totalTorque += torque;
    }

    Eigen::Vector2d totalForceWorld(
        totalForceLocal.x() * cos_angle + totalForceLocal.y() * sin_angle,
        -totalForceLocal.x() * sin_angle + totalForceLocal.y() * cos_angle
    );

    addForce(totalForceWorld);
    addTorque(totalTorque);

    static int debugCounter = 0;
    if (std::abs(angular_velocity) > 0.1 && debugCounter++ % 15 == 0) {
        double cos_a = cos(angular_position);
        double sin_a = sin(angular_position);
        Eigen::Vector2d velocityLocal(
            velocity.x() * cos_a - velocity.y() * sin_a,
            velocity.x() * sin_a + velocity.y() * cos_a
        );

        std::cout << "\n=== DEBUG (Local Coords) ===\n";
        std::cout << "Car Angle: " << angular_position * Constants::RAD_TO_DEG << "°\n";
        std::cout << "Velocity Local: [" << velocityLocal.x() << ", " << velocityLocal.y() << "]\n";
        std::cout << "Force Local: [" << totalForceLocal.x() << ", " << totalForceLocal.y() << "]\n";
        std::cout << "Net Torque: " << totalTorque << "\n";
        std::cout << "AngVel: " << angular_velocity << " | Speed: " << velocity.norm() << " m/s\n";
    }
}

void Car::moveWheels() {
    for (Wheel* wheel : wheels) {
        wheel->incrementTime(Constants::TIME_INTERVAL);
    }
    applyForceFeedback();
}

void Car::updateLoadTransfer() {
    double cos_angle = cos(angular_position);
    double sin_angle = sin(angular_position);

    double ax_local = acceleration.x() * cos_angle - acceleration.y() * sin_angle;
    double ay_local = acceleration.x() * sin_angle + acceleration.y() * cos_angle;

    double wheelbase = Constants::WHEELBASE;
    double track_width = Constants::TRACK_WIDTH;
    double cg_height = Constants::CG_HEIGHT;
    double weight = Constants::CAR_WEIGHT;
    double nominal_load = weight / 4.0;

    double dFz_longitudinal = -mass * ax_local * cg_height / wheelbase;
    double dFz_lateral = -mass * ay_local * cg_height / track_width;

    frontLeft->normalForce = nominal_load + dFz_longitudinal - dFz_lateral;
    frontRight->normalForce = nominal_load + dFz_longitudinal + dFz_lateral;
    backLeft->normalForce = nominal_load - dFz_longitudinal - dFz_lateral;
    backRight->normalForce = nominal_load - dFz_longitudinal + dFz_lateral;

    for (Wheel* wheel : wheels) {
        wheel->normalForce = std::max(100.0, wheel->normalForce);
    }
}

void Car::drawCar(SDL_Renderer* renderer) {
    SDL_Texture* tex = getRectangleTexture(renderer);
    SDL_Rect rect{getPositionX(), getPositionY(), getWidth(), getHeight()};

    double angleDegrees = angular_position * Constants::RAD_TO_DEG;

    SDL_RenderCopyEx(renderer, tex, NULL, &rect, angleDegrees, NULL, SDL_FLIP_NONE);

    drawDebugVectors(renderer);
}

void Car::drawDebugVectors(SDL_Renderer* renderer) {
    if (!showDebugVectors) return;

    int centerX = getPositionX() + getWidth() / 2;
    int centerY = getPositionY() + getHeight() / 2;

    const double velocityScale = 5.0;
    const double accelScale = 20.0;

    if (velocity.norm() > 0.01) {
        int velEndX = centerX + static_cast<int>(velocity.x() * velocityScale);
        int velEndY = centerY - static_cast<int>(velocity.y() * velocityScale);

        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        SDL_RenderDrawLine(renderer, centerX, centerY, velEndX, velEndY);

        double angle = std::atan2(-velocity.y(), velocity.x());
        int arrowSize = 8;
        int arrow1X = velEndX - arrowSize * std::cos(angle - 0.5);
        int arrow1Y = velEndY - arrowSize * std::sin(angle - 0.5);
        int arrow2X = velEndX - arrowSize * std::cos(angle + 0.5);
        int arrow2Y = velEndY - arrowSize * std::sin(angle + 0.5);

        SDL_RenderDrawLine(renderer, velEndX, velEndY, arrow1X, arrow1Y);
        SDL_RenderDrawLine(renderer, velEndX, velEndY, arrow2X, arrow2Y);
    }

    if (acceleration.norm() > 0.01) {
        int accelEndX = centerX + static_cast<int>(acceleration.x() * accelScale);
        int accelEndY = centerY - static_cast<int>(acceleration.y() * accelScale);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderDrawLine(renderer, centerX, centerY, accelEndX, accelEndY);

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

SDL_Texture* Car::getRectangleTexture(SDL_Renderer* renderer) {
    if (carTexture == nullptr) {
        carTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                       SDL_TEXTUREACCESS_TARGET, width, height);
        SDL_SetTextureBlendMode(carTexture, SDL_BLENDMODE_BLEND);
        SDL_SetRenderTarget(renderer, carTexture);
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
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
    return steering_angle * Constants::STEERING_RACK;
}