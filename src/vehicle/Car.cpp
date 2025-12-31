#include "vehicle/Car.h"
#include "config/PhysicsConstants.h"
#include "config/RenderingConstants.h"
#include "config/EngineConstants.h"
#include "rendering/Camera.h"
#include <cmath>
#include <iostream>

Car::Car(double x, double y, int w, int h)
    : width(w), height(h),
      gearbox({3.5, 2.2, 1.5, 1.0, 0.75, 0.6}, 4.2),
      tcs(PhysicsConstants::TIRE_TCS_kP, PhysicsConstants::TIRE_TCS_kD),
      abs(PhysicsConstants::ABS_kP, PhysicsConstants::ABS_kD) {
    pos_x = x;
    pos_y = y;

    double halfWidth = (RenderingConstants::CAR_WIDTH / 10.0) / 2.0;
    double halfLength = (RenderingConstants::CAR_LENGTH / 10.0) / 2.0;

    frontLeft = new Wheel();
    frontLeft->position = Eigen::Vector2d(-halfWidth + RenderingConstants::WHEEL_WIDTH_INSET,
                                           halfLength - RenderingConstants::WHEEL_LENGTH_INSET);

    frontRight = new Wheel();
    frontRight->position = Eigen::Vector2d(halfWidth - RenderingConstants::WHEEL_WIDTH_INSET,
                                            halfLength - RenderingConstants::WHEEL_LENGTH_INSET);

    backLeft = new Wheel();
    backLeft->position = Eigen::Vector2d(-halfWidth + RenderingConstants::WHEEL_WIDTH_INSET,
                                          -halfLength + RenderingConstants::WHEEL_LENGTH_INSET);

    backRight = new Wheel();
    backRight->position = Eigen::Vector2d(halfWidth - RenderingConstants::WHEEL_WIDTH_INSET,
                                           -halfLength + RenderingConstants::WHEEL_LENGTH_INSET);

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
    double speed = velocity.norm();
    double maxSpeed = 50.0;
    double speedFactor = 1.0 - (speed / maxSpeed) * 0.5;
    speedFactor = std::max(0.5, speedFactor);

    steering_angle += amount * speedFactor;
    steering_angle = std::clamp(steering_angle, -PhysicsConstants::MAX_STEERING_ANGLE, PhysicsConstants::MAX_STEERING_ANGLE);

    double wheelbase = RenderingConstants::WHEELBASE;
    double trackWidth = RenderingConstants::TRACK_WIDTH;
    double steeringRack = PhysicsConstants::STEERING_RACK;
    double baseAngle = steering_angle * steeringRack;

    if (std::abs(baseAngle) < 0.001) {
        frontLeft->wheelAngle = 0.0;
        frontRight->wheelAngle = 0.0;
    } else {
        double turnRadius = wheelbase / std::tan(std::abs(baseAngle));

        if (steering_angle > 0) {
            double innerRadius = turnRadius - trackWidth / 2.0;
            double outerRadius = turnRadius + trackWidth / 2.0;
            frontLeft->wheelAngle = std::atan(wheelbase / innerRadius);
            frontRight->wheelAngle = std::atan(wheelbase / outerRadius);
        } else {
            double innerRadius = turnRadius - trackWidth / 2.0;
            double outerRadius = turnRadius + trackWidth / 2.0;
            frontLeft->wheelAngle = -std::atan(wheelbase / outerRadius);
            frontRight->wheelAngle = -std::atan(wheelbase / innerRadius);
        }
    }
}

void Car::applyForceFeedback()
{
    steering_angle *= PhysicsConstants::FORCE_FEEDBACK_DECAY;

    double wheelbase = RenderingConstants::WHEELBASE;
    double trackWidth = RenderingConstants::TRACK_WIDTH;
    double steeringRack = PhysicsConstants::STEERING_RACK;
    double baseAngle = steering_angle * steeringRack;

    if (std::abs(baseAngle) < 0.001) {
        frontLeft->wheelAngle = 0.0;
        frontRight->wheelAngle = 0.0;
    } else {
        double turnRadius = wheelbase / std::tan(std::abs(baseAngle));

        if (steering_angle > 0) {
            double innerRadius = turnRadius - trackWidth / 2.0;
            double outerRadius = turnRadius + trackWidth / 2.0;
            frontLeft->wheelAngle = std::atan(wheelbase / innerRadius);
            frontRight->wheelAngle = std::atan(wheelbase / outerRadius);
        } else {
            double innerRadius = turnRadius - trackWidth / 2.0;
            double outerRadius = turnRadius + trackWidth / 2.0;
            frontLeft->wheelAngle = -std::atan(wheelbase / outerRadius);
            frontRight->wheelAngle = -std::atan(wheelbase / innerRadius);
        }
    }
}

void Car::updateEngine(double throttle) {
    gearbox.update();
    engine.calculateTorque(actualThrottle);

    Wheel* rearWheels[] = {backLeft, backRight};

    double avgWheelOmega = (backLeft->angular_velocity + backRight->angular_velocity) / 2.0;

    std::cout << "RPM: " << engine.getRPM() << " | WheelOmega: " << avgWheelOmega
            << " | ClutchTorque: " << gearbox.getClutchTorque() << std::endl;



    double totalWheelTorque = gearbox.convertEngineTorqueToWheel(engine.getEngineTorque(), &engine, avgWheelOmega);
    double baseTorque = totalWheelTorque;

    double reflectedInertia = gearbox.getReflectedEngineInertia(EngineConstants::ENGINE_MOMENT_OF_INERTIA);
    double wheelInertia = PhysicsConstants::WHEEL_MOMENT_OF_INERTIA;
    double effectiveInertia = wheelInertia + reflectedInertia;
    double inertiaCorrection = wheelInertia / effectiveInertia;

    static int inertiaDebugCounter = 0;
    if (inertiaDebugCounter++ % 30 == 0 && gearbox.getCurrentGear() >= 0) {
        std::cout << "\n=== REFLECTED INERTIA ===" << std::endl;
        std::cout << "Wheel I: " << wheelInertia << " kg·m² | Reflected Engine I: " << reflectedInertia << " kg·m²" << std::endl;
        std::cout << "Effective I: " << effectiveInertia << " kg·m² | Correction: " << inertiaCorrection << std::endl;
        std::cout << "Torque Before: " << (baseTorque / inertiaCorrection) << " Nm | After: " << (baseTorque / inertiaCorrection * inertiaCorrection) << " Nm" << std::endl;
        std::cout << "========================" << std::endl;
    }

    baseTorque *= inertiaCorrection;
    baseTorque *= 1.25;

    double reflectedWheelInertia = gearbox.getReflectedWheelInertia(PhysicsConstants::WHEEL_MOMENT_OF_INERTIA * 2.0);
    double effectiveEngineInertia = (EngineConstants::ENGINE_MOMENT_OF_INERTIA + reflectedWheelInertia) * 0.12;

    engine.addLoadTorque(gearbox.getClutchTorque());
    engine.updateRPM(throttle, effectiveEngineInertia);

    if (engine.getRPM() < 800.0 && getCurrentGear() != -1) {
        bool wasClutchHeld = gearbox.isClutchHeld();
        if (!wasClutchHeld) {
            gearbox.holdClutch();
        }
        while (getCurrentGear() > -1) {
            gearbox.shiftDown();
        }
        if (!wasClutchHeld) {
            gearbox.releaseClutch();
        }
    }

    for (Wheel* wheel : rearWheels) {
        if (wheel->angular_velocity * wheel->wheelRadius >= PhysicsConstants::CAR_TOP_SPEED)
        {
            wheel->tcsInterference = 0.0;
        }
        Eigen::Vector2d wheelVelocityLocal = calculateWheelVelocityLocal(wheel->position);
        double adjustedTorque = tcs.regulateTorque(
            *wheel,
            baseTorque,
            PhysicsConstants::TIRE_SLIP_SETPOINT,
            wheelVelocityLocal,
            PhysicsConstants::TIME_INTERVAL
        );

        if (engine.getRPM() >= 8000.0 && adjustedTorque > 0.0) {
            adjustedTorque = 0.0;
        }

        wheel->addTorque(adjustedTorque);
    }

}

void Car::applyBrakes() {
    for (Wheel* wheel : wheels) {
        Eigen::Vector2d wheelVelocityLocal = calculateWheelVelocityLocal(wheel->position);
        Eigen::Vector2d wheelForward{sin(wheel->wheelAngle), cos(wheel->wheelAngle)};
        double vehicleSpeed = std::abs(wheelVelocityLocal.dot(wheelForward));

        double requestedBrakeTorque = braking_power * actualBrake * wheel->wheelRadius;
        double adjustedBrakeTorque = abs.regulateBrakePressure(
            *wheel,
            requestedBrakeTorque,
            PhysicsConstants::ABS_SLIP_SETPOINT,
            wheelVelocityLocal,
            vehicleSpeed,
            PhysicsConstants::TIME_INTERVAL
        );

        wheel->addTorque(adjustedBrakeTorque);
    }
}

void Car::setThrottle(double throttle) {
    targetThrottle = std::clamp(throttle, 0.0, 1.0);
}

void Car::setBrake(double brake) {
    targetBrake = std::clamp(brake, 0.0, 1.0);
}

void Car::setSteering(double steering) {
    targetSteering = std::clamp(steering, -1.0, 1.0);
}

void Car::updateInputs(double timeInterval) {
    const double throttleRate = 6.0;
    const double brakeRate = 9.0;
    const double steeringRate = 7.0;

    double throttleDiff = targetThrottle - actualThrottle;
    double throttleChange = std::clamp(throttleDiff, -throttleRate * timeInterval, throttleRate * timeInterval);
    actualThrottle += throttleChange;
    actualThrottle = std::clamp(actualThrottle, 0.0, 1.0);

    double brakeDiff = targetBrake - actualBrake;
    double brakeChange = std::clamp(brakeDiff, -brakeRate * timeInterval, brakeRate * timeInterval);
    actualBrake += brakeChange;
    actualBrake = std::clamp(actualBrake, 0.0, 1.0);

    double steeringDiff = targetSteering - actualSteering;
    double steeringChange = std::clamp(steeringDiff, -steeringRate * timeInterval, steeringRate * timeInterval);
    actualSteering += steeringChange;
    actualSteering = std::clamp(actualSteering, -1.0, 1.0);

    double targetSteeringAngle = actualSteering * PhysicsConstants::MAX_STEERING_ANGLE;
    double speed = velocity.norm();
    double maxSpeed = 50.0;
    double speedFactor = 1.0 - (speed / maxSpeed) * 0.5;
    speedFactor = std::max(0.5, speedFactor);
    targetSteeringAngle *= speedFactor;

    steering_angle = targetSteeringAngle;

    double wheelbase = RenderingConstants::WHEELBASE;
    double trackWidth = RenderingConstants::TRACK_WIDTH;
    double steeringRack = PhysicsConstants::STEERING_RACK;
    double baseAngle = steering_angle * steeringRack;

    if (std::abs(baseAngle) < 0.001) {
        frontLeft->wheelAngle = 0.0;
        frontRight->wheelAngle = 0.0;
    } else {
        double turnRadius = wheelbase / tan(std::abs(baseAngle));
        double innerRadius = turnRadius - trackWidth / 2.0;
        double outerRadius = turnRadius + trackWidth / 2.0;

        double innerAngle = atan(wheelbase / innerRadius);
        double outerAngle = atan(wheelbase / outerRadius);

        if (baseAngle > 0) {
            frontLeft->wheelAngle = innerAngle;
            frontRight->wheelAngle = outerAngle;
        } else {
            frontLeft->wheelAngle = -outerAngle;
            frontRight->wheelAngle = -innerAngle;
        }
    }
}

void Car::sumWheelForces() {
    updateLoadTransfer();

    double cos_angle = cos(angular_position);
    double sin_angle = sin(angular_position);

    Eigen::Vector2d totalForceLocal = Eigen::Vector2d::Zero();
    double totalTorque = 0.0;

    const char* wheelNames[] = {"FL Friction", "FR Friction", "RL Friction", "RR Friction"};
    int wheelIndex = 0;

    for (Wheel* wheel : wheels) {
        Eigen::Vector2d wheelVelocityLocal = calculateWheelVelocityLocal(wheel->position);

        Eigen::Vector2d wheelForceLocal = wheel->calculateFriction(wheelVelocityLocal, PhysicsConstants::TIME_INTERVAL);

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

        addForce(wheelForceWorld, wheelNames[wheelIndex++]);

        totalForceLocal += wheelForceLocal;
        totalTorque += torque;
    }

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
        std::cout << "Car Angle: " << angular_position * PhysicsConstants::RAD_TO_DEG << "°\n";
        std::cout << "Velocity Local: [" << velocityLocal.x() << ", " << velocityLocal.y() << "]\n";
        std::cout << "Force Local: [" << totalForceLocal.x() << ", " << totalForceLocal.y() << "]\n";
        std::cout << "Net Torque: " << totalTorque << "\n";
        std::cout << "AngVel: " << angular_velocity << " | Speed: " << velocity.norm() << " m/s\n";
    }
}

void Car::moveWheels() {
    for (Wheel* wheel : wheels) {
        wheel->incrementTime(PhysicsConstants::TIME_INTERVAL);
    }
    applyForceFeedback();
}

void Car::updateLoadTransfer() {
    double cos_angle = cos(angular_position);
    double sin_angle = sin(angular_position);

    double ax_local = acceleration.x() * cos_angle - acceleration.y() * sin_angle;
    double ay_local = acceleration.x() * sin_angle + acceleration.y() * cos_angle;

    double wheelbase = RenderingConstants::WHEELBASE;
    double track_width = RenderingConstants::TRACK_WIDTH;
    double cg_height = PhysicsConstants::CG_HEIGHT;
    double weight = PhysicsConstants::CAR_WEIGHT;

    double frontWeightBias = 0.6;
    double rearWeightBias = 0.4;
    double frontNominalLoad = (weight * frontWeightBias) / 2.0;
    double rearNominalLoad = (weight * rearWeightBias) / 2.0;

    double dFz_longitudinal = -mass * ay_local * cg_height / wheelbase;
    double dFz_lateral = -mass * ax_local * cg_height / track_width;

    frontLeft->normalForce = frontNominalLoad + dFz_longitudinal - dFz_lateral;
    frontRight->normalForce = frontNominalLoad + dFz_longitudinal + dFz_lateral;
    backLeft->normalForce = rearNominalLoad - dFz_longitudinal - dFz_lateral;
    backRight->normalForce = rearNominalLoad - dFz_longitudinal + dFz_lateral;

    for (Wheel* wheel : wheels) {
        wheel->normalForce = std::max(60.0, wheel->normalForce);
    }
}

void Car::drawCar(SDL_Renderer* renderer, const Camera* camera) {
    SDL_Texture* tex = getRectangleTexture(renderer);
    SDL_Rect rect{
        getPositionX(camera, RenderingConstants::SDL_WINDOW_WIDTH),
        getPositionY(camera, RenderingConstants::SDL_WINDOW_LENGTH),
        getWidth(),
        getHeight()
    };

    double angleDegrees = angular_position * PhysicsConstants::RAD_TO_DEG;

    SDL_RenderCopyEx(renderer, tex, NULL, &rect, angleDegrees, NULL, SDL_FLIP_NONE);

    double cos_angle = cos(angular_position);
    double sin_angle = sin(angular_position);

    int carCenterX = rect.x + rect.w / 2;
    int carCenterY = rect.y + rect.h / 2;

    for (Wheel* wheel : wheels) {
        double worldX = pos_x + wheel->position.x() * cos_angle - wheel->position.y() * sin_angle;
        double worldY = pos_y + wheel->position.x() * sin_angle + wheel->position.y() * cos_angle;

        wheel->pos_x = worldX;
        wheel->pos_y = worldY;

        double wheelLocalPixelX = wheel->position.x() * 10;
        double wheelLocalPixelY = wheel->position.y() * 10;

        int wheelScreenX = carCenterX + static_cast<int>(wheelLocalPixelX * cos_angle - wheelLocalPixelY * sin_angle);
        int wheelScreenY = carCenterY + static_cast<int>(wheelLocalPixelX * sin_angle + wheelLocalPixelY * cos_angle);

        int tireRadius = static_cast<int>(PhysicsConstants::WHEEL_RADIUS * 10);
        SDL_Rect tireRect = {
            wheelScreenX - tireRadius,
            wheelScreenY - tireRadius,
            tireRadius * 2,
            tireRadius * 2
        };

        SDL_SetRenderDrawColor(renderer, 40, 40, 40, 255);
        SDL_RenderFillRect(renderer, &tireRect);
    }

    drawDebugVectors(renderer, camera);
}

void Car::drawDebugVectors(SDL_Renderer* renderer, const Camera* camera) {
    if (!showDebugVectors) return;

    int centerX = getPositionX(camera, RenderingConstants::SDL_WINDOW_WIDTH) + getWidth() / 2;
    int centerY = getPositionY(camera, RenderingConstants::SDL_WINDOW_LENGTH) + getHeight() / 2;

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

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
        SDL_RenderClear(renderer);

        int w = static_cast<int>(width);
        int h = static_cast<int>(height);

        SDL_SetRenderDrawColor(renderer, 180, 180, 180, 255);
        SDL_Rect body = {0, 0, w, h};
        SDL_RenderFillRect(renderer, &body);

        SDL_SetRenderDrawColor(renderer, 220, 220, 220, 255);
        SDL_Rect frontIndicator = {0, 0, w, h/10};
        SDL_RenderFillRect(renderer, &frontIndicator);

        SDL_SetRenderTarget(renderer, NULL);
    }
    return carTexture;
}

void Car::eraseCar(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
}

double Car::getAngleToWheel(Wheel* wheel) {
    return steering_angle * PhysicsConstants::STEERING_RACK;
}

void Car::shiftUp() {
    gearbox.shiftUp();
}

void Car::shiftDown() {
    gearbox.shiftDown();
}

void Car::holdClutch() {
    gearbox.holdClutch();
}

void Car::releaseClutch() {
    gearbox.releaseClutch();
}

int Car::getCurrentGear() const {
    return gearbox.getCurrentGear();
}

bool Car::isClutchHeld() const {
    return gearbox.isClutchHeld();
}

const Engine& Car::getEngine() const {
    return engine;
}

const Gearbox& Car::getGearbox() const {
    return gearbox;
}
