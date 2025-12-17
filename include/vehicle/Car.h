#ifndef CAR_H
#define CAR_H
#include <SDL_rect.h>
#include <SDL_render.h>
#include <Eigen/Core>

#include "core/RigidBody.h"
#include "vehicle/Wheel.h"
#include "vehicle/Engine.h"
#include "vehicle/Gearbox.h"
#include "control/TractionControl.h"
#include "control/AntiLockBrakes.h"

class Car : public RigidBody {
    public:
        Car(double x, double y, int w, int h);
        ~Car();

        double steering_angle{0};

        double engine_power{PhysicsConstants::CAR_POWER};
        double braking_power{PhysicsConstants::BRAKING_POWER};

        bool showDebugVectors{true};

        Wheel* frontLeft;
        Wheel* frontRight;
        Wheel* backLeft;
        Wheel* backRight;

        std::vector<Wheel*> wheels;

        const int getWidth();
        const int getHeight();

        void drawCar(SDL_Renderer* renderer);
        void drawDebugVectors(SDL_Renderer* renderer);
        void eraseCar(SDL_Renderer *renderer);

        double getAngleToWheel(Wheel *wheel);

        void applySteering(double amount);
        void applyEngineTorque(double throttle);
        void applyBrakes();
        void applyForceFeedback();

        void shiftUp();
        void shiftDown();
        void holdClutch();
        void releaseClutch();

        int getCurrentGear() const;
        bool isClutchHeld() const;

        void sumWheelForces();
        void moveWheels();
        void updateLoadTransfer();

    private:
        const double width;
        const double height;
        SDL_Texture* carTexture{nullptr};

        Engine engine;
        Gearbox gearbox;
        TractionControl tcs;
        AntiLockBrakes abs;

        SDL_Texture* getRectangleTexture(SDL_Renderer *renderer);

        Eigen::Vector2d calculateWheelVelocityLocal(Eigen::Vector2d wheelPosition);
};

#endif
