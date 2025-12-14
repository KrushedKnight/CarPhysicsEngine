#ifndef CAR_H
#define CAR_H
#include <SDL_rect.h>
#include <SDL_render.h>
#include <Eigen/Core>

#include "RigidBody.h"
#include "Wheel.h"

class Car : public RigidBody {
    public:
        Car(double x, double y, int w, int h);
        ~Car();

        double steering_angle{0};

        double engine_power{Constants::CAR_POWER};
        double braking_power{Constants::BRAKING_POWER};

        bool showDebugVectors{true};
        double previous_slip{0};

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
        void applyEngineTorque();
        void applyBrakes();
        void applyForceFeedback();

        void sumWheelForces();
        void moveWheels();
        void updateLoadTransfer();

    private:
        const double width;
        const double height;
        SDL_Texture* carTexture{nullptr};

        SDL_Texture* getRectangleTexture(SDL_Renderer *renderer);
};

#endif