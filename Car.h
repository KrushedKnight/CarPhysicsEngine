//
// Created by beast-machine-2 on 7/2/25.
//

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
        ~Car();  // Destructor to clean up wheels

        double steering_angle{0};  // Current steering angle (radians)

        double engine_power{Constants::CAR_POWER};
        double braking_power{Constants::BRAKING_POWER};

        Wheel* frontLeft;
        Wheel* frontRight;
        Wheel* backLeft;
        Wheel* backRight;

        std::vector<Wheel*> wheels;

        const int getWidth();
        const int getHeight();

        void drawCar(SDL_Renderer* renderer);
        void eraseCar(SDL_Renderer *renderer);

        double getAngleToWheel(Wheel *wheel);

        void applySteering(double amount);  // amount in radians
        void applyEngineTorque();
        void applyBrakes();
        void applyForceFeedback();

        void sumWheelForces();
        void moveWheels();

    private:
        const double width;
        const double height;
        SDL_Texture* carTexture{nullptr};  // Cached texture to prevent memory leak

        SDL_Texture* getRectangleTexture(SDL_Renderer *renderer);
};



#endif //CAR_H
