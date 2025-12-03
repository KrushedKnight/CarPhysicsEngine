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

        double steering_angle{0};

        double engine_power{Constants::CAR_POWER};
        double braking_power{Constants::BRAKING_POWER};

        Wheel* frontLeft;

        Wheel* frontRight;

        Wheel* backLeft;

        Wheel* backRight;

        std::vector<Wheel*> wheels;

        SDL_Texture *getRectangleTexture(SDL_Renderer *renderer);


        const int getWidth();

        const int getHeight();


        void drawCar(SDL_Renderer* renderer);

        void eraseCar(SDL_Renderer *renderer);

        double getAngleToWheel(Wheel *wheel);

        void applySteering(double amount);
        void applyEngineTorque();

        void applyBrakes();

        void sumWheelForces();

        void moveWheels();

    private:
        const double width;
        const double height;
};



#endif //CAR_H
