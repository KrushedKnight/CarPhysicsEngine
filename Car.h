//
// Created by beast-machine-2 on 7/2/25.
//

#ifndef CAR_H
#define CAR_H
#include <SDL_rect.h>
#include <SDL_render.h>
#include <eigen-3.4.0/Eigen/Core>

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

        std::vector<Wheel*> wheels{frontLeft, frontRight, backLeft, backRight};


        const int getWidth();
        const int getHeight();


        void drawCar(SDL_Renderer* renderer);
        void eraseCar(SDL_Renderer *renderer);

        void applySteering(double amount);
        void applyEngineTorque();

        void applyBrakes();

        void sumWheelForces();

    private:
        const double width;
        const double height;
};



#endif //CAR_H
