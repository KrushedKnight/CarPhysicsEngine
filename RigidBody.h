//
// Created by beast-machine-2 on 7/6/25.
//

#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include <Eigen/Dense>


class RigidBody {
public:
    double pos_x;
    double pos_y;


    Eigen::Vector2d velocity;
    Eigen::Vector2d acceleration;
    Eigen::Vector2d forces;

    double angular_position; //from -180 to 180
    double angular_velocity;
    double angular_acceleration;
    double angular_torque;

    //TODO move constants

    double mass;
    double moment_of_inertia;

    RigidBody();

    int getPositionX();
    int getPositionY();


    void addForce(Eigen::Vector2d force);

    void addTorque(double torque);

    void clearForces();
    void clearTorques();


    void incrementTime(double time_interval);
};



#endif //RIGIDBODY_H
