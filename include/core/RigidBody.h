#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include <Eigen/Dense>
#include <map>
#include <string>

class Camera;

class RigidBody {
public:
    double pos_x;
    double pos_y;

    Eigen::Vector2d velocity;
    Eigen::Vector2d acceleration;
    Eigen::Vector2d forces;
    std::map<std::string, Eigen::Vector2d> namedForces;

    double angular_position;
    double angular_velocity;
    double angular_acceleration;
    double angular_torque;

    double mass;
    double moment_of_inertia;

    RigidBody();

    int getPositionX(const Camera* camera = nullptr, int screenWidth = 0);
    int getPositionY(const Camera* camera = nullptr, int screenHeight = 0);

    void addForce(Eigen::Vector2d force, const std::string& name = "");

    void addTorque(double torque);

    const std::map<std::string, Eigen::Vector2d>& getNamedForces() const;

    void clearForces();
    void clearTorques();

    void updateAcceleration();

    void incrementTime(double time_interval);
};

#endif
