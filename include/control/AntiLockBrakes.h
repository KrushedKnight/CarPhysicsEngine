#ifndef ANTILOCKBRAKES_H
#define ANTILOCKBRAKES_H

#include "vehicle/Wheel.h"
#include <Eigen/Dense>

class AntiLockBrakes {
public:
    AntiLockBrakes(double kp, double kd);

    double regulateBrakePressure(Wheel& wheel, double requestedBrakeTorque,
                                 double slipSetpoint,
                                 const Eigen::Vector2d& wheelVelocityLocal,
                                 double vehicleSpeed, double dt);

    double getInterferencePercent() const { return interferencePercent; }

    void reset();

private:
    double kp;
    double kd;
    double interferencePercent;
};

#endif
