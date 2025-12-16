#ifndef TRACTIONCONTROL_H
#define TRACTIONCONTROL_H

#include "vehicle/Wheel.h"
#include <Eigen/Dense>

class TractionControl {
public:
    TractionControl(double kp, double kd);

    double regulateTorque(Wheel& wheel, double requestedTorque, double slipSetpoint,
                         const Eigen::Vector2d& wheelVelocityLocal, double dt);

    double getInterferencePercent() const { return interferencePercent; }

    void reset();

private:
    double kp;
    double kd;
    double interferencePercent;
};

#endif
