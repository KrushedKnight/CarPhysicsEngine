#include "control/AntiLockBrakes.h"
#include <cmath>
#include <algorithm>

AntiLockBrakes::AntiLockBrakes(double kp, double kd)
    : kp(kp), kd(kd), interferencePercent(0.0) {}

double AntiLockBrakes::regulateBrakePressure(Wheel& wheel, double requestedBrakeTorque,
                                             double slipSetpoint,
                                             const Eigen::Vector2d& wheelVelocityLocal,
                                             double vehicleSpeed, double dt) {
    if (std::abs(wheel.angular_velocity) < 1e-3) {
        wheel.angular_velocity = 0.0;
        wheel.absInterference = 0.0;
        interferencePercent = 0.0;
        return 0.0;
    }

    if (vehicleSpeed < 0.1) {
        double baseBrakeTorque = -std::abs(requestedBrakeTorque) * std::copysign(1.0, wheel.angular_velocity);
        wheel.absInterference = 0.0;
        interferencePercent = 0.0;
        return baseBrakeTorque;
    }

    double slipRatio = wheel.calculateSlipRatio(wheelVelocityLocal);
    double error = slipSetpoint - slipRatio;
    double changeInSlip = slipRatio - wheel.previousAbsSlipError;

    double baseBrakeTorque = -std::abs(requestedBrakeTorque) * std::copysign(1.0, wheel.angular_velocity);
    double adjustedBrakeTorque = baseBrakeTorque + kp * error - kd * changeInSlip;

    if (wheel.angular_velocity > 0 && adjustedBrakeTorque > 0) {
        adjustedBrakeTorque = 0.0;
    } else if (wheel.angular_velocity < 0 && adjustedBrakeTorque < 0) {
        adjustedBrakeTorque = 0.0;
    }

    double reduction = std::abs(baseBrakeTorque) - std::abs(adjustedBrakeTorque);
    if (reduction > 0 && std::abs(baseBrakeTorque) > 1e-6) {
        interferencePercent = (reduction / std::abs(baseBrakeTorque)) * 100.0;
    } else {
        interferencePercent = 0.0;
    }

    wheel.previousAbsSlipError = slipRatio;
    wheel.absInterference = interferencePercent;

    return adjustedBrakeTorque;
}

void AntiLockBrakes::reset() {
    interferencePercent = 0.0;
}
