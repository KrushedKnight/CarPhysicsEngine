#include "control/TractionControl.h"
#include <algorithm>

TractionControl::TractionControl(double kp, double kd)
    : kp(kp), kd(kd), interferencePercent(0.0) {}

double TractionControl::regulateTorque(Wheel& wheel, double requestedTorque,
                                       double slipSetpoint,
                                       const Eigen::Vector2d& wheelVelocityLocal,
                                       double dt) {
    double slipRatio = wheel.calculateSlipRatio(wheelVelocityLocal);
    double error = slipSetpoint - slipRatio;
    double changeInSlip = slipRatio - wheel.previousSlipError;

    double adjustedTorque = requestedTorque + kp * error - kd * changeInSlip;

    double reduction = requestedTorque - adjustedTorque;
    if (reduction > 0) {
        interferencePercent = (reduction / requestedTorque) * 100.0;
    } else {
        interferencePercent = 0.0;
    }

    wheel.previousSlipError = slipRatio;
    wheel.tcsInterference = interferencePercent;

    return adjustedTorque;
}

void TractionControl::reset() {
    interferencePercent = 0.0;
}
