
#ifndef SIMPLETRAFFICGAME_ENGINE_H
#define SIMPLETRAFFICGAME_ENGINE_H

#endif


class Engine
{
private:
    double rpm{0};
    double loadTorque{0};
    double engineTorque{0};
    double getVolumetricEfficiency();
    double getAirFlowRate(double throttle);
    double getAirFuelRatio();
    double getPowerGenerated(double throttle);


public:
    void updateRPM(double throttle);
    double getRPM();
    double calculateTorque(double throttle);
    double addLoadTorque(double torque);
};