
#ifndef SIMPLETRAFFICGAME_ENGINE_H
#define SIMPLETRAFFICGAME_ENGINE_H

#endif


class Engine
{
private:
    double rpm;
    double getVolumetricEfficiency();
    double getAirFlowRate(double throttle);
    double getAirFuelRatio();
    double getPowerGenerated(double throttle);


public:
    void updateRPM(double throttle, double loadTorque);
    double getRPM();
    double calculateTorque(double throttle);
};